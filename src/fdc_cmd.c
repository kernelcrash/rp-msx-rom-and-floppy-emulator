// The core 1 half of the WD2793 emulation. See fdc.h for the split.
//
// Called repeatedly from core 1's idle points (no interrupt): each call advances one small step,
// so it never holds up the renderer for more than one sector's worth of card access. It replays the
// register writes core 0 pushes into the event ring into a model of the registers, executes the
// commands (seek, sector setup), keeps each drive's cylinder buffer loaded, and writes changed sectors
// back to the disk image. It never writes a register: it posts the model as an image and core 0
// copies it in.
#include "fdc.h"
#include <string.h>
#include "floppy.h"

// The least time a type I command (restore, seek, step) stays BUSY, in ms: 0 = none (finished as soon as it
// is carried out). The STM32 project waits 1 ms after every seek's track load (delay_ms(1)); -DFDC_SEEK_MIN_MS=1
// does the same here. The clock has 1 ms steps, so n means between n and n + 1 ms.
#ifndef FDC_SEEK_MIN_MS
#define FDC_SEEK_MIN_MS 0
#endif

struct snapshot {
    uint32_t lock;
    uint8_t command, data, track, sector;
    uint16_t system;
};


void fdc_cmd_init(struct fdc_cmd *c, struct fdc_link *link, struct floppy *floppy, uint8_t *cyl0, uint8_t *cyl1) {
    c->link = link;
    c->floppy = floppy;
    c->cyl[0] = cyl0;
    c->cyl[1] = cyl1;
    c->state = FDC_IDLE;
    memset(&c->m, 0, sizeof c->m);
    c->m.status = F_TRACK0;
    c->msys = 0;
    c->tail = 0;
    c->cancelled = false;
    c->img_dirty = false;
    c->started = false;
    c->serial = 0;
    c->images = 0;
    c->now_ms = c->cmd_start_ms = 0;
    c->settle_status = 0;
    c->cur_lock = 0;
    c->cur_drive = 0;
    c->target = 0;
    for(int d = 0; d < FDC_MAX_DRIVES; d++) {
        c->actual[d] = 0;
        c->valid[d] = false;
        c->load_tried[d] = false;
        for(int i = 0; i < FDC_SECTORS_PER_CYL; i++)
            c->flushed[d][i] = 0;
    }
    c->op_sector = -1;
    c->op_count = 0;
    c->op_fails = 0;
    c->last_write_total = 0;
    c->last_write_ms = 0;
    c->retry_after_ms = 0;
    c->stats = (struct fdc_stats){0};
}


// ---- the model, and the image sent to core 0

// Put a finished command's outcome into the model. A command that was cancelled by a Force Interrupt while it
// was being carried out (a seek cannot be stopped half way) only leaves its mark on the track register, the
// head position and the step direction: the status and IRQ were settled by the Force Interrupt.
static void apply_result(struct fdc_cmd *c, const struct fdc_result *r, bool cancelled) {
    if(!cancelled) {
        c->m.status = r->status;
        c->m.irq_drq = r->irq_drq;
        if(r->flags & RES_SET_INTENDED)
            c->m.intended = r->intended;
        c->m.ptr = r->ptr;
        c->m.read_len = r->read_len;
        c->m.write_len = r->write_len;
    } else if(r->flags & RES_SET_INTENDED) {
        c->m.intended = r->intended;
    }
    if(r->flags & RES_SET_TRACK)
        c->m.track = r->track;
    if(r->flags & RES_SET_LASTSTEP)
        c->msys = (uint16_t)((c->msys & ~S_LASTSTEP) | (r->laststep ? S_LASTSTEP : 0));
    c->img_dirty = true;
}

// Send the model to core 0, if core 0 has written nothing that this has not seen. (If it has, the image would
// be thrown away: wait until the events are consumed, and send it then.)
static void post_image(struct fdc_cmd *c) {
    struct fdc_img *m = &c->link->img;

    if(!c->img_dirty)
        return;
    if(__atomic_load_n(&c->link->ev_head, __ATOMIC_ACQUIRE) != c->tail)
        return;

    // registers 0-7 as staged: the data register shows the next byte of a transfer
    const uint8_t data = c->m.read_len ? c->cyl[c->msys & S_DRIVE0][c->m.ptr] : c->m.data;
    const uint32_t w4 = FDC_STATUS_BYTE(c->m.status) | (uint32_t)c->m.track << 8 | (uint32_t)c->m.sector << 16 | (uint32_t)data << 24;
    const uint32_t w5 = 0x00ffffffu | (uint32_t)fdc_reg7(c->m.irq_drq) << 24;
    uint32_t w[4];
    memcpy(w, &c->m, sizeof w);

    const uint32_t lock = m->lock;
    m->lock = lock + 1;
    FDC_BARRIER();
    FDC_HOOK(4);
    m->consumed = c->tail;
    m->w[0] = w[0];
    m->w[1] = w[1];
    FDC_HOOK(5);
    m->w[2] = w[2];
    m->w[3] = w[3];
    m->w[4] = w4;
    m->w[5] = w5;
    FDC_BARRIER();
    FDC_HOOK(6);
    m->lock = lock + 2;
    c->img_dirty = false;
    c->images++;
}


// ---- sectors that core 0 has written and this side has not yet copied back

static int find_dirty(const struct fdc_cmd *c, unsigned drive) {
    const struct floppy_drive *d = &c->floppy->drive[drive];
    if(!d->present || d->read_only || !c->valid[drive])
        return -1;
    const unsigned n = d->sides * FDC_SECTORS_PER_SIDE;
    for(unsigned i = 0; i < n; i++) {
        if(c->link->written[drive][i] != c->flushed[drive][i])
            return (int)i;
    }
    return -1;
}

// Start copying sector `index` back. The count is sampled first: if core 0 rewrites the sector
// while the copy is in progress the counts differ afterwards and it is copied again.
static bool start_flush(struct fdc_cmd *c, unsigned drive, unsigned index) {
    c->op_sector = (int)index;
    c->op_count = c->link->written[drive][index];
    return floppy_start_write(c->floppy, drive, c->actual[drive], index, c->cyl[drive] + index * FDC_SECTOR_BYTES);
}


// ---- type I: restore, seek, step

// Work out what a type I command does. Fills c->proto (the result, less BUSY handling) and c->target.
static void plan_type1(struct fdc_cmd *c, const struct snapshot *s) {
    const uint8_t cmd = s->command;
    const uint8_t kind = cmd >> 4;
    const unsigned drive = s->system & S_DRIVE0;
    const int actual = c->actual[drive];
    struct fdc_result *p = &c->proto;
    int intended, target;

    p->flags = RES_SET_INTENDED;
    p->laststep = 0;
    p->track = 0;

    if(kind == 0) {  // restore
        intended = 0;
        target = 0;
        p->flags |= RES_SET_TRACK;
        p->track = 0;
    } else if(kind == 1) {  // seek: the data register holds the destination
        intended = s->data;
        target = intended + actual - s->track;
        p->flags |= RES_SET_TRACK;
        p->track = s->data;
    } else {  // step, step in, step out (each with or without updating the track register)
        bool out;
        if(kind >= 4) {
            out = (cmd & 0x20) != 0;  // step in/out give the direction and remember it
            p->flags |= RES_SET_LASTSTEP;
            p->laststep = out;
        } else {
            out = (s->system & S_LASTSTEP) != 0;  // plain step repeats the last direction
        }
        intended = s->track + (out ? -1 : 1);
        if(intended < 0)
            intended = 0;
        if(intended > 255)
            intended = 255;
        target = intended + actual - s->track;
        if(cmd & C_SETTRACK) {
            p->flags |= RES_SET_TRACK;
            p->track = (uint8_t)intended;
        }
    }

    // The head is `actual` cylinders from home. The track register only says what the program
    // believes; the move is by the difference.
    if(target < 0)
        target = 0;
    if(target > 255)
        target = 255;

    p->intended = (uint8_t)intended;
    p->status = F_INDEX | (intended == 0 ? F_TRACK0 : 0) | ((cmd & C_LOADHEAD) ? F_HEADLOAD : 0);
    p->irq_drq = WD_IRQ;
    p->ptr = p->read_len = p->write_len = 0;

    c->target = (uint8_t)target;
}

// The command is done. (cur_lock is 0 for the start-up load, which no program is waiting for.)
static void finish_command(struct fdc_cmd *c, uint8_t extra_status) {
#if FDC_SEEK_MIN_MS
    if(c->cur_lock && c->state != FDC_CMD_SETTLE && (uint32_t)(c->now_ms - c->cmd_start_ms) <= FDC_SEEK_MIN_MS) {
        c->settle_status = extra_status;
        c->state = FDC_CMD_SETTLE;  // fdc_cmd_poll finishes it once the time is up
        return;
    }
#endif
    if(c->cur_lock) {
        FDC_TRACE1('Q', c->proto.status | extra_status, (uint32_t)c->proto.irq_drq << 24 | (uint32_t)c->actual[c->cur_drive] << 16 | c->proto.ptr);
        c->proto.status |= extra_status;
        if(c->floppy->drive[c->cur_drive].read_only)
            c->proto.status |= F_READONLY;
        apply_result(c, &c->proto, c->cancelled);
    }
    c->state = FDC_IDLE;
    c->cur_lock = 0;
}

// Advance a seek: write back the old cylinder if it has changed, then load the new one, then answer
static void continue_seek(struct fdc_cmd *c) {
    const unsigned drive = c->cur_drive;
    const struct floppy_drive *d = &c->floppy->drive[drive];

    if(c->valid[drive] && c->actual[drive] == c->target) {
        finish_command(c, 0);  // already there
        return;
    }

    // The old cylinder's changes go back to the card before it is replaced
    for(int tries = 0; tries < 2 * FDC_SECTORS_PER_CYL; tries++) {
        const int dirty = find_dirty(c, drive);
        if(dirty < 0)
            break;
        if(start_flush(c, drive, dirty)) {
            c->state = FDC_CMD_FLUSH;
            return;
        }
        // could not even start: give the sector up rather than never seeking again
        c->stats.flush_errors++;
        c->flushed[drive][dirty] = c->link->written[drive][dirty];
    }

    if(c->target >= d->cylinders) {  // off the end of the disk
        c->valid[drive] = false;
        c->actual[drive] = c->target;
        finish_command(c, F_SEEKERR);
        return;
    }

    if(floppy_start_load(c->floppy, drive, c->target, c->cyl[drive])) {
        FDC_TRACE1('L', c->target, drive);
        c->stats.loads++;
        c->state = FDC_CMD_LOAD;
        return;
    }

    c->valid[drive] = false;
    c->stats.load_errors++;
    finish_command(c, F_SEEKERR);
}


// ---- type II: read sector, write sector. All the arithmetic, no card access.

static void plan_type2(struct fdc_cmd *c, const struct snapshot *s) {
    const uint8_t cmd = s->command;
    const bool write = (cmd & 0x20) != 0;
    const unsigned drive = s->system & S_DRIVE0;
    const struct floppy_drive *d = &c->floppy->drive[drive];
    struct fdc_result *p = &c->proto;

    p->flags = 0;
    p->laststep = 0;
    p->track = 0;
    p->intended = 0;  // (not applied: only type I commands set it)
    p->irq_drq = WD_IRQ;
    p->ptr = p->read_len = p->write_len = 0;
    p->status = 0;

    if(!d->present) {
        p->status = F_NOTREADY;
        return;
    }
    if(write && d->read_only) {
        p->status = F_READONLY;
        return;
    }

    const unsigned side = (cmd & C_SIDECOMP) ? ((cmd & C_SIDE) ? 1 : 0) : ((s->system & S_SIDE) ? 1 : 0);
    const unsigned sec = s->sector;
    if(!c->valid[drive] || side >= d->sides || sec < 1 || sec > FDC_SECTORS_PER_SIDE) {
        p->status = F_NOTFOUND;  // record not found
        return;
    }

    // Multiple sectors run to the end of the side
    const unsigned count = (cmd & C_MULTIREC) ? FDC_SECTORS_PER_SIDE - sec + 1 : 1;
    p->ptr = side * FDC_SIDE_BYTES + (sec - 1) * FDC_SECTOR_BYTES;
    if(write)
        p->write_len = count * FDC_SECTOR_BYTES;
    else
        p->read_len = count * FDC_SECTOR_BYTES;
    p->status = F_BUSY | F_DRQ;
    p->irq_drq = WD_DRQ;
}


static void start_command(struct fdc_cmd *c) {
    // the registers as the program had them when it wrote the command
    const struct snapshot snap = { .lock = 0, .command = c->m.command, .data = c->m.data, .track = c->m.track,
                                   .sector = c->m.sector, .system = c->msys };
    const struct snapshot *s = &snap;
    FDC_TRACE1('K', s->command, (uint32_t)s->track << 24 | (uint32_t)s->sector << 16 | (uint32_t)s->data << 8 | (s->system & 0xff));
    c->stats.commands++;
    c->started = true;
    c->cancelled = false;
    c->cur_lock = ++c->serial;
    c->cmd_start_ms = c->now_ms;
    c->cur_drive = s->system & S_DRIVE0;
    c->proto.laststep = 0;

    if(s->command >= 0x80) {
        plan_type2(c, s);
        c->state = FDC_IDLE;
        apply_result(c, &c->proto, false);
        c->cur_lock = 0;
        return;
    }

    c->stats.seeks++;
    plan_type1(c, s);
    if(!c->floppy->drive[c->cur_drive].present) {
        finish_command(c, F_NOTREADY);
        return;
    }
    continue_seek(c);
}


// ---- the register writes core 0 has pushed

// Is this event a command that starts something? (Seen from the event alone: it must wait for the command in
// flight to finish before it is taken.)
static bool is_start(uint32_t ev) {
    const uint8_t v = ev >> 8;
    return (ev & 0xff) == 0 && !((ev >> 16) & EVF_BUSY) && (v & 0xf0) != 0xd0 && v < 0xc0;
}

static void replay_event(struct fdc_cmd *c, uint32_t ev) {
    const unsigned reg = ev & 0xff;
    const uint8_t v = (uint8_t)(ev >> 8);
    const bool busy = ((ev >> 16) & EVF_BUSY) != 0;

    switch(reg) {
        case 0: {
            const bool model_busy = (c->m.status & F_BUSY) != 0;
            const enum fdc_action act = fdc_regs_command(&c->m, v, busy);
            if(act == FDC_ACT_FORCE) {
                // Core 0 thought a command was still going. If this model has already finished it (the result
                // was not in yet), the status core 0 had was BUSY alone, which the Force Interrupt cleared.
                if(busy && !model_busy)
                    c->m.status = 0;
                if(c->state != FDC_IDLE)
                    c->cancelled = true;
            } else if(act == FDC_ACT_START) {
                start_command(c);
            }
            break;
        }
        case 1: c->m.track = v; break;
        case 2: c->m.sector = v; break;
        case 3: c->m.data = v; break;
        case 4: c->msys = (uint16_t)((c->msys & ~S_SIDE) | ((v & 1) ? S_SIDE : 0)); break;
        case 5: c->msys = (uint16_t)((c->msys & ~(S_MOTOR | 0x03)) | (v & (S_MOTOR | 0x03))); break;
        case EV_END:
            // a data transfer finished: core 0 has cleared DRQ and BUSY and raised IRQ
            c->m.status &= ~(F_DRQ | F_BUSY);
            c->m.irq_drq = WD_IRQ;
            c->m.read_len = c->m.write_len = 0;
            c->m.data = v;
            break;
        default:
            break;
    }
    c->img_dirty = true;
}

// Replay everything pushed so far, in order. A command is only started when nothing else is being carried out;
// it and what follows it stay in the ring until then.
static void consume_events(struct fdc_cmd *c) {
    const uint32_t head = __atomic_load_n(&c->link->ev_head, __ATOMIC_ACQUIRE);

    while(c->tail != head) {
        const uint32_t ev = c->link->ev[c->tail & (FDC_EVENTS - 1)];
        if(is_start(ev) && c->state != FDC_IDLE)
            break;
        c->tail++;
        __atomic_store_n(&c->link->ev_tail, c->tail, __ATOMIC_RELEASE);
        replay_event(c, ev);
    }
}


// ---- an operation on the card has finished (or failed)

static void finish_op(struct fdc_cmd *c, bool ok, uint32_t now_ms) {
    const unsigned drive = c->cur_drive;
    const enum fdc_cmd_state state = c->state;

    if(state == FDC_CMD_FLUSH) {
        if(c->op_sector >= 0) {
            if(ok) {
                c->flushed[drive][c->op_sector] = c->op_count;
                c->op_fails = 0;
                c->stats.flushes++;
            } else if(++c->op_fails >= 3) {
                // Give this sector up: a seek that can never finish would hang the machine
                c->flushed[drive][c->op_sector] = c->op_count;
                c->op_fails = 0;
                c->stats.flush_errors++;
            }
        }
        c->op_sector = -1;
        continue_seek(c);
    } else if(state == FDC_CMD_LOAD) {
        FDC_TRACE1('l', c->target, ok);
        if(ok) {
            c->actual[drive] = c->target;
            c->valid[drive] = true;
            for(int i = 0; i < FDC_SECTORS_PER_CYL; i++)
                c->flushed[drive][i] = c->link->written[drive][i];
            finish_command(c, 0);
        } else {
            c->valid[drive] = false;
            c->actual[drive] = c->target;
            c->stats.load_errors++;
            finish_command(c, F_SEEKERR);
        }
    } else if(state == FDC_BG_FLUSH) {
        if(ok) {
            c->flushed[drive][c->op_sector] = c->op_count;
            c->stats.flushes++;
        } else {
            c->stats.flush_errors++;
            c->retry_after_ms = now_ms + FDC_FLUSH_RETRY_MS;
        }
        c->op_sector = -1;
        c->state = FDC_IDLE;
    }
}


// ---- nothing to do for the program: keep the buffers loaded and the card up to date

static void background(struct fdc_cmd *c, uint32_t now_ms) {
    // The start-up load of cylinder 0 of each drive
    for(unsigned d = 0; d < FDC_MAX_DRIVES; d++) {
        if(c->floppy->drive[d].present && !c->valid[d] && !c->load_tried[d]) {
            c->load_tried[d] = true;
            c->cur_drive = d;
            c->target = 0;
            c->cur_lock = 0;
            if(floppy_start_load(c->floppy, d, 0, c->cyl[d])) {
                c->state = FDC_CMD_LOAD;
                c->stats.loads++;
            }
            return;
        }
    }

    if(now_ms < c->retry_after_ms)
        return;

    const bool motor_off = !(c->link->system & S_MOTOR);
    const bool quiet = (uint32_t)(now_ms - c->last_write_ms) >= FDC_FLUSH_IDLE_MS;
    if(!motor_off && !quiet)
        return;

    for(unsigned d = 0; d < FDC_MAX_DRIVES; d++) {
        const int dirty = find_dirty(c, d);
        if(dirty < 0)
            continue;
        c->cur_drive = d;
        if(start_flush(c, d, dirty)) {
            c->state = FDC_BG_FLUSH;
        } else {
            c->stats.flush_errors++;
            c->retry_after_ms = now_ms + FDC_FLUSH_RETRY_MS;
        }
        return;
    }
}


void fdc_cmd_poll(struct fdc_cmd *c, uint32_t now_ms) {
    const uint32_t total = c->link->write_total;
    if(total != c->last_write_total) {
        c->last_write_total = total;
        c->last_write_ms = now_ms;
    }

    c->now_ms = now_ms;
    c->started = false;
    consume_events(c);
    if(c->started) {
        post_image(c);
        return;
    }

#if FDC_SEEK_MIN_MS
    if(c->state == FDC_CMD_SETTLE) {
        if((uint32_t)(now_ms - c->cmd_start_ms) > FDC_SEEK_MIN_MS)
            finish_command(c, c->settle_status);
        post_image(c);
        return;
    }
#endif
    if(c->state != FDC_IDLE) {
        const enum floppy_status st = floppy_poll(c->floppy);
        if(st != FLOPPY_PENDING)
            finish_op(c, st == FLOPPY_OK, now_ms);
    } else {
        background(c, now_ms);
    }
    post_image(c);
}
