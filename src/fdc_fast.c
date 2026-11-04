// The core 0 half of the WD2793 emulation. See fdc.h for the split.
//
// Everything here is constant time and short: it runs in the gaps between ROM reads. "Short" means one
// step of a few tens of instructions: a ROM request that arrives while core 0 is busy waits for the step to
// finish, and has only a couple of hundred ns of slack before its byte is late and the Z80 executes junk.
// An earlier version did a whole command or a whole result in one go (200-400 instructions) and broke the
// running machine at random. So an image from core 1 is copied and then applied in separate steps, and
// staging into the ROM image is one register per step. A captured write is different: the bus loop calls
// fdc_fast_write_now(), which queues it and drains the queue immediately, because nothing may run between
// the write and the Z80's next read of the register (see the comment on it, and on the command-write case
// in apply_write() below). Tests also use the plain queue-only fdc_fast_write(), drained a step at a time by
// fdc_fast_idle(). The register file is kept as bytes staged into the cartridge image (four copies, one in
// each 16 KB page of the slot), so a read of any register is answered by the ordinary ROM path with no extra latency;
// what this file adds is the work AFTER a read (fdc_fast_after_read) and the handling of writes.
//
// What core 0 does NOT do any more: work out what a command means. A write takes its immediate effect here
// (the register changes, BUSY goes up: the Z80 may look at once) and is pushed into an event ring; core 1
// replays the ring into a full model of the registers, does the command, and sends back the registers as an
// image, which is copied in here if the Z80 has not written anything since core 1 looked.
//
// Behaviour follows the STM32 project's interrupt.S (itself after fMSX's WD1793.c). Where it
// deliberately differs:
//  - Results of a command appear when core 1 has finished it (tens of microseconds to some
//    milliseconds later); only BUSY is visible at once. In particular IRQ is raised when a seek
//    completes, not when it starts, and BUSY is cleared when core 1 says so, not at the next
//    status read.
//  - A seek does not set DRQ in $7FFF (interrupt.S did, by accident of its completion path).
//  - Force Interrupt also clears the write length. It abandons the result of a command core 1 is
//    still working on: core 1 finishes the move (a seek cannot be stopped half way) but only the
//    track register it leaves behind is applied, so that the register still says where the head
//    is, as interrupt.S arranged. A command written before that has happened waits for it.
//  - The status register is not masked after a read for a type II/III command (that masked away
//    error bits after the first read); only the INDEX toggle after a type I command remains.
//  - Read address, read track and write track answer "record not found" straight away instead of
//    leaving the status at 0.
//  - The data register write of a byte that completes a sector counts that sector as written, so
//    that only sectors written whole are ever copied back to the card.
#include "fdc.h"
#include <string.h>
#include "board_config.h"
#include "rom.h"

// The four mirrors of the register window: the last 8 bytes of every 16K page of the slot (f->rom is laid
// out like the slot, so every mirror is a separate byte)
static inline void stage_write(struct fdc_fast *f, unsigned offset, uint8_t v) {
    uint8_t *p = f->rom + FDC_STAGE_BASE + offset;
    p[0] = v;
    p[FDC_STAGE_MIRROR] = v;
    p[2 * FDC_STAGE_MIRROR] = v;
    p[3 * FDC_STAGE_MIRROR] = v;
}
static inline void stage(struct fdc_fast *f, unsigned reg, uint8_t v) {
    stage_write(f, reg, v);
}

static inline uint8_t *cylinder(const struct fdc_fast *f) {
    return f->cyl[f->system & S_DRIVE0];
}

#define DIRTY(reg) (1u << (reg))

// What a read of register `reg` returns now. The data register returns the next byte of a transfer in
// progress, or the last byte read or written.
static uint8_t reg_value(const struct fdc_fast *f, unsigned reg) {
    switch(reg) {
        case 0: return FDC_STATUS_BYTE(f->status);
        case 1: return f->track;
        case 2: return f->sector;
        case 3: return f->read_len ? cylinder(f)[f->ptr] : f->data;
        case 7: return fdc_reg7(f->irq_drq);
        default: return 0xff;
    }
}

void fdc_fast_init(struct fdc_fast *f, struct fdc_link *link, uint8_t *rom, uint8_t *cyl0, uint8_t *cyl1) {
    f->link = link;
    f->rom = rom;
    f->cyl[0] = cyl0;
    f->cyl[1] = cyl1;
    f->status = F_TRACK0;
    f->irq_drq = 0;
    f->track = f->sector = f->data = f->command = f->intended = 0;
    f->system = 0;
    f->ptr = f->read_len = f->write_len = 0;
    f->head = 0;
    f->dirty = 0;
    f->wq_head = f->wq_tail = 0;
    f->img_pending = false;
    f->applied = 0;
    // The whole link back to zero, not just what this side counts from: a disk attached after another has been
    // used (fdc_attach_prepared) reuses the link, and a stale ev_tail made every event look like a full ring
    // (dropped), a stale img.lock an image to apply. Both halves re-initialise from zero after this.
    memset((void *)link, 0, sizeof *link);
    for(unsigned r = 0; r < 8; r++)
        stage(f, r, reg_value(f, r));
}


// ---- telling core 1

// Push a register write for core 1 to replay. The event is stored first and the count published after it, so
// core 1 never sees a count that covers an event which is not there yet.
static void push_event(struct fdc_fast *f, unsigned reg, uint8_t v, unsigned flags) {
    struct fdc_link *l = f->link;
    const uint32_t head = f->head;

    if(head - l->ev_tail >= FDC_EVENTS) {
        l->ev_lost++;  // core 1 is not keeping up: the model is now wrong. Cannot happen if it polls.
        return;
    }
    l->ev[head & (FDC_EVENTS - 1)] = reg | (uint32_t)v << 8 | (uint32_t)flags << 16;
    FDC_HOOK(1);
    __atomic_store_n(&l->ev_head, head + 1, __ATOMIC_RELEASE);
    f->head = head + 1;
    FDC_HOOK(2);
}


// ---- register writes (applied one per step)

static void data_write(struct fdc_fast *f, uint8_t v) {
    f->data = v;

    if(f->write_len) {
        cylinder(f)[f->ptr++] = v;
        f->write_len--;

        if((f->ptr & (FDC_SECTOR_BYTES - 1)) == 0) {  // that completed a sector
            f->link->written[f->system & S_DRIVE0][(f->ptr - 1) / FDC_SECTOR_BYTES]++;
            f->link->write_total++;
        }

        if(f->write_len == 0) {
            f->status &= ~(F_DRQ | F_BUSY);
            f->irq_drq = WD_IRQ;
            f->dirty |= DIRTY(0) | DIRTY(7);
            push_event(f, EV_END, v, 0);
        }
        // (the bytes of a transfer are not events: core 1 only needs to hear that it has finished)
    } else {
        push_event(f, 3, v, f->status & F_BUSY ? EVF_BUSY : 0);
    }

    f->dirty |= DIRTY(3);
}

void fdc_fast_write(struct fdc_fast *f, unsigned reg, uint8_t v) {
    // Writes can come close together, and steps only run in the gaps between ROM requests: a single slot here
    // lost the first of two writes that arrived before a step had applied it.
    while(!fdc_fast_can_queue(f))  // not in practice: the queue is deeper than any burst of writes
        fdc_fast_idle(f);
    f->wq[f->wq_head & (FDC_WQ - 1)] = (uint16_t)((reg & 7) << 8 | v);
    f->wq_head++;
}

static void apply_write(struct fdc_fast *f);

// A captured write, applied at once (together with any that are still waiting). The bus loop uses this: the write
// must have taken effect before the Z80 reads anything again, and there is no second chance to run a step in between.
void fdc_fast_write_now(struct fdc_fast *f, unsigned reg, uint8_t v) {
    fdc_fast_write(f, reg, v);
    while(f->wq_head != f->wq_tail)
        apply_write(f);
}

static inline void __attribute__((always_inline)) apply_write(struct fdc_fast *f) {
    const uint16_t w = f->wq[f->wq_tail & (FDC_WQ - 1)];
    const unsigned reg = w >> 8;
    const uint8_t v = (uint8_t)w;
    const bool busy = (f->status & F_BUSY) != 0;
    f->wq_tail++;
    FDC_COUNT(applied_writes);  // was FDC_TRACE0('W', ...): a timer read on every write is too much in the else-branch design

    switch(reg) {
        case 0: {
            const enum fdc_action act = fdc_regs_command(&f->r, v, busy);
#ifdef FLOPPY_TRACE
            // the commands the NMS-8250 disk ROM issues (traced in openMSX): a different one is a corrupted write
            if(act == FDC_ACT_START && v != 0x02 && v != 0x10 && v != 0x50 && v != 0x82 && v != 0x8a && v != 0xa2 && v != 0xa8)
                FDC_COUNT(unknown_cmds), FDC_FREEZE();
            if(v >= 0xc0 && (v & 0xf0) != 0xd0)
                FDC_FREEZE();  // the disk ROM never issues these
#else
            (void)act;
#endif
            // The Z80's very next instruction may read the status (BUSY) or $7FFF (the IRQ and DRQ this write just cleared),
            // and a stale answer there is fatal: the disk ROM reads $7FFF right after writing a read command, sees the
            // old interrupt request, decides the command is over, and never reads the data. So both are staged now, not
            // as dirty registers a later step gets round to. (A Force Interrupt ends a transfer: register 3 shows the
            // data byte again; that one can wait.)
            stage(f, 0, FDC_STATUS_BYTE(f->status));
            stage(f, 7, fdc_reg7(f->irq_drq));
            f->dirty = (uint8_t)((f->dirty & ~(DIRTY(0) | DIRTY(7))) | DIRTY(3));
            push_event(f, 0, v, busy ? EVF_BUSY : 0);
            break;
        }
        case 1:
            if(!busy) {
                f->track = v;
                f->dirty |= DIRTY(1);
                push_event(f, 1, v, 0);
            }
            break;
        case 2:
            if(!busy) {
                f->sector = v;
                f->dirty |= DIRTY(2);
                push_event(f, 2, v, 0);
            }
            break;
        case 3:
            data_write(f, v);
            break;
        case 4:
            f->system = (f->system & ~S_SIDE) | ((v & 1) ? S_SIDE : 0);
            f->link->system = f->system;
            push_event(f, 4, v, 0);
            break;
        case 5:
            f->system = (f->system & ~(S_MOTOR | 0x03)) | (v & (S_MOTOR | 0x03));
            f->link->system = f->system;
            f->dirty |= DIRTY(3);  // the selected drive may have changed
            push_event(f, 5, v, 0);
            break;
        default:
            break;
    }
}


// ---- the work after a read (inline, tiny: the next request is at least 800 ns away, but keep it short)

void fdc_fast_after_read(struct fdc_fast *f, unsigned reg) {
    FDC_COUNT(reads[reg & 7]);
    if(reg == 3) {
        if(f->read_len) {
            f->data = cylinder(f)[f->ptr++];
            if(--f->read_len == 0) {
                FDC_COUNT(transfer_ends);  // was FDC_TRACE0('E', ...)
                f->status &= ~(F_DRQ | F_BUSY);
                f->irq_drq = WD_IRQ;
                // (register 3 is left alone: the staged byte is the one just read, which is what an extra
                // read after the end of the transfer returns)
                f->dirty |= DIRTY(0) | DIRTY(7);
                push_event(f, EV_END, f->data, 0);
            } else {
                f->dirty |= DIRTY(3);
            }
        }
    } else if(reg == 0) {
        // After a type I command the INDEX bit toggles on every read (with the motor on) so that a
        // program that waits for index pulses sees them
        if(f->command < 0x80 && (f->system & S_MOTOR)) {
            f->status ^= F_INDEX;
            f->dirty |= DIRTY(0);
        }
    }
}


// ---- images from core 1

// Copy an image out of the mailbox (one step)
static void fetch_image(struct fdc_fast *f) {
    const struct fdc_img *m = &f->link->img;
    const uint32_t lock = m->lock;

    if(lock == f->applied || (lock & 1))
        return;
    FDC_BARRIER();

    f->img_consumed = m->consumed;
    f->img_w[0] = m->w[0];
    f->img_w[1] = m->w[1];
    FDC_HOOK(7);
    f->img_w[2] = m->w[2];
    f->img_w[3] = m->w[3];
    f->img_w[4] = m->w[4];
    f->img_w[5] = m->w[5];

    FDC_BARRIER();
    if(m->lock != lock)
        return;  // core 1 was in the middle of writing: look again next time
    f->applied = lock;
    f->img_pending = true;
}

// Apply a copied image (one step): the registers, and their staged bytes in all four mirrors
static void apply_image(struct fdc_fast *f) {
    f->img_pending = false;

    // While this core is moving data it owns the registers: an image made before the transfer began says
    // nothing about it (the end of the transfer is an event, and core 1 sends a new image after it)
    if(f->read_len | f->write_len)
        return;
    // The Z80 has written since core 1 looked: this image is out of date, and core 1 will send another
    if(f->img_consumed != f->head)
        return;

    struct fdc_regs r;
    memcpy(&r, f->img_w, sizeof r);
    // Never let a transfer run outside the cylinder buffer, whatever core 1 says
    if((uint32_t)r.ptr + r.read_len + r.write_len > FDC_CYL_BYTES || (r.read_len && r.write_len)) {
        r.ptr = r.read_len = r.write_len = 0;
        r.status = F_NOTFOUND;
        r.irq_drq = WD_IRQ;
        f->r = r;
        f->dirty = DIRTY(0) | DIRTY(1) | DIRTY(2) | DIRTY(3) | DIRTY(7);
        return;
    }
    FDC_COUNT(images_applied);  // was FDC_TRACE0('R', ...)

    f->r = r;
    f->dirty = 0;
    // registers 0-3 and 4-7 are two words: the same 8 bytes go to every mirror -- see stage_write
    const uint8_t *src = (const uint8_t *)&f->img_w[4];
    for(unsigned i = 0; i < 8; i++)
        stage_write(f, i, src[i]);
}


// ---- staging: one register into the four mirrors (one step)

static void stage_one(struct fdc_fast *f) {
    const uint8_t d = f->dirty;
    const unsigned reg = (unsigned)__builtin_ctz(d);
    f->dirty = d & (uint8_t)(d - 1);  // clear the lowest set bit
    stage(f, reg, reg_value(f, reg));
}


void fdc_fast_idle(struct fdc_fast *f) {
    // (a write goes first: the Z80 is waiting for it. An image that is being applied when a write comes in
    // is simply found out of date, as the write is an event it does not cover)
    if(f->wq_head != f->wq_tail)
        apply_write(f);
    else if(f->img_pending)
        apply_image(f);
    else if(f->link->img.lock != f->applied)
        fetch_image(f);
    else if(f->dirty)
        stage_one(f);
}

void fdc_fast_settle(struct fdc_fast *f) {
    for(int i = 0; i < 1000 && fdc_fast_work(f); i++)
        fdc_fast_idle(f);
}
