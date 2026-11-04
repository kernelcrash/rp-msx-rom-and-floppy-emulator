#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "fdc_trace.h"

// WD2793 floppy controller emulation, memory mapped at $7FF8-$7FFF (mirrored in each 16 KB page
// of slot 1), split between the two cores. The behaviour is that of the STM32 project's
// interrupt.S (which follows fMSX's WD1793.c); the differences are listed in fdc_fast.c.
//
//   fdc_fast.c (core 0, the bus core): the register file, kept as bytes staged in rom_image so
//     that a register READ is served by the normal ROM path. It does only what the Z80 can notice
//     within a few microseconds: the immediate effect of a register write (BUSY, the register
//     itself), the data path byte by byte, and the copying in of register images from core 1.
//     Every register write is also pushed into an event ring for core 1. Constant time work only.
//   fdc_cmd.c (core 1, polled, no interrupt): keeps a complete model of the registers by replaying
//     the event ring, executes commands (seek, sector setup, loading a cylinder from the disk image,
//     writing changed sectors back), and posts the whole register state as an image.
//
// The two cores share only the fdc_link below. Every field has ONE writer; nothing is a shared
// read-modify-write. The event ring is a single producer, single consumer queue. The image uses a
// sequence lock (the counter is odd while the writer is in the middle of an update), so a reader
// that overlaps a writer notices and retries.
//
// An image says how many events core 1 had consumed when it was made. Core 0 applies it only if that
// is every event it has pushed: if the Z80 has written since, the image is out of date, and core 1
// will post a newer one after it has consumed the write.

#define FDC_MAX_DRIVES 2
#define FDC_WQ 16              // writes that can wait for core 0 to apply them (a power of two)
#define FDC_SECTOR_BYTES 512
#define FDC_SECTORS_PER_SIDE 9
#define FDC_SIDE_BYTES (FDC_SECTORS_PER_SIDE * FDC_SECTOR_BYTES)   // 4608: one side of one cylinder
#define FDC_CYL_BYTES (2 * FDC_SIDE_BYTES)                         // 9216: a cylinder buffer, both sides
#define FDC_SECTORS_PER_CYL (2 * FDC_SECTORS_PER_SIDE)             // 18

// Where the registers are staged: the last 8 bytes of each 16 KB page of the slot ($3FF8, $7FF8, $BFF8,
// $FFF8), in an image laid out like the slot (offset = Z80 address)
#define FDC_STAGE_BASE 0x3ff8u
#define FDC_STAGE_MIRROR 0x4000u
// An address (A15..A0) of a read of this slot is a controller access when A3..A13 are all ones. A14 and A15
// are free, which gives the four mirrors.
#define FDC_DECODE_MASK 0x3ff8u

// ---- WD2793 status register
#define F_BUSY     0x01
#define F_READONLY 0x40   // write protected
#define F_NOTREADY 0x80
#define F_INDEX    0x02   // type I
#define F_TRACK0   0x04
#define F_SEEKERR  0x10
#define F_HEADLOAD 0x20
#define F_DRQ      0x02   // type II and III
#define F_NOTFOUND 0x10   // record not found

// The STM32 project masks NOT READY out of every status read ("all drives ready"). Built with FDC_ALWAYS_READY the
// controller does the same: an absent drive still fails the way it would, but the status byte the Z80 sees never has
// bit 7.
#ifdef FDC_ALWAYS_READY
#define FDC_STATUS_BYTE(s) ((uint8_t)((s) & ~F_NOTREADY))
#else
#define FDC_STATUS_BYTE(s) ((uint8_t)(s))
#endif

// ---- $7FFF
#define WD_IRQ 0x80
#define WD_DRQ 0x40

// What a read of $7FFF returns for a given irq_drq (the model's own byte: WD_IRQ 0x80, WD_DRQ 0x40, a
// flag each). The real register is ACTIVE LOW with the low six bits high: $FF nothing pending, $7F data
// request, $BF interrupt request, $3F both. (The STM32 project returned irq_drq itself, whose idle value
// $00 the disk ROM reads as "data request"; traced in openMSX with the real NMS-8250 ROM.)
static inline uint8_t fdc_reg7(uint8_t irq_drq) {
    uint8_t v = 0xff;
    if(irq_drq & WD_DRQ)
        v &= (uint8_t)~0x80;
    if(irq_drq & WD_IRQ)
        v &= (uint8_t)~0x40;
    return v;
}

// ---- command bits
#define C_SIDECOMP 0x02
#define C_LOADHEAD 0x08
#define C_SIDE     0x08
#define C_IRQ      0x08
#define C_SETTRACK 0x10
#define C_MULTIREC 0x10

// ---- system register (drive select, side and motor, as written to $7FFC and $7FFD)
#define S_DRIVE0   0x01   // bit 0 of the drive select: drive A (0) or B (1)
#define S_SIDE     0x10
#define S_MOTOR    0x80
#define S_LASTSTEP 0x100  // direction of the last step (internal)

// ---- the registers, as core 0 and core 1's model both hold them (16 bytes: it is copied as four words)
struct fdc_regs {
    uint8_t status, irq_drq, track, sector;
    uint8_t data, command, intended, reserved;
    uint16_t ptr, read_len, write_len, reserved2;
};
_Static_assert(sizeof(struct fdc_regs) == 16, "fdc_regs is copied as four words");

// What a write to the command register does to the registers, shared by core 0 (which does it at once, so that
// BUSY is visible straight away) and core 1's model (which repeats it when it replays the event). `busy` is
// whether the controller was busy when the Z80 wrote, as seen by core 0: core 1 is told, it does not decide.
enum fdc_action { FDC_ACT_NONE, FDC_ACT_START, FDC_ACT_FORCE };
static inline enum fdc_action fdc_regs_command(struct fdc_regs *r, uint8_t v, bool busy) {
    r->irq_drq = 0;  // every command write resets the IRQ request, even one that is then ignored

    if((v & 0xf0) == 0xd0) {
        // Force Interrupt: stop whatever is going on
        r->read_len = 0;
        r->write_len = 0;
        if(busy) {
            r->status &= ~F_BUSY;
            if(r->command >= 0x80)  // a type II command's DRQ goes too (bit 1 is INDEX for type I)
                r->status &= ~F_DRQ;
        } else {
            r->status = r->intended == 0 ? F_TRACK0 : 0;
        }
        r->command = 0xd0;
        if(v & C_IRQ)
            r->irq_drq = WD_IRQ;
        return FDC_ACT_FORCE;
    }
    if(busy)
        return FDC_ACT_NONE;  // ignored while busy
    if(v >= 0xc0) {
        // read address, read track, write track: not implemented. Fail in a defined way.
        r->command = v;
        r->status = F_NOTFOUND;
        r->irq_drq = WD_IRQ;
        return FDC_ACT_NONE;
    }
    r->command = v;
    r->status = F_BUSY;
    return FDC_ACT_START;
}

// ---- messages

// Register writes, core 0 to core 1, in order. An event is reg | value << 8 | flags << 16.
#define FDC_EVENTS 1024      // a power of two: core 1 polls at least every 32 us or so, the Z80 writes far less often
#define EV_END 7             // not a register: a data transfer has finished (value: the last data byte)
#define EVF_BUSY 0x01        // the controller was busy when the Z80 wrote

struct fdc_img {               // core 1 writes, core 0 reads: the whole register state
    volatile uint32_t lock;    // odd while being written
    volatile uint32_t consumed; // events core 1 had consumed when it made this
    volatile uint32_t w[6];    // w[0..3]: struct fdc_regs; w[4], w[5]: registers 0-3 and 4-7 as staged bytes
};

#define RES_SET_TRACK 0x01     // load the track register from `track`
#define RES_SET_LASTSTEP 0x02  // set the last step direction from `laststep`
#define RES_SET_INTENDED 0x04  // load `intended` (type I commands)

// The outcome of one command, as core 1 works it out (never shared)
struct fdc_result {
    uint8_t status;
    uint8_t irq_drq;
    uint8_t flags;
    uint8_t track;
    uint8_t intended; // the track the command aimed for (the TRACK0 test in Force Interrupt)
    uint8_t laststep;
    uint16_t ptr;     // a data transfer: where in the cylinder buffer, and how many bytes
    uint16_t read_len;
    uint16_t write_len;
};

struct fdc_link {
    volatile uint32_t ev_head;      // core 0: events pushed
    volatile uint32_t ev_tail;      // core 1: events consumed
    volatile uint32_t ev_lost;      // core 0: events dropped because the ring was full (never, if core 1 polls)
    volatile uint32_t ev[FDC_EVENTS];
    struct fdc_img img;
    // Written by core 0 only. A sector counts as written when its last byte has been stored.
    // Core 1 compares each with what it has flushed, so there is no shared bit to clear.
    volatile uint16_t written[FDC_MAX_DRIVES][FDC_SECTORS_PER_CYL];
    volatile uint32_t write_total;  // sum of the above (wraps), for "has anything been written lately"
    volatile uint16_t system;       // copy of the system register, for the motor bit
};

// ---- test hooks: the host tests run the other core at chosen points inside a message update
#ifdef FDC_TEST_HOOKS
void fdc_test_hook(int point);
#define FDC_HOOK(p) fdc_test_hook(p)
#else
#define FDC_HOOK(p) ((void)0)
#endif
#define FDC_BARRIER() __atomic_thread_fence(__ATOMIC_SEQ_CST)

// ---- core 0 ---------------------------------------------------------------------------------
struct fdc_fast {
    struct fdc_link *link;
    uint8_t *rom;                 // the slot's 64K as the Z80 sees it; the registers are staged into it
    uint8_t *cyl[FDC_MAX_DRIVES]; // the cylinder buffers, FDC_CYL_BYTES each

    union {
        struct fdc_regs r;        // what the next reads return, and where the transfer is
        struct {
            uint8_t status;
            uint8_t irq_drq;
            uint8_t track, sector, data, command, intended, reserved;
            uint16_t ptr, read_len, write_len, reserved2;
        };
    };
    uint16_t system;              // drive select, side and motor bits (S_LASTSTEP is core 1's)
    uint32_t head;                // events pushed (this core's copy of link->ev_head)
    // Work in progress, one small step per call of fdc_fast_idle (see fdc_fast.c): nothing on core 0 may
    // take long enough to hold up a ROM request
    uint8_t dirty;                // registers whose staged byte is out of date (bit n = register n)
    uint16_t wq[FDC_WQ];          // register writes waiting to be applied, oldest first: reg << 8 | value
    uint8_t wq_head, wq_tail;     // written / applied so far (free running, FDC_WQ is a power of two)
    bool img_pending;             // an image copied from core 1, waiting to be applied
    uint32_t img_consumed;
    uint32_t img_w[6];
    uint32_t applied;             // lock of the last image copied
};

void fdc_fast_init(struct fdc_fast *f, struct fdc_link *link, uint8_t *rom, uint8_t *cyl0, uint8_t *cyl1);
// A write to register `reg` (0-7) of the controller: it is only QUEUED here (a few instructions); the work is
// done a step at a time by fdc_fast_idle, oldest first. Writes are never dropped: if the queue is full (it holds
// FDC_WQ, so it never is in the bus loop) this runs steps until there is room.
void fdc_fast_write(struct fdc_fast *f, unsigned reg, uint8_t value);
// The same, but applied before it returns (the bus loop: nothing may run between the write and the Z80's next read)
void fdc_fast_write_now(struct fdc_fast *f, unsigned reg, uint8_t value);
static inline bool fdc_fast_can_queue(const struct fdc_fast *f) { return (uint8_t)(f->wq_head - f->wq_tail) < FDC_WQ; }
// Call after a read of register `reg` has been served: the side effects of a read (a few instructions)
void fdc_fast_after_read(struct fdc_fast *f, unsigned reg);
// Is there anything for fdc_fast_idle to do?
static inline bool fdc_fast_work(const struct fdc_fast *f) {
    return (f->wq_head != f->wq_tail) | f->img_pending | (f->dirty != 0) | (f->link->img.lock != f->applied);
}
// Call from the idle part of the loop while fdc_fast_work(): ONE small step of: applying a queued write (its
// immediate effect, and the event for core 1), copying an image from core 1, applying it, or staging one
// register into the ROM image. A ROM request never waits behind more than one step.
void fdc_fast_idle(struct fdc_fast *f);
// For tests: run steps until there is nothing left to do
void fdc_fast_settle(struct fdc_fast *f);

// ---- core 1 ---------------------------------------------------------------------------------
struct floppy;

enum fdc_cmd_state {
    FDC_IDLE,
    FDC_CMD_FLUSH,   // a seek is writing back the old cylinder's changed sectors
    FDC_CMD_LOAD,    // a seek is loading the new cylinder
    FDC_BG_FLUSH,    // writing back changed sectors while nothing else is going on
    FDC_CMD_SETTLE,  // a type I command is done but stays BUSY until FDC_SEEK_MIN_MS has passed
};

struct fdc_stats {
    uint32_t commands, seeks, loads, flushes, flush_errors, load_errors, stale_results;
};

struct fdc_cmd {
    struct fdc_link *link;
    struct floppy *floppy;
    uint8_t *cyl[FDC_MAX_DRIVES];

    // The model: the registers as the Z80 has them, rebuilt from the event ring
    struct fdc_regs m;
    uint16_t msys;                            // drive select, side, motor, and the direction of the last step
    uint32_t tail;                            // events consumed
    bool cancelled;                           // a Force Interrupt came while the command in flight was being carried out
    bool img_dirty;                           // the model has changed since the last image was posted
    bool started;                             // a command was started by this poll

    enum fdc_cmd_state state;
    uint32_t serial;                          // number of the last command started (never 0)
    uint32_t cur_lock;                        // serial of the command being carried out, 0 for the start-up load
    struct fdc_result proto;                  // the result being prepared for the current command
    uint8_t cur_drive;
    uint8_t target;                           // cylinder a seek is going to

    uint8_t actual[FDC_MAX_DRIVES];           // cylinder held in each buffer
    bool valid[FDC_MAX_DRIVES];               // the buffer holds `actual` intact
    uint16_t flushed[FDC_MAX_DRIVES][FDC_SECTORS_PER_CYL];  // the `written` count each sector was flushed at
    int op_sector;                            // sector being written back by the operation in flight
    uint16_t op_count;                        // ... and the `written` count sampled before it started

    bool load_tried[FDC_MAX_DRIVES];          // the start-up load of cylinder 0 has been attempted
    uint8_t op_fails;                         // consecutive failed write-backs of one sector during a seek
    uint32_t last_write_total, last_write_ms;
    uint32_t retry_after_ms;                  // after a failed write-back
    uint32_t images;                          // images posted
    uint32_t now_ms;                          // the time of the current poll
    uint32_t cmd_start_ms;                    // when the command being carried out was started
    uint8_t settle_status;                    // FDC_CMD_SETTLE: the extra status to finish with
    struct fdc_stats stats;
};

#define FDC_FLUSH_IDLE_MS 300     // write changed sectors back this long after the last write
#define FDC_FLUSH_RETRY_MS 1000   // after a write-back failed

void fdc_cmd_init(struct fdc_cmd *c, struct fdc_link *link, struct floppy *floppy, uint8_t *cyl0, uint8_t *cyl1);
// Call repeatedly from core 1 (from the render loop's idle points). Never blocks: it advances one
// step of whatever is in progress.
void fdc_cmd_poll(struct fdc_cmd *c, uint32_t now_ms);
