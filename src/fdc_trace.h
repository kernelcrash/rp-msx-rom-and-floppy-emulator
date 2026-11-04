#pragma once
// A trace of what the controller did on the rig, kept in RAM and read out with picotool after a run
// (see tools/fdc_trace.py). Off unless built with -DFLOPPY_TRACE=ON: the macros compile to nothing.
//
// Two rings, so that each core has only one writer: TRACE0 (core 0: the bus side) and TRACE1 (core 1: the
// command engine). An entry is 12 bytes: time (us, low 32 bits), a kind letter, and two values.
#include <stdint.h>

#ifdef FLOPPY_TRACE
struct fdc_trace_entry {
    uint32_t t;
    uint8_t kind;
    uint8_t pad;
    uint16_t a;
    uint32_t b;
};

#define FDC_TRACE_ENTRIES 1024  // per ring, a power of two
#define FDC_TRACE_MAGIC 0x46444354u  // "FDCT"
#define RQ_RING 32  // ROM requests remembered, a power of two

struct fdc_trace_data {
    uint32_t magic;
    uint32_t head0, head1;                 // entries written to each ring so far
    uint32_t rom_backlog;                  // ROM requests found with another already waiting behind them
    uint32_t rom_requests;                 // ROM requests served (a sample: only counted while tracing is on)
    uint32_t max_step_us;                  // (unused)
    uint32_t reads[8];                     // reads of each controller register that core 0 has seen (answered ones)
    uint32_t unknown_cmds;                 // commands accepted that the disk ROM never issues (corrupted writes?)
    uint32_t rej_ce, rej_slot0, rej_other; // rejected captures: CE high (ordinary RAM write) / A16 low / anything else
    uint32_t data_disagree;                // captures whose three snapshots disagreed on a data bit
    uint32_t rq_head;                      // ROM requests recorded in rq_ring
    uint32_t rq_stop;                      // ~0: recording; 0: stopped; otherwise entries still to record before stopping
    uint32_t rq_late;                      // requests already over (_MREQ high) at core 0's first look
    uint32_t rq_unanswered;                // requests not answered: writes, refresh cycles, or a read whose _RD never showed
    uint32_t rq_ring[RQ_RING];             // address | flags << 17: see main.c
    // what the ROM request loop did (main.c, trace builds): how each popped request was dealt with
    uint32_t c_answered;                   // answered as a read
    uint32_t c_else;                       // not answered
    uint32_t c_else_wr;                    //   ... because _WR was low: a write
    uint32_t c_else_mismatch;              //   ... because the pins were another cycle's address (stale entry)
    uint32_t c_else_rdhi;                  //   ... because _RD and _WR were both high: a refresh, or _RD not yet low
    uint32_t c_pending;                    // after the delay a captured write was waiting and was taken
    uint32_t c_idle;                       // after the delay a controller step was run
    uint32_t c_cap, c_cap_wr_high, c_cap_oe_high, c_cap_rd_low, c_cap_ce_high;  // the state of _WR, _MREQ, _RD and CE in every write capture
    uint32_t c_bad;                        // accepted writes the real disk ROM never makes (a wrong register or value)
    uint32_t bad[8];                       // the raw pin words of the first few of them
    uint32_t cap_ring[16];                 // every write capture's raw pin word, classified later (off the hot path)
    uint32_t cap_head;
    uint32_t applied_writes, transfer_ends, images_applied;  // plain counts, replacing per-event timer traces
    uint32_t c_since;                      // answered reads since the last fetch of $0000
    uint32_t c_gap;                        // ... and how many there were between the last two: the run length of the last Z80 start
    uint32_t c_zero;                       // answered fetches of address $0000 in slot 0: one per (re)start of the Z80
    uint32_t c_spin;                       // passes through the idle branch (nothing waiting): where the VDP is serviced
    struct fdc_trace_entry ring0[FDC_TRACE_ENTRIES];
    struct fdc_trace_entry ring1[FDC_TRACE_ENTRIES];
};

extern struct fdc_trace_data fdc_trace;
uint32_t fdc_trace_now(void);

static inline void fdc_trace0(uint8_t kind, uint16_t a, uint32_t b) {
    struct fdc_trace_entry *e = &fdc_trace.ring0[fdc_trace.head0 & (FDC_TRACE_ENTRIES - 1)];
    e->t = fdc_trace_now();
    e->kind = kind;
    e->a = a;
    e->b = b;
    fdc_trace.head0++;
}
static inline void fdc_trace1(uint8_t kind, uint16_t a, uint32_t b) {
    struct fdc_trace_entry *e = &fdc_trace.ring1[fdc_trace.head1 & (FDC_TRACE_ENTRIES - 1)];
    e->t = fdc_trace_now();
    e->kind = kind;
    e->a = a;
    e->b = b;
    fdc_trace.head1++;
}
// Core 0 records nothing, not even counts: any extra work on its write or read path has changed what the
// MSX does (a counter and a ring store per memory write broke booting on the Omega). Only core 1 traces.
#define FDC_COUNT(field) ((void)0)
// Stop the request history a few requests from now: what the Z80 was fetching just before something odd happened
#define FDC_FREEZE() ((void)0)  // (the request history is now frozen by main.c, at a Z80 restart)
#define FDC_TRACE0(kind, a, b) fdc_trace0((kind), (uint16_t)(a), (uint32_t)(b))
#define FDC_TRACE1(kind, a, b) fdc_trace1((kind), (uint16_t)(a), (uint32_t)(b))
#else
#define FDC_COUNT(field) ((void)0)
#define FDC_FREEZE() ((void)0)
#define FDC_TRACE0(kind, a, b) ((void)0)
#define FDC_TRACE1(kind, a, b) ((void)0)
#endif
