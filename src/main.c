#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/clocks.h>
#include <hardware/flash.h>
#include <hardware/pio.h>
#include <hardware/sync.h>
#include <hardware/structs/sio.h>
#include <hardware/vreg.h>
#include "board_config.h"
#include "bus_write.h"
#include "render.h"
#include "rom.h"
#include "tms9918a.h"
#include "vdp_bus.h"
#include "vga.h"

// SCRATCH_HOT: put the bus loop's code in scratch X and its hot variables in scratch Y -- the two small
// SRAM banks nothing else (DMA, core 1's VGA work) uses, so the loop's instruction fetches and variable
// accesses never wait on another bus master and its timing no longer depends on where the linker put
// anything in main SRAM. The ROM image itself (64K/128K) cannot go there; it stays in main SRAM.
#ifndef SCRATCH_HOT
#define SCRATCH_HOT 0
#endif
#if SCRATCH_HOT
#define HOT_VAR __scratch_y("hot")
#else
#define HOT_VAR
#endif
#if FLOPPY
#include "fdc_bus.h"
#endif
#if MENU
#include "menu_bus.h"
#endif

// The cartridge in the slot (cart.h): rom_image is the cartridge buffer, bank_register says which 8K of
// it (or of cart_ff) each 8K region of the slot shows. rom.c sets them up (rom_cart_setup) at boot and
// when the menu loads a new cartridge (core 1); mapper writes change bank_register here (core 0).
cart_bank_t bank_register[CART_BANK_SLOTS] HOT_VAR;
volatile enum megarom_type megarom_type HOT_VAR;
volatile uint32_t cart_mask HOT_VAR;
volatile enum cart_mode cart_mode HOT_VAR;
uint8_t cart_ff[CART_REGION_BYTES];

#if MEGAROM
static uint32_t cart_staged HOT_VAR;  // cart_stage_write's result for the write whose LATE half is next
#endif

// A memory write to this slot: _MREQ, _SLTSL and _WR low. Checked on both halves of a write.
#define CART_WRITE_MASK ((1u << CONFIG_PIN_MREQ) | (1u << CONFIG_PIN_SLTSL) | (1u << CONFIG_PIN_WR))

static void core1_main() {
    vga_init();
    render_init();
    render_loop();
}

#if FLOPPY && MENU
// Exists only to keep fdc_fast_work()'s body OUT of core0_loop's own instruction stream.
// fdc_fast_work is tiny and `static inline` (fdc.h), and the compiler dutifully inlines it at its
// call site below even though fdc_pio_active() is false for the whole session whenever a mapper
// cartridge (not a disk) is in use -- the dead-at-runtime inlined body still has live ranges
// (fdc_fast_state's several fields), and the register allocator has to account for them, which
// measurably changes the codegen of the surrounding code that DOES run every pass (confirmed by
// diffing the disassembly with/without this check: a Konami4 cartridge that works with it removed
// stalls a few seconds into a game with it inlined in place, and the only difference in the
// always-executed address-match path is extra register shuffling, not extra work actually done --
// see RPROM/plan-floppy-megarom-reconcile.md). `noinline` turns it into one real, isolated call: a
// few cycles when it actually has something to do (acceptable -- the FLOPPY&&MENU-without-MEGAROM
// configuration that worked before this phase already paid a real call here, just via
// fdc_pio_active() being a cross-TU call instead), and zero influence on the hot loop's own
// register allocation either way. fdc_fast_after_read (the other FDC read-side hook, used just
// above) needs no such wrapper: it is already a real, out-of-line function, never `static inline`.
static void __attribute__((noinline)) fdc_hot_idle(void) {
    if(fdc_pio_active() && fdc_fast_work(&fdc_fast_state))
        fdc_fast_idle(&fdc_fast_state);
}
#endif

#if ROM_EMULATION
// generic_read (rom.pio) pushes an unconditional snapshot of every Z80 read cycle, memory or I/O;
// this classifies one (see refactor-pio-sm.md). As on the breadboard with _CE: another slot's read is
// dropped from the sample's _SLTSL, and the RK_ROM case rechecks the live pins.
enum read_kind {
    RK_IRRELEVANT = 0,
    RK_ROM,
#if VDP_READS
    RK_VDP_IO,
#endif
#if VDP_READ_TRACK
    RK_VDP_TRACK,  // an I/O read, seen but not answered (the real VDP answers it)
#endif
};

static inline enum read_kind read_classify(uint32_t sample) {
    if(!(sample & (1u << CONFIG_PIN_MREQ)))  // _MREQ low: a memory read
        return (sample & (1u << CONFIG_PIN_SLTSL)) ? RK_IRRELEVANT : RK_ROM;
#if VDP_READS
    if(!(sample & (1u << CONFIG_PIN_IORQ)))  // _IORQ low: an I/O read
        return RK_VDP_IO;
#elif VDP_READ_TRACK
    if(!(sample & (1u << CONFIG_PIN_IORQ)))
        return RK_VDP_TRACK;
#endif
    // Neither _MREQ nor _IORQ low at the instant _RD fell. A real read always ends up with one of
    // them low, but _MREQ can lag _RD: caught on the rig right after a (bouncing) reset, where the
    // BIOS's very first fetch at 0000 was sampled with _MREQ still high, left undriven, and the
    // Z80 derailed on a floating opcode. io4.bin showed why a single look at the live pins is not
    // enough: after the last reset bounce _RD is sampled low with _MREQ high hundreds of times at
    // A=0000, and the real first fetch follows with no new _RD falling edge for generic_read to
    // trigger on. So wait on the live pins, for as long as _RD stays low, until _MREQ or _IORQ goes
    // low -- only reset can hold _RD low for longer than one bus cycle.
    // (Refresh cycles never get here: they assert _MREQ without _RD, so generic_read never fires.)
    for(;;) {
        const uint32_t pins = sio_hw->gpio_in;
        if(!(pins & (1u << CONFIG_PIN_MREQ)))
            return (pins & (1u << CONFIG_PIN_SLTSL)) ? RK_IRRELEVANT : RK_ROM;
#if VDP_READS
        if(!(pins & (1u << CONFIG_PIN_IORQ)))
            return RK_VDP_IO;
#elif VDP_READ_TRACK
        if(!(pins & (1u << CONFIG_PIN_IORQ)))
            return RK_VDP_TRACK;
#endif
        if(pins & (1u << CONFIG_PIN_RD))
            return RK_IRRELEVANT;
    }
}
#endif

// generic_write (bus_write.pio) pushes an unconditional EARLY snapshot (address/control, data not
// yet valid) and then a LATE one (data valid) for every Z80 write cycle, memory or I/O -- see
// refactor-pio-sm.md. write_classify_early() decides what an EARLY snapshot is (an I/O write, or a
// memory write to this slot that the cartridge mode wants); write_dispatch_late() applies the LATE one
// once data is valid, using whatever write_classify_early() decided and cached (cart_staged, for a
// mapper write: the address-dependent half, done early). The toggle never needs resetting: generic_write is always resident
// now (no more swap to desync against), so every write produces exactly two pushes, always in order,
// for the life of the program.
enum write_kind { WK_IGNORE = 0, WK_VDP_IO, WK_FDC, WK_MENU, WK_MAPPER,
#if VDPW_EARLY_PORT
    WK_VDP_DATA, WK_VDP_CTRL,  // -DVDPW_EARLY_PORT: the VDP port, decided from the EARLY sample
#endif
};
#if VDPW_EARLY_PORT && !(VDPW_INLINE_DATA && VDPW_NO_COUNTERS && !V9938)
#error "VDPW_EARLY_PORT needs VDPW_INLINE_DATA and VDPW_NO_COUNTERS (TMS9918A)"
#endif

static bool write_expect_late HOT_VAR;
static enum write_kind write_pending_kind HOT_VAR;

// TEMPORARY DIAGNOSTIC (refactor-pio-sm bring-up): build with -DWTRACE=1 (add_compile_definitions)
// to get the reset-armed bus trace described below. Off by default: it costs bus-loop time.
#ifndef WTRACE
#define WTRACE 0
#endif
enum { RT_IGNORED = 0, RT_DRIVEN = 1, RT_LATE = 2, RT_IO = 3 };
enum { IO_W_EARLY = 0, IO_W_LATE = 1, IO_R_ANSWERED = 2, IO_R_LATE = 3, IO_R_OTHER = 4 };
#if WTRACE

// TEMPORARY DIAGNOSTIC, armed by the MSX reset button. Recording restarts whenever the Z80 fetches
// 0000 after running somewhere else (a reset), so each press gives a fresh capture of:
//  * every I/O cycle since the reset (writes: early and late halves; reads: answered value, or
//    dropped as too late, or not a VDP port), up to IOT_N of them;
//  * a ring of the last MRT_N memory reads, to show where the Z80 is when the trace is taken.
// Written to flash WTRACE_DUMP_US after the most recent reset, then the border goes white.
// Read out with: picotool save -r 0x103F0000 0x103F4900 io.bin
#define IOT_N 2048
#define MRT_N 512
#define WTRACE_DUMP_US 4000000u
struct wtrace {
    uint32_t magic;
    uint32_t resets;                 // resets seen since power-on
    uint32_t io_total;               // I/O events since the last reset
    uint32_t r_total;                // reads since the last reset
    uint32_t w_total;                // write FIFO entries since the last reset
    uint32_t vdp_captured, vdp_writes, vdp_reads, vdp_late;  // since the last reset
    uint32_t hdr_spare[7];           // header is 16 words
    uint32_t io_sample[IOT_N];
    uint8_t io_kind[IOT_N];          // IO_*
    uint8_t io_value[IOT_N];         // the answer, for IO_R_ANSWERED
    uint32_t r_sample[MRT_N];        // ring
    uint8_t r_flag[MRT_N];           // bits 1:0: RT_*; bits 7:4: read RX level after the pop
    uint8_t r_value[MRT_N];          // the byte driven (RT_DRIVEN), or the late pins (RT_LATE)
    uint32_t f_sample[MRT_N];        // the FIRST MRT_N reads after the last reset, same layout
    uint8_t f_flag[MRT_N];
    uint8_t f_value[MRT_N];
    uint8_t pad[192];                // to a whole number of 256 byte flash pages
};
_Static_assert(sizeof(struct wtrace) % FLASH_PAGE_SIZE == 0, "wtrace must be whole flash pages");
struct wtrace wtrace = { .magic = 0x5754524a };  // "WTRJ": reset-armed I/O + first reads
static uint32_t wt_prev_addr = 1, wt_reset_us, wt_loop_us;
static uint32_t wt_base_cap, wt_base_wr, wt_base_rd, wt_base_late;

static inline void io_rec(uint32_t sample, uint32_t kind, uint32_t value) {
    const uint32_t i = wtrace.io_total++;
    if(i < IOT_N) {
        wtrace.io_sample[i] = sample;
        wtrace.io_kind[i] = (uint8_t)kind;
        wtrace.io_value[i] = (uint8_t)value;
    }
}

static void wtrace_reset(void) {
    wtrace.resets++;
    wtrace.io_total = wtrace.r_total = wtrace.w_total = 0;
    wt_base_cap = vdp_bus_captured;
    wt_base_wr = vdp_bus_vdp_writes;
    wt_base_rd = vdp_bus_vdp_reads;
    wt_base_late = vdp_bus_late_reads;
    // Only arm the dump timer for a reset at least 3 s after serving started: the Z80 runs garbage
    // from power-on until the reset button is pressed, and that garbage can pass through 0000 too.
    const uint32_t now = time_us_32();
    wt_reset_us = (now - wt_loop_us >= 3000000u) ? (now | 1) : 0;
}

static inline void rtrace_rec(uint32_t sample, uint32_t kind, uint32_t value) {
    if(kind == RT_DRIVEN) {
        const uint32_t a = BUS_ADDRESS(sample);
        if(a == 0 && wt_prev_addr != 0)
            wtrace_reset();
        wt_prev_addr = a;
    }
    const uint32_t n = wtrace.r_total++;
    const uint8_t flag = (uint8_t)(kind | (pio_sm_get_rx_fifo_level(CONFIG_ROM_PIO, GENERIC_READ_SM) << 4));
    const uint32_t i = n % MRT_N;
    wtrace.r_sample[i] = sample;
    wtrace.r_flag[i] = flag;
    wtrace.r_value[i] = (uint8_t)value;
    if(n < MRT_N) {
        wtrace.f_sample[n] = sample;
        wtrace.f_flag[n] = flag;
        wtrace.f_value[n] = (uint8_t)value;
    }
}

#define WTRACE_FLASH_OFFSET 0x3F0000u
static void wtrace_to_flash(void) {
    const uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(WTRACE_FLASH_OFFSET, (sizeof wtrace + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1));
    flash_range_program(WTRACE_FLASH_OFFSET, (const uint8_t *)&wtrace, sizeof wtrace);
    restore_interrupts(ints);
}
#else
static inline void io_rec(uint32_t sample, uint32_t kind, uint32_t value) { (void)sample; (void)kind; (void)value; }
static inline void rtrace_rec(uint32_t sample, uint32_t kind, uint32_t value) { (void)sample; (void)kind; (void)value; }
#endif

// TEMPORARY DIAGNOSTIC: build with -DSTATS=ON. Records, at no cost to the per-cycle path, whether the PIO
// state machines ever stalled on a full FIFO (missed bus cycles) and the context of the first few ROM reads
// that were answered too late. Armed by the first mapper bank write; dumped to flash at 0x3F0000 one second
// after the first late read, or STATS_DUMP_US after the arming write (read with: picotool save -r 0x103F0000
// 0x103F0100 stats.bin), and the border goes white. Decode with tools/stats.py.
#ifndef STATS
#define STATS 0
#endif
#if STATS
#define STATS_DUMP_US 20000000u
#define STATS_FLASH_OFFSET 0x3F0000u
union stats_u {
    struct {
        uint32_t magic;           // "STAT"
        uint32_t rom_late;        // ROM reads that were already over when core 0 got to them
        uint32_t vdp_late;        // VDP reads answered too late (vdp_bus_late_reads), since power-on
        uint32_t mapper_writes;   // memory writes seen in mapper mode since the game started
        uint32_t fdebug_rd;       // PIO0 FDEBUG at the dump: RXSTALL = bits 3:0, one per state machine
        uint32_t fdebug_wr;       // PIO1 FDEBUG at the dump
        uint32_t dump_us;         // time from the first mapper write to the dump
        uint32_t spare;
        struct {                  // the first 8 late ROM reads, captured on the (cold) late path
            uint32_t sample;      // generic_read's snapshot when _RD fell
            uint32_t pins;        // live pins when core 0 looked
            uint32_t rd_level;    // read FIFO level at that moment (entries queued behind this one)
            uint32_t wr_level;    // write FIFO level
            uint32_t wr_state;    // bit 0: mid write pair (waiting for the LATE half); bits 3:1: write kind
            uint32_t since_arm_us;
        } late[8];
    };
    uint32_t raw[64];             // one 256 byte flash page
};
static union stats_u stats = { .magic = 0x54415453 };
static uint32_t stats_arm_us, stats_late_us;
// Nothing in the per-cycle path may cost anything: an earlier version timed each piece with SysTick and
// read FIFO levels, and that alone stopped the menu from rendering. What is left is free: PIO FDEBUG's
// sticky RXSTALL bits (read once, at the dump), the already-rare "answered too late" path, and the VDP's
// own late-read counter.
static void stats_to_flash(void);
static void stats_check(void) {
    static bool dumped;
    const uint32_t now = time_us_32();
    // 1 s after the first late ROM read (the white border then marks the moment it happened), or
    // failing that STATS_DUMP_US after the game's first bank write
    if(!dumped && ((stats_late_us && now - stats_late_us >= 1000000u) ||
                   (stats_arm_us && now - stats_arm_us >= STATS_DUMP_US))) {
        dumped = true;
        stats_to_flash();
        tms9918a_set_backdrop(15);
    }
}
static void stats_to_flash(void) {
    stats.fdebug_rd = CONFIG_ROM_PIO->fdebug;
    stats.fdebug_wr = CONFIG_VDP_PIO->fdebug;
    stats.dump_us = time_us_32() - stats_arm_us;
    stats.vdp_late = vdp_bus_late_reads;
    const uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(STATS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(STATS_FLASH_OFFSET, (const uint8_t *)&stats, sizeof stats);
    restore_interrupts(ints);
}
#endif

// fdc_on is a compile-time constant in each copy of the bus loop (see core0_run): false means no
// disk is attached, and the controller's window is not even looked at.
static inline enum write_kind write_classify_early(uint32_t s, const bool fdc_on) {
#if !FLOPPY
    (void)fdc_on;
#endif
    // _IORQ low: an I/O write. Deliberately not bit 8 (_MREQ, as read_classify() uses for reads):
    // nothing ever proved bit 8 goes high during an I/O *write* on this rig -- every old write
    // program (vdp_bus.pio, and generic_write's own settle branch) told I/O from memory by _IORQ.
    if(!(s & (1u << CONFIG_PIN_IORQ))) {
        io_rec(s, IO_W_EARLY, 0);  // TEMPORARY DIAGNOSTIC
#if VDP_WRITES && VDPW_EARLY_PORT
        // The port is on A0-A7, valid well before _WR falls, so decide here, while _WR is still low,
        // and leave only the store for the LATE half. Writes to any other port are not ours.
        {
            const uint32_t port = (s >> CONFIG_PIN_ADDR_BASE) & 0xff;
            if((port & 0xfe) != CONFIG_VDP_PORT)
                return WK_IGNORE;
            return (port & 1) ? WK_VDP_CTRL : WK_VDP_DATA;
        }
#elif VDP_WRITES
        return WK_VDP_IO;
#else
        return WK_IGNORE;  // -DVDP_WRITES=OFF: the real VDP is fitted and takes its own writes
#endif
    }
    // A memory write, as on the breadboard: with a disk attached every one goes to the controller, whose
    // decode of the LATE sample (fdc_decode_write) filters; the mapper stages only a write to this slot
    // (a write elsewhere is aimed at the write sink, bank_register[0]); the menu's decode filters its own.
#if FLOPPY
    if(fdc_on)
        return WK_FDC;
#endif
    switch(cart_mode) {
#if MEGAROM
    case CART_MODE_MAPPER:
        cart_staged = (s & CART_WRITE_MASK) ? 0 : cart_stage_write(megarom_type, BUS_ADDRESS(s));
        return WK_MAPPER;
#endif
#if MENU
    case CART_MODE_MENU:
        return WK_MENU;
#endif
    default:
        return WK_IGNORE;
    }
}

#if VDPW_INLINE_DATA && !V9938
// vdp_bus_apply_write_sample() inlined, with the data-port write (the frequent one: OTIR streams) done in
// place; a control-port write still calls tms9918a_register_write(), unchanged. The port is decoded from
// this LATE sample, as before. Same steps and order as vdp_bus_apply_write_sample + tms9918a_vram_write.
extern volatile uint8_t tms9918a_vram[];
extern volatile uint16_t tms9918a_addr;
extern volatile uint8_t tms9918a_read_ahead, tms9918a_latch;
// tms9918a_vram_write(): store at the address counter, advance it, the byte becomes the read-ahead
static inline __attribute__((always_inline)) void vdp_data_write_inline(uint8_t data) {
    const uint16_t a = tms9918a_addr;
    tms9918a_vram[a] = data;
    tms9918a_addr = (uint16_t)((a + 1) & 0x3fff);
    tms9918a_read_ahead = data;
    tms9918a_latch = 0;
}
static inline __attribute__((always_inline)) void vdp_write_inline(uint32_t s) {
#if !VDPW_NO_COUNTERS
    vdp_bus_captured++;
#endif
    const uint32_t port = (s >> CONFIG_PIN_ADDR_BASE) & 0xff;
    if((port & 0xfe) != CONFIG_VDP_PORT)
        return;
#if !VDPW_NO_COUNTERS
    vdp_bus_vdp_writes++;
#endif
    const uint8_t data = BUS_DATA(s);
    if(port & 1) {
        tms9918a_register_write(data);
        return;
    }
    vdp_data_write_inline(data);
}
#endif

static inline void write_dispatch_late(enum write_kind kind, uint32_t s, const bool fdc_on) {
#if !FLOPPY
    (void)fdc_on;
#endif
    switch(kind) {
    case WK_IGNORE:
        break;
#if VDP_WRITES
    case WK_VDP_IO:
        io_rec(s, IO_W_LATE, 0);  // TEMPORARY DIAGNOSTIC
#if VDPW_INLINE_DATA && !V9938
        vdp_write_inline(s);
#else
        vdp_bus_apply_write_sample(s);
#endif
        break;
#if VDPW_EARLY_PORT
    case WK_VDP_DATA:
        vdp_data_write_inline(BUS_DATA(s));
        break;
    case WK_VDP_CTRL:
        tms9918a_register_write(BUS_DATA(s));
        break;
#endif
#endif
#if FLOPPY
    case WK_FDC:
        if(fdc_on)
            fdc_bus_step(s);
        break;
#endif
#if MENU
    case WK_MENU:
        menu_bus_step(s);
        break;
#endif
#if MEGAROM
    case WK_MAPPER:
        if(!(s & CART_WRITE_MASK))
            cart_finish_write(bank_register, (const uint8_t *)rom_image, cart_mask, cart_staged, BUS_DATA(s));
        break;
#endif
    default:
        // WK_FDC/WK_MENU/WK_MAPPER are unreachable (write_classify_early() never returns one) in a
        // build missing the matching feature -- kept in the enum anyway so its definition doesn't
        // need its own #if chain mirroring write_classify_early's.
        break;
    }
}

static inline void write_process_one(uint32_t s, const bool fdc_on) {
#if WTRACE
    wtrace.w_total++;  // TEMPORARY DIAGNOSTIC
#endif
    if(!write_expect_late) {
        write_pending_kind = write_classify_early(s, fdc_on);
        write_expect_late = true;
    } else {
        write_dispatch_late(write_pending_kind, s, fdc_on);
        write_expect_late = false;
#if STATS
        if(write_pending_kind == WK_MAPPER) {
            stats.mapper_writes++;
            if(!stats_arm_us) {
                CONFIG_ROM_PIO->fdebug = ~0u;  // stalls before the game starts do not count
                CONFIG_VDP_PIO->fdebug = ~0u;
                stats_arm_us = time_us_32() | 1;
            }
        }
#endif
    }
}

// Pops and fully applies every write still queued on generic_write's FIFO, both halves of any
// in-flight pair included. A VDP read must see every write the Z80 made before it (not just VDP
// ones: a write still queued when a read arrives is queued regardless of its own kind), so
// read_classify's RK_VDP_IO case calls this before vdp_bus_handle_read() -- the direct replacement
// for the old, VDP-write-only vdp_bus_drain_writes().
static inline void write_drain(const bool fdc_on) {
    while(!pio_sm_is_rx_fifo_empty(CONFIG_VDP_PIO, GENERIC_WRITE_SM))
        write_process_one(pio_sm_get(CONFIG_VDP_PIO, GENERIC_WRITE_SM), fdc_on);
}

#if VDP_READ_TRACK
// An I/O read the real VDP answers: noinline, so this cold case does not grow core0_run's loop (see
// its comment on how little codegen change that loop tolerates). As RK_VDP_IO, the Z80's own prior
// writes are applied first.
static void __attribute__((noinline)) vdp_track_read(uint32_t sample, bool fdc_on) {
    write_drain(fdc_on);
    vdp_bus_track_read(sample);
}
#endif

// The bus loop calls this when there is no ROM/VDP read request pending. It deals with at most one
// thing per call so a read never waits behind more than a few instructions: a captured write if one
// is waiting (so it takes effect before the Z80 can read any register it affects again), else, once
// in a while, giving the VDP a chance to notice a new frame and raise /INT.
static inline void write_poll(uint *tick, const bool fdc_on) {
    if(!pio_sm_is_rx_fifo_empty(CONFIG_VDP_PIO, GENERIC_WRITE_SM)) {
        const uint32_t w_ = pio_sm_get(CONFIG_VDP_PIO, GENERIC_WRITE_SM);
        write_process_one(w_, fdc_on);
    }
#if VDP_READS
    else if((++*tick & 15) == 0) {
        tms9918a_service();  // a new frame may need to raise /INT -- only meaningful with VDP_READS,
                              // since tms9918a_set_int() is a no-op without it
#if STATS
        if((*tick & 0xfff) == 0)
            stats_check();  // in the existing idle branch: no new work on the busy path
#endif
    }
#else
    else {
        (void)tick;
    }
#endif
}

// Core 0 is the bus core. ROM reads come first: they have the tightest deadline, so they are
// checked on every pass. A captured controller write is handled next, whenever one is waiting,
// so it takes effect before the Z80 can read the register again. Everything else (VDP writes,
// /INT) is handled one small piece at a time in write_poll(), so a ROM request never waits
// behind more than a few instructions.
// The loop body is always_inline into core0_loop() below, once per fdc_on value under FLOPPY&&MENU,
// so that when no disk is attached (every mapper cartridge session) the compiled loop contains NO
// controller code at all, not even a flag test -- see RPROM/plan-floppy-megarom-reconcile.md root
// cause #4: this loop's margin is a few cycles, and merely having the controller hooks in it (inlined
// or called) changes its codegen enough to stall Konami4 games. fdc_on is a compile-time constant in
// each copy; the loop returns, from its idle path only, when fdc_pio_active() no longer matches it.
static inline __attribute__((always_inline)) void core0_run(const bool fdc_on) {
#if !(FLOPPY && MENU)
    (void)fdc_on;
#endif
    uint tick = 0;

#if ROM_EMULATION
#if WTRACE
    wt_loop_us = time_us_32();
#endif
    while(1) {
        if(!pio_sm_is_rx_fifo_empty(CONFIG_ROM_PIO, GENERIC_READ_SM)) {
            const uint32_t sample = pio_sm_get(CONFIG_ROM_PIO, GENERIC_READ_SM);

            switch(read_classify(sample)) {
            case RK_IRRELEVANT:
                rtrace_rec(sample, RT_IGNORED, 0);  // TEMPORARY DIAGNOSTIC
                break;

            case RK_ROM: {
                // Only a read of this slot that is still going gets an answer, so the live pins are
                // checked once, as the breadboard build checked _OE/_CE/_RD: a write, or a stale FIFO
                // entry (core 0 was a whole cycle behind) is dropped, and the bus is never driven. The address is taken from the live pins too, not the sample:
                // the read is still going, so they are this cycle's (the sample may be a stale one, or
                // one read_classify() took during reset before waiting for _MREQ).
                const uint32_t cycle_mask = (1u << CONFIG_PIN_MREQ) | (1u << CONFIG_PIN_SLTSL) | (1u << CONFIG_PIN_RD);
                const uint32_t pins = sio_hw->gpio_in;
                if(!(pins & cycle_mask)) {
                    const uint32_t address = BUS_ADDRESS(pins);
                    // Every kind of cartridge, plain or mapper, reads through bank_register (cart.h)
                    const uint8_t value = cart_read(bank_register, address);
                    pio_sm_put(CONFIG_ROM_PIO, ROM_DATA_SM, value);
                    pio_sm_put(CONFIG_ROM_PIO, ROM_DIRECTION_SM, 0);  // now drive it
                    rtrace_rec(sample, RT_DRIVEN, value);  // TEMPORARY DIAGNOSTIC: after the puts
#if FLOPPY && MENU
                    // Unlike the standalone FLOPPY build, the controller may not have been set up yet
                    // (menu mode boots first; fdc_glue_init only runs, via fdc_attach_prepared, once the user
                    // has picked a .DSK -- see main() below). fdc_fast_state.link is a null pointer
                    // until then, so fdc_pio_active() (true only once it is not) guards every use.
                    // fdc_fast_after_read is a real, out-of-line function (unlike fdc_fast_work below),
                    // so this check was already as cheap as it can be -- no noinline wrapper needed here.
                    if(fdc_on && (address & FDC_DECODE_MASK) == FDC_DECODE_MASK)
                        fdc_fast_after_read(&fdc_fast_state, address & 7);
#elif FLOPPY
                    // A read of a controller register: do the work after the response (the byte
                    // was already staged in the ROM image). Only answered reads get here, so a
                    // refresh cycle that happens to carry a window address does nothing.
                    if((address & FDC_DECODE_MASK) == FDC_DECODE_MASK)
                        fdc_fast_after_read(&fdc_fast_state, address & 7);
#endif
                } else {
                    // Another slot's read, or this slot's but already over. TEMPORARY DIAGNOSTIC:
                    // value = which live pin was high
#if STATS
                    // cold path: the read was already over when core 0 got to it
                    if(stats.rom_late < 8) {
                        const uint32_t i_ = stats.rom_late;
                        stats.late[i_].sample = sample;
                        stats.late[i_].pins = pins;
                        stats.late[i_].rd_level = pio_sm_get_rx_fifo_level(CONFIG_ROM_PIO, GENERIC_READ_SM);
                        stats.late[i_].wr_level = pio_sm_get_rx_fifo_level(CONFIG_VDP_PIO, GENERIC_WRITE_SM);
                        stats.late[i_].wr_state = (uint32_t)write_expect_late | ((uint32_t)write_pending_kind << 1);
                        stats.late[i_].since_arm_us = stats_arm_us ? time_us_32() - stats_arm_us : 0;
                    }
                    stats.rom_late++;
                    if(!stats_late_us)
                        stats_late_us = time_us_32() | 1;
#endif
                    rtrace_rec(sample, RT_LATE,
                        ((pins >> CONFIG_PIN_SLTSL) & 1) | (((pins >> CONFIG_PIN_RD) & 1) << 1));
                }
                break;
            }

#if VDP_READS
            case RK_VDP_IO: {
                write_drain(fdc_on);  // the Z80's own prior writes must be visible to this read
#if WTRACE
                const uint32_t late0 = vdp_bus_late_reads, rd0 = vdp_bus_vdp_reads;  // TEMPORARY DIAGNOSTIC
                vdp_bus_handle_read(sample);
                if(vdp_bus_late_reads != late0)
                    io_rec(sample, IO_R_LATE, 0);
                else if(vdp_bus_vdp_reads != rd0)
                    io_rec(sample, IO_R_ANSWERED, vdp_bus_last_value);
                else
                    io_rec(sample, IO_R_OTHER, 0);
                rtrace_rec(sample, RT_IO, 0);
#else
                vdp_bus_handle_read(sample);
#endif
                break;
            }
#endif
#if VDP_READ_TRACK
            case RK_VDP_TRACK:
                vdp_track_read(sample, fdc_on);
                break;
#endif
            }

#if FLOPPY && MENU
            // noinline (fdc_hot_idle, just above core0_loop): see its own comment. fdc_on is constant
            // here, so the fdc_on == false copy of this loop has no trace of any of this.
            if(fdc_on)
                fdc_hot_idle();
#elif FLOPPY
            // Whether this request was answered or not, give the controller one step. Refresh cycles
            // (and now, since generic_read pushes unconditionally, every other read too) push a
            // request about once per instruction, which is what keeps this running: there is no
            // separate idle spin any more, and adding one back in was tried and made things worse (see
            // the project notes) -- a step starting at an arbitrary point just before a real read is the
            // failure mode the core-gate address check exists to avoid.
            if(fdc_fast_work(&fdc_fast_state))
                fdc_fast_idle(&fdc_fast_state);
#endif
        }
#if FLOPPY && MENU
        else if(fdc_on != fdc_pio_active() && !write_expect_late) {
            return;  // a disk was attached or dropped: core0_loop() picks the matching copy of this loop
        }
#endif

        // Whichever of the controller's window, the menu's window, a mapper cartridge's bank
        // switches or a VDP I/O write this capture belongs to, write_classify_early()/
        // write_dispatch_late() decide -- see their own comment, just above core0_loop.
        write_poll(&tick, fdc_on);

#if WTRACE
        // TEMPORARY DIAGNOSTIC: WTRACE_DUMP_US after the most recent reset, dump to flash and show
        // white (once). Time only checked every 4096 passes.
        {
            static bool wtrace_dumped;
            static uint32_t wtrace_spin;
            if(!wtrace_dumped && wt_reset_us && (++wtrace_spin & 0xfff) == 0 &&
               time_us_32() - wt_reset_us >= WTRACE_DUMP_US) {
                wtrace_dumped = true;
                wtrace.vdp_captured = vdp_bus_captured - wt_base_cap;
                wtrace.vdp_writes = vdp_bus_vdp_writes - wt_base_wr;
                wtrace.vdp_reads = vdp_bus_vdp_reads - wt_base_rd;
                wtrace.vdp_late = vdp_bus_late_reads - wt_base_late;
                wtrace_to_flash();
                tms9918a_set_backdrop(15);
            }
        }
#endif
    }
#else
    // No ROM emulation: only listen for the VDP
    while(1)
        write_poll(&tick, fdc_on);
#endif
}

// Runs the copy of the bus loop that matches whether a disk is attached right now (see core0_run).
#if SCRATCH_HOT
static void __attribute__((noinline)) __scratch_x("core0_loop") core0_loop() {
#else
static void __not_in_flash_func(core0_loop)() {
#endif
#if FLOPPY && MENU
    while(1) {
        if(fdc_pio_active())
            core0_run(true);
        else
            core0_run(false);
    }
#else
    core0_run(true);
#endif
}

int main() {
    // Load the ROM image first: from the SD card if there is a usable one, otherwise from flash
    // (which is timed against the boot clock, so this has to happen before the clock is raised).
    // Nothing is served and no picture is drawn until this is done, so a picture on the VGA
    // output means the image is loaded.
#if ROM_EMULATION
    const enum rom_source source = rom_load();
#endif

    // The RP2350 is only rated to 150 MHz. Raise the core voltage before overclocking.
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    busy_wait_us(1000);
    set_sys_clock_khz((uint32_t)(CONFIG_SYSCLOCK * 1000), true);
#if (FLOPPY || MENU) && SD_ROM
    rom_sd_clock_changed();  // the SPI divider was set at the boot clock
#endif

    // Capture bus cycles for the VDP
    tms9918a_init();
    vdp_bus_init();
    // generic_write (bus_write.pio): every Z80 write, memory or I/O -- must come before rom_init(),
    // same as vdp_bus_init(), since bus_write_init() also releases pad isolation on the pins it reads.
    bus_write_init();

#if BUS_PULLS
    // Bus input pulls, for the rig: the RP2350 pad default is a pull-down, which we never changed. The STM32
    // project used pull-ups on every bus pin. 1 = pull-up, 2 = none, 3 = pull-down (the whole slot bus:
    // GPIO 1-31).
    for(uint pin = CONFIG_PIN_BUS_FIRST; pin <= CONFIG_PIN_BUS_LAST; pin++) {
#if BUS_PULLS == 1
        gpio_pull_up(pin);
#elif BUS_PULLS == 2
        gpio_disable_pulls(pin);
#else
        gpio_pull_down(pin);
#endif
    }
#endif

#if ROM_EMULATION && SD_ROM
    // Until the machine sets up the VDP itself the picture is a blank colour. Use it to say where
    // the ROM came from: dark green = the SD card, anything else = the flash image, and the colour
    // says why the card could not be used (see rom_status_colour in rom.h).
    (void)source;
    tms9918a_set_backdrop(rom_status_colour());
#elif ROM_EMULATION
    (void)source;
#endif

#if FLOPPY && !MENU
    // The controller stages its registers into the ROM image: after the image is loaded, before it is served
    fdc_glue_init((uint8_t *)rom_image);
#endif
#if MENU
    // The menu window is staged into the cartridge buffer; only used while cart_mode is CART_MODE_MENU
    menu_glue_init((uint8_t *)rom_image);
#endif

#if ROM_EMULATION
    // Start emulating the ROM. From here on this chip answers ROM reads and drives D0-D7, so
    // the real ROM must not be fitted. Must come after vdp_bus_init()/bus_write_init(), which
    // reset the data pins to plain inputs before rom_init() hands them to the ROM PIO.
    rom_init();
#endif

    // Core 1 draws the picture. It only reads the VDP state and reports frame/sprite events.
    multicore_launch_core1(core1_main);

    core0_loop();
}
