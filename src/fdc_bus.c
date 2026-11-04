#include "fdc_bus.h"

#include <hardware/structs/timer.h>
#include <pico/stdlib.h>
#include "board_config.h"
#include "fdc_decode.h"
#include "floppy.h"
#include "rom.h"

#ifdef FLOPPY_TRACE
#include <hardware/flash.h>
#include <hardware/sync.h>
struct fdc_trace_data fdc_trace = { .magic = FDC_TRACE_MAGIC, .rq_stop = ~0u };
uint32_t fdc_trace_now(void) { return timer_hw->timerawl; }

// The trace is copied to flash once, FLOPPY_TRACE_DUMP_S seconds after the last disk was attached, so it can be
// read out after a power cycle (see tools/fdc_trace.py). Done on core 1 (fdc_glue_poll_core1): core 0's bus loop
// keeps running from RAM (the whole firmware is copied to RAM), core 1 stops drawing and polling for the ~0.5 s
// it takes.
#ifndef FLOPPY_TRACE_DUMP_S
#define FLOPPY_TRACE_DUMP_S 60
#endif
#define FDC_TRACE_FLASH_OFFSET 0x3F0000u
static uint32_t trace_attach_us;
static bool trace_armed;

static void trace_to_flash(void) {
    const uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FDC_TRACE_FLASH_OFFSET, (sizeof fdc_trace + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1));
    flash_range_program(FDC_TRACE_FLASH_OFFSET, (const uint8_t *)&fdc_trace,
        (sizeof fdc_trace + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1));
    restore_interrupts(ints);
}
#endif

volatile uint32_t fdc_bus_windows;
volatile uint32_t fdc_bus_writes;
volatile uint32_t fdc_bus_rejects;

struct fdc_fast fdc_fast_state;
struct fdc_link fdc_link_state;

static struct fdc_cmd cmd_state;
static struct floppy floppy_state;
static uint8_t cylinder_buffer[FDC_MAX_DRIVES][FDC_CYL_BYTES];

// No disk image is attached yet: the controller answers every command "not ready". Reading and
// writing the disk file comes with the SD engine. Whether a drive is actually allowed to write is
// floppy_attach's own read_only argument (see fdc_glue_init/fdc_attach_prepared, gated by FLOPPY_WRITE):
// these callbacks are reachable either way, but floppy.c/fdc_cmd.c never call the write ones for a
// drive attached read only.
#if SD_ROM
// Sector access to the card, blocking (one sector at a time, from core 1). A blocking write holds
// core 1 for as long as the card takes to program a block (up to ~250 ms): never used on the real
// disk-image attaches below (they use the stepwise pair instead), kept as the blocking fallback the
// same way card_read is for anything that does not need the stepwise form.
static bool card_read(void *ctx, uint32_t lba, uint8_t *buf) {
    (void)ctx;
#if FLOPPY_DIAG == 1
    (void)lba; (void)buf;
    return false;  // diagnostic: a drive is attached but every read fails at once, without touching the SPI
#else
    return sd_read_sector(&rom_sd_card, lba, buf) == SD_OK;
#endif
}
static bool card_write(void *ctx, uint32_t lba, const uint8_t *buf) {
    (void)ctx;
    return sd_write_sector(&rom_sd_card, lba, buf) == SD_OK;
}

// The same read in short steps, one per poll (each clocks at most SD_STEP_BYTES bytes): a blocking read holds core 1
// for about 0.5 ms, and core 1 draws the VGA picture, which loses sync if it is stalled for more than about 250 us
static struct sd_read sd_op;
static bool card_read_begin(void *ctx, uint32_t lba, uint8_t *buf) {
    (void)ctx;
#if FLOPPY_DIAG == 1
    (void)lba; (void)buf;
    return false;
#else
    sd_read_begin(&rom_sd_card, &sd_op, lba, buf);
    return true;
#endif
}
static int card_read_step(void *ctx) {
    (void)ctx;
    const enum sd_result r = sd_read_step(&rom_sd_card, &sd_op);
    return r == SD_PENDING ? 0 : (r == SD_OK ? 1 : -1);
}

// The same for writes: a card can legitimately take up to ~250 ms to program a block, which this
// breaks into SD_STEP_BYTES-ish steps instead of one long stall (see sd_write_step).
static struct sd_write sd_wop;
static bool card_write_begin(void *ctx, uint32_t lba, const uint8_t *buf) {
    (void)ctx;
    sd_write_begin(&rom_sd_card, &sd_wop, lba, buf);
    return true;
}
static int card_write_step(void *ctx) {
    (void)ctx;
    const enum sd_result r = sd_write_step(&rom_sd_card, &sd_wop);
    return r == SD_PENDING ? 0 : (r == SD_OK ? 1 : -1);
}
#else
static bool card_read(void *ctx, uint32_t lba, uint8_t *buf) { (void)ctx; (void)lba; (void)buf; return false; }
static bool card_write(void *ctx, uint32_t lba, const uint8_t *buf) { (void)ctx; (void)lba; (void)buf; return false; }
#endif

#ifdef FLOPPY_FLASHDISK
// Diagnostic and a feature in its own right: the disk image sits in flash (loaded with
//   picotool load test.dsk -t bin -o 0x10100000
// for FLOPPY_FLASHDISK=0x100000) and each sector is copied straight out of the mapped flash: no SPI, no card.
// The file is one contiguous run, so a disk sector number is also the flash sector number.
static struct fat_extent flashdisk_extent = { 0, 0 };
static struct fat_file flashdisk_file = { &flashdisk_extent, 1, 1440, 737280 };

static bool flash_read(void *ctx, uint32_t lba, uint8_t *buf) {
    (void)ctx;
    if(lba >= flashdisk_file.sectors)
        return false;
    const volatile uint8_t *src = (const volatile uint8_t *)(XIP_BASE + FLOPPY_FLASHDISK + (uint32_t)lba * 512);
    for(int i = 0; i < 512; i++)
        buf[i] = src[i];
    return true;
}
#endif

// Not static: fdc_pio_active() (fdc_bus.h) reads it directly, as a static inline, so main.c's hot
// core0_loop checks it with no function-call overhead at all -- see the note there and
// RPROM/plan-floppy-megarom-reconcile.md. (rom_bank_pio_active() in main.c is the same trick, one
// file over: a plain global read the compiler can keep inlined, not a cross-TU call.)
//
// Before refactor-pio-sm.md this tracked whether fdc_write_program was the one loaded on the
// shared write-capture state machine (a PIO program-offset indicator, -1 if not loaded). generic_write
// (bus_write.c) is always resident now -- there is nothing left to load or unload -- so this is a
// plain mode flag with the exact same meaning and call sites: is the controller currently entitled
// to slot 1's register window?
volatile bool fdc_mode_active;  // set by core 1 (the disk swap), read by core 0's loop: volatile, or core 0 can keep a stale copy in a register

// Claims the controller's window: only once this core is actually the one using it (at boot if
// built alongside FLOPPY and not MENU; otherwise only once a swap to disk mode has happened -- see
// menu_pio_disable/fdc_pio_enable in menu_bus.c).
static void fdc_pio_enable(void) {
#if FLOPPY_STAGES & 2
    fdc_mode_active = true;
#endif
}

#if MENU
// The mirror of fdc_pio_enable, for giving the window back to whoever wants it next (the menu
// support, if the user picks a plain ROM with no disk -- see menu_bus.c). Not used when this is
// built without MENU: there is nothing else that would ever want it back.
void fdc_pio_disable(void) {
#if FLOPPY_STAGES & 2
    fdc_mode_active = false;
#endif
}
#endif

void fdc_glue_init(uint8_t *rom) {
#ifdef FLOPPY_FLASHDISK
    const struct floppy_io io = { NULL, flash_read, card_write };
#else
    const struct floppy_io io = { NULL, card_read, card_write, card_read_begin, card_read_step,
        card_write_begin, card_write_step };
#endif
    floppy_init(&floppy_state, &io);
#if SD_ROM
    // TEST.DSK, if rom_load() found it on the card: drive A. The flash-backed variant stays
    // permanently write protected -- there is no flash-write path, and building one is out of
    // scope (see the floppy write-support plan); the SD-backed disk honours FLOPPY_WRITE.
#ifdef FLOPPY_FLASHDISK
    floppy_attach(&floppy_state, 0, &flashdisk_file, true);
#else
    if(rom_disk_ok)
        floppy_attach(&floppy_state, 0, &rom_disk_file, !FLOPPY_WRITE);
#endif
#endif

    fdc_fast_init(&fdc_fast_state, &fdc_link_state, rom, cylinder_buffer[0], cylinder_buffer[1]);
    fdc_cmd_init(&cmd_state, &fdc_link_state, &floppy_state, cylinder_buffer[0], cylinder_buffer[1]);
#if FLOPPY_DIAG == 2
    cmd_state.load_tried[0] = true;  // diagnostic: no start-up load of cylinder 0
#endif

    fdc_pio_enable();
}

#if MENU
static struct fat_extent swap_disk_extents[1440];  // a fully scattered 720K file on 512 byte clusters
static struct fat_file swap_disk_file;

// Attach a freshly chosen disk as drive A, with a completely fresh controller: any previous disk's
// cylinder cache, registers and command state are discarded. `rom` is the 128 KB image (this may be
// the first time the controller has ever run, if MENU booted straight into menu mode rather than
// fdc_glue_init -- so `rom` is taken as a parameter here, not remembered from an earlier call).
// `dir` and `name` locate the chosen file (found in the menu's MSX directory, by its display name
// -- long or short, matched case-insensitively); its extents are rebuilt here, independent of
// whatever TEST.DSK fdc_glue_init might separately have found at boot. The caller is responsible
// for there being somewhere for fdc_pio_enable() to put its program: see menu_bus.c, which calls
// menu_pio_disable() first.
enum fat_result fdc_prepare_disk(const struct fat_dir *dir, const char *name) {
    struct fat_dirent e;
    enum fat_result r = fat_find_by_name(card_read, NULL, dir, name, &e);
    if(r != FAT_OK)
        return r;
    if(e.is_dir)
        return FAT_ERR_NOT_FOUND;
    return fat_file_extents(&e, swap_disk_extents, sizeof swap_disk_extents / sizeof swap_disk_extents[0],
        &swap_disk_file);
}

void fdc_attach_prepared(uint8_t *rom) {
    const struct floppy_io io = { NULL, card_read, card_write, card_read_begin, card_read_step,
        card_write_begin, card_write_step };
    floppy_init(&floppy_state, &io);
    floppy_attach(&floppy_state, 0, &swap_disk_file, !FLOPPY_WRITE);
    fdc_fast_init(&fdc_fast_state, &fdc_link_state, rom, cylinder_buffer[0], cylinder_buffer[1]);
    fdc_cmd_init(&cmd_state, &fdc_link_state, &floppy_state, cylinder_buffer[0], cylinder_buffer[1]);
#ifdef FLOPPY_TRACE
    trace_attach_us = time_us_32();
    trace_armed = true;
#endif
    fdc_pio_enable();
}

bool fdc_card_busy(void) {
    return floppy_state.inflight;
}
#endif

void fdc_bus_step(uint32_t s) {
    fdc_bus_windows++;
    unsigned reg;
    uint8_t data;
    if(fdc_decode_write(s, &reg, &data)) {
        fdc_bus_writes++;
        fdc_fast_write_now(&fdc_fast_state, reg, data);  // applied now: the Z80's next read must see its effect
    } else {
        fdc_bus_rejects++;
    }
}

const struct fdc_stats *fdc_bus_stats(void) {
    return &cmd_state.stats;
}

void fdc_glue_poll_core1() {
#if FLOPPY_STAGES & 4
#if MENU
    // Unlike the standalone FLOPPY build, this may be called (from vga.c, unconditionally, so it
    // does not need to know the controller's state) before fdc_attach_prepared has ever run.
    if(!fdc_pio_active())
        return;
#endif
    // Rate limited: this is called from the render loop's idle spin
    static uint32_t last_us;
    const uint32_t now = time_us_32();
    if((uint32_t)(now - last_us) < 20)
        return;
    last_us = now;
#ifdef FLOPPY_TRACE
    if(trace_armed && now - trace_attach_us >= FLOPPY_TRACE_DUMP_S * 1000000u) {
        trace_armed = false;
        trace_to_flash();
    }
#endif
    fdc_cmd_poll(&cmd_state, now / 1000);
#endif
}
