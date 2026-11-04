#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "board_config.h"
#include "fat.h"
#include "fdc.h"

// The bus side of the floppy controller: writes to its register window arrive via generic_write
// (bus_write.pio/.c), classified and dispatched to fdc_bus_step by main.c's core0_loop -- see
// refactor-pio-sm.md. This file owns the controller state and decides whether it is the one
// currently entitled to that window (fdc_mode_active, below).

#ifndef FLOPPY
#define FLOPPY 0
#endif

// The controller, shared by the bus loop (core 0) and the render loop (core 1). core 0 uses fdc_fast
// directly; core 1 only ever calls fdc_glue_poll_core1().
extern struct fdc_fast fdc_fast_state;
extern struct fdc_link fdc_link_state;

// Set up the controller: stage its registers into the ROM image (so this must come after the image
// is loaded and before ROM serving starts). `rom` is the slot's 64K, laid out like the slot.
void fdc_glue_init(uint8_t *rom);

// Core 0: decode and apply one captured write. `s` is generic_write's LATE 32 pin snapshot (data
// valid); main.c's write dispatch calls this once it has decided the write belongs to the
// controller's window (see fdc_pio_active() below).
void fdc_bus_step(uint32_t s);

// Core 1: advance the controller a step (rate limited)
void fdc_glue_poll_core1();

#if MENU
// Is the controller currently the one entitled to slot 1's upper page writes? Not static there, so
// this can be a static inline instead of a real cross-TU call -- see its own comment. Was a PIO
// program-claim indicator before refactor-pio-sm.md (generic_write is always resident now, nothing
// to claim); kept as a plain mode flag with the exact same meaning and call sites.
extern volatile bool fdc_mode_active;

// Only meaningful (and only needed) when MENU is also built: main.c's core0_loop uses it to decide
// which of fdc_bus_step/menu_bus_step (or, with MEGAROM too, rom_bank_apply_write) a capture
// belongs to -- checked on every pass of the hot loop, so this must stay a static inline, not a
// real function call: the compiler already does exactly this for rom_bank_pio_active() in main.c
// (same file as its caller, trivially inlined), and a cross-TU call here was real, measurable
// latency on a path with only a couple hundred ns of slack (see RPROM/plan-floppy-megarom-reconcile.md).
static inline bool fdc_pio_active(void) {
    return fdc_mode_active;
}

// Give up the controller's window (menu_bus.c uses the same window for its own, mutually
// exclusive, write capture: see the note in CMakeLists.txt). Safe to call whether or not the
// controller currently owns it.
void fdc_pio_disable(void);

// Attaching a disk picked in the menu, in two halves so that all the card work is done before the MSX can
// see the disk ROM. fdc_prepare_disk finds `name` (the chosen file's display name, long or short, matched
// case-insensitively, as fat_find_by_name does) in `dir`, the menu's MSX directory, and records where its
// sectors are: nothing visible changes. fdc_attach_prepared then attaches it as drive A with a completely
// fresh controller, staging the registers into `rom` (the slot's 64K, laid out like the slot), and claims the
// window: quick, no card access.
enum fat_result fdc_prepare_disk(const struct fat_dir *dir, const char *name);
void fdc_attach_prepared(uint8_t *rom);

// Core 1: is a disk sector half way through the SPI (a stepwise read or write begun, not finished)? Another card
// access must wait until it is not.
bool fdc_card_busy(void);
#endif

// The command engine's counters (for the trace overlay)
const struct fdc_stats *fdc_bus_stats(void);

// Debug counters, readable from a debugger
extern volatile uint32_t fdc_bus_windows;    // writes into the window captured (before checking)
extern volatile uint32_t fdc_bus_writes;     // of those, accepted as controller writes
extern volatile uint32_t fdc_bus_rejects;    // rejected (noise, slot 0, not a write...)
