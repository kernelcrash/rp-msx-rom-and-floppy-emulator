#pragma once

#include <stdint.h>
#include "board_config.h"
#include "menu.h"

// The bus side of the menu support: writes to its window arrive via generic_write (bus_write.pio/
// .c), classified and dispatched to menu_bus_step by main.c's core0_loop -- see refactor-pio-sm.md.
// This file owns the menu model and keeps the window and listing staged into megarom_rom (main.c),
// where the ordinary ROM read path already serves them with no extra latency (the same trick
// fdc_fast.c's staged registers use). See menu.h for the protocol.
//
// When FLOPPY is also built, this window is shared with it (the Z80 is either running kcmfs,
// talking to this window, or it has been handed a freshly chosen ROM or disk and the menu window
// has no further use until the menu is reached again, which this project does not yet support).
// Which one is entitled to it is tracked by a plain mode flag now (fdc_mode_active, fdc_bus.h);
// before refactor-pio-sm.md this was which PIO program was loaded on their one shared state
// machine. Swapping happens as part of menu_glue_poll_core1 noticing a load command: see the .c
// file and fdc_prepare_disk/fdc_attach_prepared in fdc_bus.h.

#ifndef MENU
#define MENU 0
#endif

// Where the window is in the cartridge buffer: MENU.ROM is a plain 32K cartridge at $4000, so the
// Z80's $8000-$80FF is offset $4000 of the buffer.
#define MENU_STAGE_BASE 0x4000u

extern struct menu menu_bus_state;

// Set up the menu support: claims the window. `rom` is megarom_rom (main.c): the window and
// listing are staged into it as soon as they change, at the displacement above.
void menu_glue_init(uint8_t *rom);

// Core 0: decode and apply one captured write. `s` is generic_write's LATE 32 pin snapshot (data
// valid); main.c's write dispatch calls this once it has decided the write belongs to the menu
// window (i.e. neither fdc_pio_active() nor rom_bank_pio_active() claims it). Stages the one byte
// that changed; cheap enough for the bus loop.
void menu_bus_step(uint32_t s);

// Core 1: advance the menu support a step (rate limited, like fdc_glue_poll_core1). Runs the
// directory listing (a card access, not something core 0 may ever do) when one has been asked for,
// and the ROM/disk swap when the user has picked something (see the .c file for what that means if
// FLOPPY is also built). Either way the result is staged into rom_image once it is ready, never on
// core 0's path.
void menu_glue_poll_core1(void);

#if FLOPPY || MEGAROM
// The mirror of fdc_pio_disable/rom_bank_pio_claim: give up the window so the controller or a
// mapper cartridge can claim it instead. Safe to call whether or not the menu currently owns it.
void menu_pio_disable(void);
#endif
