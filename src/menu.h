#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "fat.h"

// The protocol kcmfs speaks (msx-rom-and-floppy-emulator/kcmfs/menu.asm), replicated byte for byte
// so that existing binary works unmodified against this firmware. Read from the assembly source and
// the STM32 firmware's write_slot_memory/load_directory (interrupt.S, util.c):
//
//   $8000 (MENU_CMD)    the Z80's own write stays here until something clears it (there is no
//                       separate "ready" flag): kcmfs writes 0x80 to ask for a directory listing,
//                       then polls this same byte, with its top bit as "still busy", until a
//                       listing clears it back to 0. It writes 0x40 to ask for the chosen file to
//                       be loaded and immediately does RST 0 (a software warm boot): it does not
//                       poll after that, so nothing needs to clear this byte for that command.
//   $8002 (MENU_COUNT)  read-only: how many files the last listing found (at most 255; kcmfs also
//                       expects $8003 to read 0, so the pair can be used as a 16 bit count)
//   $8080-$80FE         write: kcmfs copies the chosen filename here (NUL terminated, at most 127
//                       bytes) before writing MENU_CMD_LOAD
//   $8100 and up        read-only: one 128 byte, NUL terminated slot per file, in listing order
//     (up to MENU_MAX_FILES = ITEMS_PER_PAGE (20) * MAX_PAGES (9) in menu.asm, kcmfs's own limit)
//
// Only $8000-$80FF is ever written by the Z80 (kcmfs's own write_slot_memory restricts it to that,
// and nothing here needs more); everything from $8100 is filled directly by menu_run_list, never
// over the simulated bus -- the same trick fdc_fast.c's staged registers use for reads.
//
// Files and subdirectories of the root's "MSX" directory are listed alike, as upstream's load_directory does: a
// subdirectory is a multi-disk game, and picking it loads one of its disks (disk_pick.h, menu_bus.c).

#define MENU_WINDOW_BYTES 256      // $8000-$80FF
#define MENU_FNAME_OFFSET 0x80     // $8080, within the window
#define MENU_SLOT_BYTES 128        // one listing entry
#define MENU_MAX_FILES 180         // ITEMS_PER_PAGE (20) * MAX_PAGES (9) in menu.asm
#define MENU_LISTING_BYTES (MENU_MAX_FILES * MENU_SLOT_BYTES)

#define MENU_CMD_LIST 0x80
#define MENU_CMD_LOAD 0x40

struct menu {
    uint8_t window[MENU_WINDOW_BYTES];     // $8000-$80FF as the Z80 sees it (read and write)
    uint8_t *listing;                      // $8100 and up as the Z80 sees it (read only): MENU_LISTING_BYTES,
                                           // the caller's (the cartridge image itself in the firmware)
    uint32_t file_count;
    bool list_pending;   // a 0x80 write is waiting to be actioned (menu_run_list)
    bool load_pending;   // a 0x40 write is waiting to be actioned (menu_take_load)
};

// `listing` is MENU_LISTING_BYTES of the caller's, cleared by each menu_run_list.
void menu_init(struct menu *m, uint8_t *listing);

// A captured Z80 write to $8000 + offset (0..255). Anything outside that range is not part of the
// window and must not be passed here.
void menu_write(struct menu *m, unsigned offset, uint8_t value);

// A Z80 read of $8000 + offset (0..MENU_WINDOW_BYTES+MENU_LISTING_BYTES-1): serves the live window
// and the listing. An offset past the end of the listing reads as 0xff, matching an unprogrammed
// byte elsewhere in this project's ROM image.
uint8_t menu_read(const struct menu *m, unsigned offset);

// Is there a directory listing or a file load command waiting to be actioned?
static inline bool menu_work(const struct menu *m) { return m->list_pending || m->load_pending; }

// Run the directory listing: walks the root's "MSX" directory with fat_list_dir and fills the
// listing buffer, $8002 and $8003, then clears the busy bit in $8000 (mirroring the STM32
// firmware's explicit *p = 0x00 once load_directory returns) and list_pending. One call does the
// whole thing; on real hardware this will need to be a stepwise poll instead, like the floppy
// controller's cylinder loads, so it never blocks the renderer -- not needed for this host model.
// A missing or unreadable MSX directory is reported as zero files, not a hang.
void menu_run_list(struct menu *m, fat_read_sector_fn read, void *ctx);

// After a load command (menu_work() having reported load_pending): the filename the Z80 chose,
// taken from what it wrote at MENU_FNAME_OFFSET, NUL terminated. Clears load_pending. The pointer
// is into `m` and is only valid until the next call that touches the window.
const char *menu_take_load(struct menu *m);
