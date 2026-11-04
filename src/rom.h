#pragma once

#include <stdint.h>

// The state machines on CONFIG_ROM_PIO. The data state machine drives D0-D7 for both ROM reads
// and VDP reads; ROM_DIRECTION_SM (generic_read_drive) decides when the pins are outputs, for both
// kinds of read uniformly (see rom.pio).
enum {
    ROM_DIRECTION_SM = 0,
    ROM_DATA_SM = 1,
    GENERIC_READ_SM = 2,
};

// The cartridge buffer (see rom_image.c and cart.h)
extern uint32_t rom_image[];

// Where the ROM image in rom_image came from
enum rom_source {
    ROM_FROM_NONE,   // rom_load() has not run
    ROM_FROM_SD,     // DEFAULT.ROM from the SD card
    ROM_FROM_SD_REPORT, // DEFAULT.ROM could not be loaded: a cartridge that prints why (cmake -DSD_REPORT)
    ROM_FROM_EMBEDDED, // the cartridge built into the firmware (cmake -DCART_ROM=<file>)
    ROM_FROM_FLASH,  // the image stored in flash slot 1 (no SD card, or it could not be used)
};

// Load the ROM image into rom_image, and say where it came from. With SD_ROM the SD card is tried
// first (MENU.ROM, then DEFAULT.ROM, then the MSX directory's first file; a file larger than
// CART_MAX_BYTES is refused), then the built-in cartridge, then the flash image. Call once at boot,
// before the ROM state machines start and before the system clock is raised (the flash copy reads
// flash, which is timed against the slower boot clock).
enum rom_source rom_load();

// A colour (TMS9918A palette index) saying how the SD load went, for the boot status picture:
//   12 dark green   loaded from the SD card
//    6 dark red     nothing answered on the SD bus: MISO idles high (wiring, power, no card)
//   14 grey         MISO is held low: a short, or a wrong or unconnected MISO pin (the boot also
//                   takes a second or so longer than usual before the picture appears)
//   10 dark yellow  the card never became ready
//   13 magenta      an unsupported card (MMC, or a voltage range it does not accept)
//    4 dark blue    the card refused a command
//    7 cyan         the card initialised but reading data from it failed (try a lower -DSD_SPI_HZ)
//   15 white        the card works but the file or filesystem is not usable (no DEFAULT.ROM,
//                   FAT12, exFAT, no partition, empty file, too big for the slot, corrupt)
uint8_t rom_status_colour();

// For a debugger: what happened on the SD card. Only meaningful when SD_ROM is enabled.
#include "fat.h"
#include "sd_card.h"
extern volatile enum rom_source rom_source;
extern volatile enum sd_result rom_sd_result;    // SD_OK, or why the card could not be used
extern volatile enum fat_result rom_fat_result;  // FAT_OK, or why the file could not be read
extern volatile uint32_t rom_sd_file_size;       // length of DEFAULT.ROM, if it was found

// Set up and start the ROM state machines. The image must already be loaded. From here on every
// Z80 read cycle is served by whoever polls GENERIC_READ_SM (see main.c).
void rom_init();

#if (FLOPPY || MENU) && SD_ROM
#include "fat.h"
#include "sd_card.h"
// With the floppy support, the menu support, or both, the SD card is kept initialised after boot
// for runtime access: the floppy's disk image, or the menu's directory listing.
extern struct sd_card rom_sd_card;

// Call after the system clock has been raised: the SPI clock divider was computed at the boot clock
void rom_sd_clock_changed();
#endif

#if FLOPPY && SD_ROM
// TEST.DSK is located on the card at boot (a read only disk image for drive A). rom_disk_ok is
// false if there is no such file or it is not a supported disk (720K or 360K), and then the
// controller has no drive attached.
extern struct fat_file rom_disk_file;
extern volatile bool rom_disk_ok;
extern volatile enum fat_result rom_disk_result;
#endif

// The cartridge in the slot (cart.h). rom_image is the cartridge buffer (CART_MAX_BYTES, 0xff past the
// end of the file); bank_register says which 8K of it, or of cart_ff (8K of 0xff), each 8K region of
// the slot shows. Defined in main.c, next to the bus loop that reads them.
#include "cart.h"
enum cart_mode {
    CART_MODE_PLAIN,   // a plain ROM: writes to the slot are ignored
    CART_MODE_MAPPER,  // a megaROM: writes go to its bank registers (needs MEGAROM)
    CART_MODE_MENU,    // kcmfs (MENU.ROM): writes to $8000-$80FF are the menu protocol (needs MENU)
};
extern cart_bank_t bank_register[CART_BANK_SLOTS];
extern volatile enum megarom_type megarom_type;
extern volatile uint32_t cart_mask;
extern volatile enum cart_mode cart_mode;
extern uint8_t cart_ff[CART_REGION_BYTES];

// Make the `size` bytes now in rom_image the cartridge the slot shows, as a `type` cartridge (forced to
// MEGAROM_GENERIC without MEGAROM), in its reset state. The rest of rom_image must already be 0xff.
void rom_cart_setup(uint32_t size, enum megarom_type type);

// The mapper a file name asks for (.KONAMI4, .KONAMI5, .ASCII8, .ASCII16, any case, as the STM32
// project names them), or for any other name, the mapper cart_guess_type finds in the file's code.
enum megarom_type rom_cart_type(const char *name, const uint8_t *rom, uint32_t size);

#if MENU
// What the slot started with at boot
enum rom_menu_source {
    ROM_MENU_FROM_DEFAULT,      // not MENU.ROM: DEFAULT.ROM, the MSX directory's first file, or the fallbacks
    ROM_MENU_FROM_MENUROM,      // MENU.ROM, in the root
    ROM_MENU_FROM_FIRST_FILE,   // the first file found in the MSX directory (no MENU.ROM, no DEFAULT.ROM)
};
extern volatile enum rom_menu_source rom_menu_source;

// Find `name` in `dir` (the menu's MSX directory), load it into rom_image and make it the slot's
// cartridge (rom_cart_type picks the mapper). Called on core 1 when the user picks a file in kcmfs. False
// if the file could not be found, was a directory, was too big, or could not be read (rom_image may
// then be part written: the slot shows whatever is there).
bool rom_bank_load(const struct fat_dir *dir, const char *name);
#endif
