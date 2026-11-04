#pragma once

// Pin map and resources for a directly connected RP2350B (no level shifters).
// See ../RPROM/plan-tms9918a.md for the reasoning behind the layout.

// 10 x the 25.2 MHz VGA pixel clock. Gives integer PIO dividers (data SM /5, hsync/vsync /80).
// Fallback if this is not stable: 201.6 (8 x pixel clock, dividers /4 and /64), selected with
// cmake -DSYSCLOCK_MHZ=201.6. Any other value must keep the VGA dividers integral.
#ifndef CONFIG_SYSCLOCK
#define CONFIG_SYSCLOCK 252.0 /* MHz */
#endif

// Bus pins for an MSX cartridge slot (msx-cart branch), all inside the gpiobase=0 window (GPIO 0-31)
// of the ROM and write PIOs. GPIO 0 is left free (a PSRAM chip select later: it is a QMI CS1 pin).
#define CONFIG_PIN_DATA_BASE 1  /* D0-D7 = GPIO 1-8 */
#define CONFIG_PIN_MREQ 9       /* Z80 _MREQ */
#define CONFIG_PIN_SLTSL 10     /* the slot's _SLTSL (decoded by the MSX from A14/A15 and the slot register) */
#define CONFIG_PIN_IORQ 11      /* Z80 _IORQ */
#define CONFIG_PIN_WR 12        /* Z80 _WR */
#define CONFIG_PIN_RD 13        /* Z80 _RD */
#define CONFIG_PIN_INT 14       /* Z80 _INT, open drain (only with VDP reads, not ported yet) */
#define CONFIG_PIN_BUSDIR 15    /* reserved: _BUSDIR, needed before this cartridge can answer an I/O read */
#define CONFIG_PIN_ADDR_BASE 16 /* A0-A15 = GPIO 16-31 */
// Every pin of the slot bus: initialised as inputs and given the BUS_PULLS pulls
#define CONFIG_PIN_BUS_FIRST CONFIG_PIN_DATA_BASE
#define CONFIG_PIN_BUS_LAST 31

// The data byte and the address in a 32 bit GPIO snapshot (GPIO n = bit n)
#define BUS_DATA(s) ((uint8_t)((s) >> CONFIG_PIN_DATA_BASE))
#define BUS_ADDRESS(s) (((s) >> CONFIG_PIN_ADDR_BASE) & 0xffffu)

// The TMS9918A answers on I/O ports 0x98 (data) and 0x99 (control). Only A0-A7 are decoded, as
// on an MSX1, so the upper address lines are ignored.
#define CONFIG_VDP_PORT 0x98

// VGA pins, inside the gpiobase=16 window (GPIO 16-47) of the VGA PIO.
// RGB keeps the same relative order as the original AppleII-VGA GPIO 14-22.
#define CONFIG_PIN_RGB_BASE 32 /* 9 pins, GPIO 32-40 */
#define CONFIG_PIN_HSYNC 41
#define CONFIG_PIN_VSYNC 42
#define CONFIG_VGA_GPIOBASE 16

// SD card, in SPI mode, on SPI1 (GPIO 40-47 are SPI1: within each group of four the pins are RX, CSn,
// SCK, TX). Only used while loading the ROM image at boot. GPIO 43 (ADC 3) is the NEXT/PREV buttons (BUTTONS).
// GPIO 40-47 are not 5V tolerant, which is fine for a 3.3 V card.
#define CONFIG_PIN_SD_MISO 44 /* SPI1 RX, card DAT0 */
#define CONFIG_PIN_SD_CS 45   /* plain GPIO (not the SPI CSn function), card DAT3/CS */
#define CONFIG_PIN_SD_SCK 46  /* SPI1 SCK, card CLK */
#define CONFIG_PIN_SD_MOSI 47 /* SPI1 TX, card CMD */
// NEXT and PREV on one ADC pin: 10k to 3V3, NEXT shorts it to GND, PREV pulls it to GND through 10k
#define CONFIG_PIN_BUTTONS 43
#ifndef CONFIG_SD_SPI_HZ
#define CONFIG_SD_SPI_HZ 8000000 /* bus speed after initialisation; initialisation itself runs at 400 kHz */
#endif

// Pulls on the bus input pins (GPIO 1-31): 0 = leave the pad default (a pull-down), 1 = pull-up (what the STM32
// project used), 2 = none, 3 = pull-down. Set with cmake -DBUS_PULLS=n.
#ifndef BUS_PULLS
#define BUS_PULLS 0
#endif

// The WD2793 floppy controller emulation at $7FF8-$7FFF (and its mirrors) of slot 1. Set with cmake
// -DFLOPPY=ON. Needs the ROM read path's core gate (ROM_CORE_GATE).
#ifndef FLOPPY
#define FLOPPY 0
#endif
#ifndef MENU
#define MENU 0
#endif
// Cartridge mapper ("megaROM") support: Konami4, Konami5 (no SCC), ASCII8, ASCII16, picked by file
// extension or guessed from the code (cart.h). Set with cmake -DMEGAROM=OFF for plain ROMs only.
#ifndef MEGAROM
#define MEGAROM 0
#endif
// Allow writes to the attached disk (SAVE and other writes from MSX-DOS/Disk BASIC), instead of
// attaching it read only. Set with cmake -DFLOPPY_WRITE=ON; needs FLOPPY.
#ifndef FLOPPY_WRITE
#define FLOPPY_WRITE 0
#endif
// Which parts of the floppy support are built, for bisecting a problem on the rig (default: all).
// 1 = the registers are staged in the ROM image and reads run the controller's side effects,
// 2 = memory writes to the window are captured (PIO1 SM2) and applied, 4 = core 1 runs the command engine.
// Diagnostics for the disk path on the rig: 1 = reads of the disk image fail at once (no SPI access), 2 = no
// start-up load of cylinder 0. 0 = normal.
#ifndef FLOPPY_DIAG
#define FLOPPY_DIAG 0
#endif
#ifndef FLOPPY_STAGES
#define FLOPPY_STAGES 7
#endif

// Core 0 decides whether a memory access is a read of this cartridge (it re-reads _MREQ, _SLTSL and _RD
// from the live pins) and only then puts the byte on the bus, through a drive state machine that also
// holds the data 68 ns after the access. The original RPROM scheme (-DROM_CORE_GATE=OFF) is not available
// on the cartridge pinout.
#ifndef ROM_CORE_GATE
#define ROM_CORE_GATE 1
#endif

// An MSX2 has its own V9938, whose register numbers go up to 46. A real TMS9918A only decodes 3 bits of the
// register number, so a write to R#8 and up would land on R#0-R#7 and wreck the mirrored picture: with
// MSX2_HOST those writes are ignored instead. Set with cmake -DMSX2_HOST=OFF for an MSX1.
#ifndef MSX2_HOST
#define MSX2_HOST 0
#endif

// Load the cartridge from the SD card at boot (MENU.ROM, DEFAULT.ROM, the MSX directory's first file; falls
// back to the built-in cartridge, then the flash image). Set with cmake -DSD_ROM=OFF for the firmware without the card.
#ifndef SD_ROM
#define SD_ROM 1
#endif
#define CONFIG_SD_ROM_NAME "DEFAULT ROM" /* the 8.3 name, padded with spaces, no dot */
#define CONFIG_SD_DISK_NAME "TEST    DSK" /* the floppy disk image (a 720K or 360K MSX disk) */
#define CONFIG_SD_DISKROM_NAME "DISK    ROM" /* the fixed 16K disk ROM, loaded whenever the menu picks a .DSK */
#define CONFIG_SD_MENU_NAME "MENU    ROM" /* the menu selector (kcmfs), tried before the MSX directory */
#define CONFIG_SD_MENU_DIR "MSX        " /* ROM and DSK images the menu lists and the user picks from */

// MENU.ROM (kcmfs) is a 32K cartridge at $4000, so its protocol window at $8000-$80FF is in its own page 2
#define MENU_CART_BYTES 0x4000u

// Other resources
#define CONFIG_VGA_PIO pio2
#define CONFIG_VDP_PIO pio1
#define CONFIG_ROM_PIO pio0

// Where the ROM image lives in flash: slot 1 of the RPROM layout (512K slots, slot 0 is firmware),
// which is where `picotool load image.bin -t bin -o 0x10080000` puts it.
#define CONFIG_ROM_FLASH_OFFSET (512 * 1024)

// The slot's address lines (A0-A15)
#define CONFIG_ROM_ADDRESS_BITS 16
// The cartridge buffer, rom_image: CART_MAX_BYTES (cart.h). Its own SRAM region (linker/), 256K.
#define CONFIG_ROM_IMAGE_SIZE (256u * 1024u)
