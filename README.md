# rp-msx-rom-and-floppy-emulator

An MSX cartridge built around an RP2350B (WeAct Studio RP2350B core board) and an SD card. It is the RP2350
version of [msx-rom-and-floppy-emulator](https://github.com/kernelcrash/msx-rom-and-floppy-emulator).

- **ROM cartridges** from the SD card: plain 16K/32K/64K ROMs, and Konami4, Konami5 (no SCC sound), ASCII8 and
  ASCII16 mapper ROMs up to 256K.
- **Floppy disk images** (720K or 360K `.DSK`) as drive A, through an emulated WD2793 controller and a disk ROM.
  Disks are read only.
- **kcmfs menu**: an MSX program that lists the SD card's `MSX` directory and lets you pick a ROM or disk.
- **NEXT / PREV buttons** to step through the files (and through the disks of a multi-disk game).
- **VGA output**: a copy of the MSX's video on a VGA monitor. The MSX's own VDP keeps working as normal; the
  RP2350 only listens to what is written to it, so some screens may not look right on the VGA output.

Tested on an Omega MSX2 and a Panasonic FS-A1.

## Parts

- WeAct Studio RP2350B core board
- Something to plug into the MSX cartridge slot (a cartridge PCB or edge connector)
- SD card socket (3.3 V, SPI)
- VGA connector and 11 resistors (see below)
- Two push buttons and two 10k resistors (optional)

## Wiring

All signals connect directly, with no level shifters.

**Power the RP2350B before the MSX is switched on.** Its GPIO 0-39 tolerate 5 V only while the board's 3.3 V
is up. GPIO 40-47 are not 5 V tolerant at all: only the 3.3 V VGA, SD card and button signals go there.

### Cartridge slot

| GPIO | Signal | Slot pin |
|---|---|---|
| 1-8 | D0-D7 | 34, 33, 36, 35, 38, 37, 40, 39 |
| 9 | /MREQ | 12 |
| 10 | /SLTSL | 4 |
| 11 | /IORQ | 11 |
| 12 | /WR | 13 |
| 13 | /RD | 14 |
| 16-31 | A0-A15 | 28, 27, 30, 29, 32, 31, 22, 21, 24, 17, 20, 19, 23, 26, 25, 18 |
| GND | GND | 41, 43 |

GPIO 0, 14 and 15 are not used. The board's LED is on GPIO 25 (A9), so it flickers while the MSX runs.

MSX cartridge connector:

```
 1 /CS1     2 /CS2     3 /CS12    4 /SLTSL   5 -        6 /RFSH    7 /WAIT    8 /INT
 9 /M1     10 /BUSDIR 11 /IORQ   12 /MREQ   13 /WR     14 /RD     15 /RESET  16 -
17 A9      18 A15     19 A11     20 A10     21 A7      22 A6      23 A12     24 A8
25 A14     26 A13     27 A1      28 A0      29 A3      30 A2      31 A5      32 A4
33 D1      34 D0      35 D3      36 D2      37 D5      38 D4      39 D7      40 D6
41 GND     42 CLOCK   43 GND     44 SW1     45 +5V     46 SW2     47 +5V     48 +12V
49 SOUNDIN 50 -12V
```

### VGA

Each colour is three resistors joined at the VGA connector pin.

| GPIO | Signal | Resistor | VGA pin |
|---|---|---|---|
| 32 / 33 / 34 | Blue bit 0 / 1 / 2 | 2k / 1k / 500 ohm | 3 |
| 35 / 36 / 37 | Green bit 0 / 1 / 2 | 2k / 1k / 500 ohm | 2 |
| 38 / 39 / 40 | Red bit 0 / 1 / 2 | 2k / 1k / 500 ohm | 1 |
| 41 | HSYNC | 47 ohm | 13 |
| 42 | VSYNC | 47 ohm | 14 |
| GND | GND | | 5-8, 10 |

### SD card

| GPIO | SD card |
|---|---|
| 44 | DAT0 (MISO) |
| 45 | DAT3 / CS |
| 46 | CLK |
| 47 | CMD (MOSI) |
| 3V3 / GND | VDD / VSS |

10k pull-ups to 3.3 V on DAT0, CMD and CS help on longer wires.

### Buttons

Both buttons share GPIO 43:

- 10k from GPIO 43 to 3.3 V
- NEXT: from GPIO 43 straight to GND
- PREV: from GPIO 43 to GND through 10k

## SD card

FAT16 or FAT32.

```
MENU.ROM          kcmfs (kcmfs/menu.rom from this project)
DISK.ROM          a 16K MSX disk ROM (tested with the Philips NMS 8250's)
MSX/              the ROMs and disks the menu lists
MSX/game.rom
MSX/game.konami4  a mapper can be named by the extension: .konami4 .konami5 .ascii8 .ascii16
MSX/game.dsk      (otherwise the mapper is guessed from the ROM)
MSX/Some Game/    a directory: a multi-disk game
```

A directory in `MSX` holds the disks of one game. Picking it loads the disk with "Disk 1" in its name
(or "Disk1"); NEXT and PREV then change to the next or previous disk.

Without `MENU.ROM`, the cartridge starts with `DEFAULT.ROM` from the root if there is one, otherwise the first
file in `MSX`.

## Building

You need the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk), CMake and the
`arm-none-eabi` GCC toolchain.

```
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S . -B build
cmake --build build -j8
```

The firmware is `build/rp_msx_rom_and_floppy_emulator.uf2`.

## Uploading the firmware

Hold the BOOT button on the WeAct board while plugging it into USB (or while pressing its reset button). It
appears as a USB drive: copy the `.uf2` file onto it. Or, with
[picotool](https://github.com/raspberrypi/picotool):

```
picotool load -x build/rp_msx_rom_and_floppy_emulator.uf2
```

## Using it

1. Power the RP2350B board, then switch on the MSX.
2. The kcmfs menu appears. Pick a ROM, a disk or a directory; the MSX restarts into it.
3. NEXT / PREV load the next or previous file in `MSX` (or the next or previous disk in a directory).
4. To get back to the menu, power-cycle the RP2350B board.

Disks need Disk BASIC to start, so a machine that boots into its own menu needs to get to BASIC first. On a
Panasonic FS-A1 hold DEL while it starts up to skip its menu.

kcmfs can be rebuilt from `kcmfs/` (see `kcmfs/README-kcmfs.md`).

## Credits and licences

- [msx-rom-and-floppy-emulator](https://github.com/kernelcrash/msx-rom-and-floppy-emulator) by kernelcrash
  (MIT): the STM32 original, kcmfs, and the WD2793 emulation (after fMSX's WD1793.c by Marat Fayzullin).
- [AppleII-VGA](https://github.com/markadev/AppleII-VGA) by Mark Aikens and David Kuder (MIT, see
  `LICENSE-AppleII-VGA`): the VGA output and VDP rendering.
- [RPROM](https://github.com/niklasekstrom/RPROM) by Niklas Ekström and Per Bengtsson (free for
  non-commercial use, see its README): the starting point for the RP2350 ROM emulation.
