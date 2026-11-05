# rp-msx-rom-and-floppy-emulator

Yet another AI experiment, or excessive 'clauding'.

An MSX cartridge built around an RP2350B (WeAct Studio RP2350B core board) and an SD card. It is the RP2350B
version of [msx-rom-and-floppy-emulator](https://github.com/kernelcrash/msx-rom-and-floppy-emulator).

- **ROM cartridges** from the SD card: plain 16K/32K/64K ROMs, and Konami4, Konami5 (no SCC sound), ASCII8 and
  ASCII16 mapper ROMs up to 256K. Generally you have to rename cartridge dumps to include the type as a suffix. 
  eg. usas.konami4. 
- **Floppy disk images** (720K or 360K `.DSK`) as drive A, through an emulated WD2793 controller and a disk ROM.
  Writes (SAVE, copying files in MSX-DOS) go back to the image on the SD card.
- **kcmfs menu**: an MSX program that lists the SD card's `MSX` directory and lets you pick a ROM or disk. This is 
  the same as the version in msx-rom-and-floppy-emulator
- **NEXT / PREV buttons** to step through the files (and through the disks of a multi-disk game).
- **VGA output**: a copy of the MSX's video on a VGA monitor. The MSX's own VDP keeps working as normal; the
  RP2350B only listens to what is written to it, so some screens may not look right on the VGA output. It tries to
  emulate the V9938 so 9918 and V9938 stuff should work, though there are limitations for the V9938 emulation

Tested on an Omega MSX2 and a Panasonic FS-A1.

## Parts

- WeAct Studio RP2350B core board. Make sure to get an A4 stepping or later board.
- Something to plug into the MSX cartridge slot (a cartridge PCB or edge connector)
- SD card socket (3.3 V, SPI)
- VGA connector and 11 resistors (see below)
- Two push buttons and two 10k resistors (optional)

## Wiring

All signals connect directly, with no level shifters.

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

MSX cartridge connector (as on the [MSX wiki](https://www.msx.org/wiki/MSX_Cartridge_slot)), with the RP2350B GPIO
in brackets:

```
       /CS2  2 |  1 /CS1
 /SLTSL(10)  4 |  3 /CS12
      /RFSH  6 |  5 -
       /INT  8 |  7 /WAIT
    /BUSDIR 10 |  9 /M1
   /MREQ(9) 12 | 11 /IORQ(11)
    /RD(13) 14 | 13 /WR(12)
          - 16 | 15 /RESET
    A15(31) 18 | 17 A9(25)
    A10(26) 20 | 19 A11(27)
     A6(22) 22 | 21 A7(23)
     A8(24) 24 | 23 A12(28)
    A13(29) 26 | 25 A14(30)
     A0(16) 28 | 27 A1(17)
     A2(18) 30 | 29 A3(19)
     A4(20) 32 | 31 A5(21)
      D0(1) 34 | 33 D1(2)
      D2(3) 36 | 35 D3(4)
      D4(5) 38 | 37 D5(6)
      D6(7) 40 | 39 D7(8)
      CLOCK 42 | 41 GND
        SW1 44 | 43 GND
        SW2 46 | 45 +5V
       +12V 48 | 47 +5V
       -12V 50 | 49 SOUNDIN
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
DISK.ROM          a 16K MSX disk ROM (tested with the Philips NMS 8250's with MD5 0ed6dbd654da55b56dfb331dd3df82f0)
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

Changed sectors are written back to the card a moment after the MSX stops writing. Use a good quality SD
card: a write the card rejects is not reported on the MSX.

Disks need Disk BASIC to start, so a machine that boots into its own menu needs to get to BASIC first. On a
Panasonic FS-A1 hold DEL while it starts up to skip its menu. ie. when you pick something in kcmfs, and you 
see the system rebooting, hold down the DEL key for a few seconds then release it.

kcmfs can be rebuilt from `kcmfs/` (see `kcmfs/README-kcmfs.md`).

## Credits and licences

- [msx-rom-and-floppy-emulator](https://github.com/kernelcrash/msx-rom-and-floppy-emulator) by kernelcrash
  (MIT): the STM32 original, kcmfs, and the WD2793 emulation (after fMSX's WD1793.c by Marat Fayzullin).
- [AppleII-VGA](https://github.com/markadev/AppleII-VGA) by Mark Aikens and David Kuder (MIT, see
  `LICENSE-AppleII-VGA`): the VGA output and VDP rendering.
- [RPROM](https://github.com/niklasekstrom/RPROM) by Niklas Ekström and Per Bengtsson (free for
  non-commercial use, see its README): the starting point for the RP2350 ROM emulation.
