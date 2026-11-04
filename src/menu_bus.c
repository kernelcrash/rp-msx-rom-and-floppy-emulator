#include "menu_bus.h"

#include <string.h>
#include <pico/stdlib.h>
#include "board_config.h"
#include "menu_decode.h"
#include "rom.h"
#if FLOPPY
#include "disk_pick.h"
#include "fdc_bus.h"
#endif
#if BUTTONS
#include <hardware/adc.h>
#include <hardware/gpio.h>
#endif

struct menu menu_bus_state;
static uint8_t *menu_rom;

// menu.c copies a whole display name straight into one listing slot (see list_build_cb): this must
// always fit.
_Static_assert(FAT_MAX_NAME < MENU_SLOT_BYTES, "a long name must fit a menu listing slot with room for its NUL");

// Blocking, one sector at a time, from core 1: a directory listing is a handful of sectors (the
// root, for the "MSX" entry, then the directory itself), nowhere near the cost of a whole disk
// image, and core 1 only ever does this once per command, not in a hot loop. MENU requires SD_ROM
// (see CMakeLists.txt), so rom_sd_card always exists here.
static bool card_read(void *ctx, uint32_t lba, uint8_t *buf) {
    (void)ctx;
    return sd_read_sector(&rom_sd_card, lba, buf) == SD_OK;
}

#if FLOPPY || MEGAROM
// The menu window needs no mode flag of its own: main.c's write_classify_early() treats it as
// whatever is left once fdc_pio_active()/rom_bank_pio_active() have both said no (the same
// precedence core0_loop's write dispatch always used). Before refactor-pio-sm.md this function
// gave up menu_write_program's claim on the shared state machine so fdc_write_program or
// mapper_write_program could be loaded instead; generic_write is always resident now, so there is
// nothing left to release here -- kept as a no-op for the call sites that still mark this step in
// the mode-transition sequence (menu_bus.c's own perform_swap, main.c's rom_bank_pio_claim).
void menu_pio_disable(void) {
}
#endif

#if BUTTONS
static void buttons_init(void);
static int button_index_of(const char *name);
static int button_first_file(void);
static int button_pos;  // index (in directory order, files and directories) of what was last loaded from MSX; -1 none yet
#endif

void menu_glue_init(uint8_t *rom) {
    menu_rom = rom;
    // The listing is built straight into the cartridge image, where the Z80 reads it: a second 23K
    // copy in RAM (the old struct menu array) did not fit beside the V9938's VRAM.
    menu_init(&menu_bus_state, menu_rom + MENU_STAGE_BASE + MENU_WINDOW_BYTES);
#if BUTTONS
    buttons_init();
#endif
}

void menu_bus_step(uint32_t s) {
    unsigned offset;
    uint8_t data;
    if(!menu_decode_write(s, &offset, &data))
        return;
    menu_write(&menu_bus_state, offset, data);
    // Stage just the one byte that changed: this runs on core 0, so it must stay O(1). The busy
    // bit the Z80 polls after a list command lives here, which is why this cannot wait for core 1.
    menu_rom[MENU_STAGE_BASE + offset] = data;
}

// ASCII case-insensitive: `name` is whatever kcmfs echoed back from the listing, which, since long
// names are now supported, preserves whatever case the file actually has on the card (unlike the
// old 8.3-only display, which was always upper case) -- "zaxxon.dsk" and "Zaxxon.DSK" must be
// treated the same, matching fat_find_by_name's own case-insensitivity (fat.c).
static bool has_extension(const char *name, const char *ext) {
    const size_t nlen = strlen(name), elen = strlen(ext);
    if(nlen < elen)
        return false;
    const char *suffix = name + nlen - elen;
    for(size_t i = 0; i < elen; i++) {
        char a = suffix[i], b = ext[i];
        if(a >= 'a' && a <= 'z')
            a = (char)(a - ('a' - 'A'));
        if(b >= 'a' && b <= 'z')
            b = (char)(b - ('a' - 'A'));
        if(a != b)
            return false;
    }
    return true;
}

#if FLOPPY
// Where NEXT/PREV are inside a multi-disk game's directory, as the STM32's disk_index/disk_index_max: max 0 means
// the last thing loaded was not a directory. Set by perform_swap, stepped by the buttons.
static uint32_t disk_index, disk_index_max;

// Attach `name` in `dir` (MSX itself, or a multi-disk game's directory in it) as drive A.
static void swap_disk(const struct fat_dir *dir, const char *name) {
    // The disk ROM (DISK.ROM, a fixed 16K file in the root) at $4000, and the controller's registers
    // at the end of every 16K page: the cartridge image is laid out like the slot (offset = Z80
    // address) while a disk is attached. The MSX is resetting meanwhile and may run the disk ROM's
    // INIT as soon as it can see it, so the order matters:
    //  1. all the card work (where the disk image is, then DISK.ROM into image $4000-$7FFF, which is
    //     kcmfs's page 2, not the page 1 code it is still running from until the reset)
    //  2. the controller attached and its registers staged: live before anything can talk to it
    //  3. only then the banks moved, so that the disk ROM appears together with a working controller
    //  4. kcmfs's old code (image $0000-$3FFF, now the slot's page 0) blanked, around the registers
    cart_mode = CART_MODE_PLAIN;  // the menu window is done with
    uint32_t size;
    if(fdc_prepare_disk(dir, name) != FAT_OK)
        return;  // stay in kcmfs (it resets into it again)
    if(fat_load_file(card_read, NULL, CONFIG_SD_DISKROM_NAME, menu_rom + 0x4000, 0x4000, &size) != FAT_OK)
        return;
    memset(menu_rom + 0x8000, 0xff, 0x8000);
    // Page 2 (all but the registers) reads RET: for a transfer into page 1 the NMS-8250 disk ROM puts this
    // slot in page 2 and enables interrupts, and a game whose interrupt handler is in page 2 RAM (F-1 Spirit
    // 3D Special: JP 8602h) then jumps here. $FF there is RST 38h, which loops until the stack runs down
    // into the registers; RET returns at once and the interrupt is taken later, once page 2 is RAM again.
    memset(menu_rom + 0x8000, 0xc9, FDC_STAGE_BASE);
    fdc_attach_prepared(menu_rom);
    cart_set_flat(bank_register, menu_rom);
    memset(menu_rom, 0xff, FDC_STAGE_BASE);
}
#endif

// The user has picked something from the menu, or a button has: load it. A .DSK needs the disk ROM and the
// controller (fdc_mode_active takes over from the menu window, so this window goes quiet until the
// Z80 is reset into Disk BASIC and back to BASIC's own boot path -- reaching the menu again is not
// supported yet); a directory is a multi-disk game: disk `disk_index` of it, by disk_pick.h's rules, goes
// in drive A the same way; anything else is loaded as a plain 16K cartridge, and the controller, if it was
// ever running, has nothing left to talk to.
static void perform_swap(const char *chosen_name) {
    struct fat_dir dir;
    if(fat_open_subdir(card_read, NULL, CONFIG_SD_MENU_DIR, &dir) != FAT_OK)
        return;  // the directory vanished between listing and choosing: give up quietly

    // Looked up by display name (long or short, matching what menu.c listed and kcmfs echoed back
    // verbatim) -- found here to confirm it still exists and whether it is a directory; the .DSK
    // path below has fdc_prepare_disk do its own lookup the same way, rather than thread a
    // struct fat_dirent through fdc_bus.h for what is otherwise a self-contained operation there.
    struct fat_dirent e;
    if(fat_find_by_name(card_read, NULL, &dir, chosen_name, &e) != FAT_OK)
        return;

#if FLOPPY
    if(e.is_dir) {
        static struct disk_pick pick;
        const char *disk = disk_pick_dir(card_read, NULL, &e, &pick, disk_index);
        if(!disk) {
            disk_index = disk_index_max = 0;  // no disk in it: nothing loaded, NEXT goes on to the next entry
            return;
        }
        disk_index = pick.index;
        disk_index_max = pick.max;
        const struct fat_dir sub = { e.first_cluster };
        swap_disk(&sub, disk);
        return;
    }
    disk_index = disk_index_max = 0;
    if(has_extension(chosen_name, ".DSK")) {
        swap_disk(&dir, chosen_name);
        return;
    }
#endif
    // Everything else loads via rom_bank_load (rom.c): a plain, identity-mapped cartridge, or one
    // of the bank-switching types it detects by extension (Konami4/5, ASCII8/16) -- see
    // folded-megarom.md / RPROM/plan-folded-megarom.md.
    rom_bank_load(&dir, chosen_name);
}

#if BUTTONS
// NEXT and PREV, msx-rom-and-floppy-emulator's two buttons, on one ADC pin (CONFIG_PIN_BUTTONS): 10k to 3V3, NEXT
// shorts the pin to GND, PREV pulls it to GND through another 10k (half way). NEXT wins if both are pressed. Each
// press loads the next or previous entry of the MSX directory, in directory order (the order kcmfs lists them,
// the order they were copied to the card), wrapping round past the last, exactly as picking it in kcmfs would
// (perform_swap): reset the MSX for a ROM; a .DSK gets DISK.ROM and a fresh controller. Inside a multi-disk game's
// directory NEXT and PREV step its disk number first, as the STM32's button loop does: NEXT past its highest
// disk goes on to the next entry, PREV below disk 1 back to the previous one (which, if a directory, starts at
// disk 1 again). Unlike the STM32, one press loads once (no repeat while held), and PREV on the first entry
// stays on it. Core 1 only, ADC read every BUTTON_POLL_US: nothing on core 0's path.
#define BUTTON_ADC_INPUT (CONFIG_PIN_BUTTONS - 40)  // RP2350B: GPIO 40-47 are ADC inputs 0-7
#define BUTTON_POLL_US 10000
#define BUTTON_STABLE 3  // the same reading on this many polls in a row (30 ms) before it counts

enum button { BUTTON_NONE, BUTTON_PREV, BUTTON_NEXT };

static void buttons_init(void) {
    adc_init();
    adc_gpio_init(CONFIG_PIN_BUTTONS);
    // The pad's own pull-up as well, so a board with no buttons fitted reads "none" instead of floating (with the
    // 10k ladder fitted PREV then reads about 1.8 V, still mid range)
    gpio_pull_up(CONFIG_PIN_BUTTONS);
    adc_select_input(BUTTON_ADC_INPUT);
    // Booted from the MSX directory's first plain file (no MENU.ROM, no DEFAULT.ROM): NEXT goes to the entry after it
    button_pos = rom_menu_source == ROM_MENU_FROM_FIRST_FILE ? button_first_file() : -1;
}

static enum button button_classify(uint16_t v) {  // 12 bit
    if(v < 4096 / 3)
        return BUTTON_NEXT;
    if(v < 4096 * 2 / 3)
        return BUTTON_PREV;
    return BUTTON_NONE;
}

// One walk of the MSX directory, counting entries (files and directories) as the listing does: stops at index
// `want`, or, with `name` set, at the entry with exactly that display name, or, with `first_file`, at the first
// plain file (rom.c's boot fallback).
struct button_walk {
    int want;
    const char *name;
    bool first_file;
    int index;  // entries passed so far
    int found;  // the index stopped at, or -1
};
static char button_name[FAT_MAX_NAME + 1];  // the entry found by index

static bool button_walk_cb(void *vctx, const struct fat_dirent *e) {
    struct button_walk *w = vctx;
    const bool hit = w->first_file ? !e->is_dir : w->name ? strcmp(e->name, w->name) == 0 : w->index == w->want;
    if(hit) {
        w->found = w->index;
        if(!w->name)
            strcpy(button_name, e->name);
        return false;
    }
    w->index++;
    return true;
}

static int button_walk(int want, const char *name, bool first_file) {
    struct fat_dir dir;
    struct button_walk w = { want, name, first_file, 0, -1 };
    if(fat_open_subdir(card_read, NULL, CONFIG_SD_MENU_DIR, &dir) == FAT_OK)
        fat_list_dir(card_read, NULL, &dir, button_walk_cb, &w);
    return w.found;
}

static int button_index_of(const char *name) {
    return button_walk(0, name, false);
}

static int button_first_file(void) {
    return button_walk(0, NULL, true);
}

static void button_load(enum button b) {
    int want;
#if FLOPPY
    // The STM32's button loop: inside a directory (disk_index_max > 0) the disk number moves first
    want = button_pos;
    if(b == BUTTON_PREV) {
        if(disk_index_max > 0 && disk_index > 0)
            disk_index--;
        if(disk_index == 0)
            want = button_pos > 0 ? button_pos - 1 : 0;
    } else {
        if(disk_index >= 1)
            disk_index++;
        if(disk_index == 0 || disk_index > disk_index_max) {
            disk_index = disk_index_max = 0;
            want = button_pos + 1;
        }
    }
#else
    want = b == BUTTON_NEXT ? button_pos + 1 : (button_pos > 0 ? button_pos - 1 : 0);
#endif
    int found = button_walk(want, NULL, false);
    if(found < 0 && want > 0)
        found = button_walk(0, NULL, false);  // NEXT past the last entry: wrap round to the first
    if(found < 0)
        return;  // no MSX directory, or nothing in it
    button_pos = found;
#if FLOPPY
    // A disk may be attached: give the window up before perform_swap, which assumes, as after kcmfs, that the
    // controller is not running (the disk path re-initialises it, the ROM path loads over its registers). core 0
    // changes to its no-disk loop within microseconds; both paths read the card for milliseconds before they
    // touch the image.
    fdc_pio_disable();
#endif
    perform_swap(button_name);
}

static void buttons_poll(uint32_t now) {
    static uint32_t last_us;
    static enum button seen, held, pending;
    static uint8_t count;
    if((uint32_t)(now - last_us) < BUTTON_POLL_US)
        return;
    last_us = now;

    const enum button b = button_classify(adc_read());
    if(b != seen) {
        seen = b;
        count = 1;
    } else if(count < BUTTON_STABLE) {
        count++;
    }
    if(count == BUTTON_STABLE && b != held) {
        held = b;
        if(b != BUTTON_NONE)
            pending = b;  // on the press, once
    }
    if(pending == BUTTON_NONE)
        return;
#if FLOPPY
    if(fdc_pio_active() && fdc_card_busy())
        return;  // a disk sector is half way through the SPI: after it
#endif
    const enum button p = pending;
    pending = BUTTON_NONE;
    button_load(p);
}
#endif

void menu_glue_poll_core1(void) {
    // Rate limited: this is called from the render loop's idle spin, like fdc_glue_poll_core1.
    static uint32_t last_us;
    const uint32_t now = time_us_32();
    if((uint32_t)(now - last_us) < 20)
        return;
    last_us = now;

    if(menu_bus_state.list_pending) {
        // The listing goes straight into the image; then the window, whose cleared busy bit tells
        // kcmfs it is done, so it goes last. Only once per command, never on core 0's path.
        menu_run_list(&menu_bus_state, card_read, NULL);
        memcpy(menu_rom + MENU_STAGE_BASE, menu_bus_state.window, sizeof menu_bus_state.window);
    } else if(menu_bus_state.load_pending) {
        // menu_take_load's pointer is only valid until the next call that touches the window, so
        // the name is copied out before anything else runs
        char chosen[MENU_WINDOW_BYTES];
        strncpy(chosen, menu_take_load(&menu_bus_state), sizeof chosen - 1);
        chosen[sizeof chosen - 1] = 0;
#if BUTTONS
        // NEXT/PREV carry on from the file picked, as on the STM32 (it finds the pick in the directory first)
        button_pos = button_index_of(chosen);
#endif
#if FLOPPY
        disk_index = 1;  // as the STM32's load command: a directory starts at its disk 1
#endif
        perform_swap(chosen);
    }
#if BUTTONS
    else
        buttons_poll(now);
#endif
}
