#include <string.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/structs/busctrl.h>
#include <pico/stdlib.h>
#include "board_config.h"
#include "rom.h"
#include "rom.pio.h"
#include "sd_hal_pico.h"


// The PIO programs are written in terms of these, so they must agree with board_config.h
_Static_assert(ROM_RD_GPIO == CONFIG_PIN_RD, "rom.pio _RD pin does not match board_config.h");
_Static_assert(ROM_ADDRESS_BITS == CONFIG_ROM_ADDRESS_BITS, "rom.pio address width does not match board_config.h");
_Static_assert(CONFIG_PIN_ADDR_BASE + CONFIG_ROM_ADDRESS_BITS == 32,
    "BUS_ADDRESS assumes A15 is the top bit of the 32 bit snapshot");
_Static_assert(CONFIG_PIN_BUS_LAST < 32, "every bus pin must be inside the PIOs' GPIO 0-31 window");
#if !ROM_CORE_GATE
#error "-DROM_CORE_GATE=OFF (the original RPROM rom_direction scheme) needs _CE, which the cartridge pinout does not have"
#endif

_Static_assert(CONFIG_ROM_IMAGE_SIZE == CART_MAX_BYTES, "rom_image is the cartridge buffer");

#if CART_EMBEDDED
// The cartridge image built into the firmware (cart_rom.S, from cmake -DCART_ROM=<file>)
extern const uint8_t cart_rom[], cart_rom_end[];
#endif

// Our own word copy, so the ROM path does not depend on libc memcpy. GCC would otherwise
// recognise the loop and turn it back into a memcpy() call. Both pointers must be word aligned.
static void __attribute__((optimize("no-tree-loop-distribute-patterns")))
__not_in_flash_func(copy_words)(uint32_t *__restrict dst, const uint32_t *__restrict src, uint count) {
    while(count--)
        *dst++ = *src++;
}


volatile enum rom_source rom_source;
volatile enum sd_result rom_sd_result;
volatile enum fat_result rom_fat_result;
volatile uint32_t rom_sd_file_size;


#if SD_ROM
static bool sd_read_cb(void *ctx, uint32_t lba, uint8_t *buf) {
    return sd_read_sector(ctx, lba, buf) == SD_OK;
}

#if FLOPPY || MENU
// Both the floppy support and the menu support keep the card open after boot, for runtime access
// (the floppy's disk image, or the menu's directory listing).
struct sd_card rom_sd_card;
static volatile bool rom_sd_card_open;

void rom_sd_clock_changed() {
    if(rom_sd_card_open)
        rom_sd_card.hal->set_speed(rom_sd_card.hal->ctx, CONFIG_SD_SPI_HZ);
}
#endif

#if FLOPPY
struct fat_file rom_disk_file;
volatile bool rom_disk_ok;
volatile enum fat_result rom_disk_result = FAT_ERR_NOT_FOUND;
#if !MENU
static struct fat_extent rom_disk_extents[1440];  // a fully scattered 720K file on 512 byte clusters
#endif
#endif

#if MENU
// The first plain file (skipping subdirectories, as the menu's own listing does) found in the MSX
// directory, for the boot fallback below.
struct first_entry_ctx {
    struct fat_dirent found;
    bool have;
};
static bool first_entry_cb(void *ctx, const struct fat_dirent *e) {
    struct first_entry_ctx *c = ctx;
    if(e->is_dir)
        return true;  // keep looking
    c->found = *e;
    c->have = true;
    return false;  // stop: this is the one
}

volatile enum rom_menu_source rom_menu_source;

bool rom_bank_load(const struct fat_dir *dir, const char *name) {
    struct fat_dirent e;
    if(fat_find_by_name(sd_read_cb, &rom_sd_card, dir, name, &e) != FAT_OK || e.is_dir)
        return false;
    if(e.size == 0 || e.size > CART_MAX_BYTES)
        return false;
    // From here on the old cartridge (normally kcmfs) is being overwritten: stop decoding writes for it
    cart_mode = CART_MODE_PLAIN;
    if(fat_load_dirent(sd_read_cb, &rom_sd_card, &e, (uint8_t *)rom_image, CART_MAX_BYTES) != FAT_OK)
        return false;
    rom_cart_setup(e.size, rom_cart_type(name, (const uint8_t *)rom_image, e.size));
    return true;
}
#endif

// Read the cartridge from the card into rom_image and set it up: MENU.ROM (with MENU), else DEFAULT.ROM,
// else (with MENU) the first file in the MSX directory. On any failure rom_image may be part written:
// the caller fills it some other way.
static bool load_from_sd() {
    const struct sd_hal *hal = sd_hal_pico_init();

#if MENU
    struct sd_card *card = &rom_sd_card;
#else
    struct sd_card card_storage, *card = &card_storage;
#endif
    rom_sd_result = sd_init(card, hal, CONFIG_SD_SPI_HZ);

    uint8_t *const image = (uint8_t *)rom_image;
    bool ok = false;
    if(rom_sd_result == SD_OK) {
#if MENU
        uint32_t size;
        if(fat_load_file(sd_read_cb, card, CONFIG_SD_MENU_NAME, image, CART_MAX_BYTES, &size) == FAT_OK) {
            rom_cart_setup(size, MEGAROM_GENERIC);
            cart_mode = CART_MODE_MENU;
            rom_menu_source = ROM_MENU_FROM_MENUROM;
            ok = true;
        }
#endif
        if(!ok) {
            rom_fat_result = fat_load_file(sd_read_cb, card, CONFIG_SD_ROM_NAME, image, CART_MAX_BYTES,
                (uint32_t *)&rom_sd_file_size);
            if(rom_fat_result == FAT_OK) {
                rom_cart_setup(rom_sd_file_size, rom_cart_type("DEFAULT.ROM", image, rom_sd_file_size));
                ok = true;
            }
        }
#if MENU
        if(!ok) {
            struct fat_dir dir;
            struct first_entry_ctx fctx = { .have = false };
            if(fat_open_subdir(sd_read_cb, card, CONFIG_SD_MENU_DIR, &dir) == FAT_OK &&
               fat_list_dir(sd_read_cb, card, &dir, first_entry_cb, &fctx) == FAT_OK && fctx.have &&
               rom_bank_load(&dir, fctx.found.name)) {
                rom_menu_source = ROM_MENU_FROM_FIRST_FILE;
                ok = true;
            }
        }
#endif
    }

#if MENU
    // The menu keeps the card: its directory listing and the chosen file are read at run time
    if(rom_sd_result == SD_OK)
        rom_sd_card_open = true;
    else
        sd_hal_pico_deinit();
#else
    // The card is not used again: stop the SPI block and give the pins back
    sd_hal_pico_deinit();
#endif
    return ok;
}
#endif


// ---- the cartridge in the slot

#if MEGAROM
// ASCII case-insensitive suffix match: picks the mapper by file extension, as
// msx-rom-and-floppy-emulator does.
static bool has_suffix_ci(const char *name, const char *suffix) {
    const size_t nlen = strlen(name), slen = strlen(suffix);
    if(nlen < slen)
        return false;
    const char *p = name + nlen - slen;
    for(size_t i = 0; i < slen; i++) {
        char a = p[i], b = suffix[i];
        if(a >= 'a' && a <= 'z')
            a = (char)(a - ('a' - 'A'));
        if(b >= 'a' && b <= 'z')
            b = (char)(b - ('a' - 'A'));
        if(a != b)
            return false;
    }
    return true;
}
#endif

enum megarom_type rom_cart_type(const char *name, const uint8_t *rom, uint32_t size) {
#if MEGAROM
    if(has_suffix_ci(name, ".KONAMI4"))
        return MEGAROM_KONAMI4;
    if(has_suffix_ci(name, ".KONAMI5"))
        return MEGAROM_KONAMI5;
    if(has_suffix_ci(name, ".ASCII8"))
        return MEGAROM_ASCII8;
    if(has_suffix_ci(name, ".ASCII16"))
        return MEGAROM_ASCII16;
    return cart_guess_type(rom, size);
#else
    (void)name;
    (void)rom;
    (void)size;
    return MEGAROM_GENERIC;
#endif
}

void rom_cart_setup(uint32_t size, enum megarom_type type) {
#if !MEGAROM
    type = MEGAROM_GENERIC;
#endif
    cart_mode = CART_MODE_PLAIN;  // no writes are decoded while the banks change
    megarom_type = type;
    cart_mask = cart_bank_mask(size);
    cart_set_banks(bank_register, (const uint8_t *)rom_image, size, type, cart_ff);
    cart_mode = type == MEGAROM_GENERIC ? CART_MODE_PLAIN : CART_MODE_MAPPER;
}


uint8_t rom_status_colour() {
    if(rom_source == ROM_FROM_SD)
        return 12;

    switch(rom_sd_result) {
        case SD_OK: break;  // the card is fine, so it was the file: see below
        case SD_ERR_NO_CARD: return 6;
        case SD_ERR_BUS_LOW: return 14;
        case SD_ERR_INIT_TIMEOUT: return 10;
        case SD_ERR_UNSUPPORTED: return 13;
        case SD_ERR_CMD: return 4;
        default: return 7;  // reading failed: read timeout, error token, out of range
    }

    // A sector read that failed while reading the file is a card problem too, not a file problem
    return rom_fat_result == FAT_ERR_IO ? 7 : 15;
}


#if SD_ROM && SD_REPORT
// A tiny cartridge that says on the MSX's own screen why DEFAULT.ROM could not be loaded, for a rig
// with no VGA monitor to show the status colour. Its INIT prints a zero-terminated message with the
// BIOS's CHPUT and then stops there.
static const uint8_t report_code[] = {
    0x21, 0x40, 0x40,  // 4010  ld hl,4040h   the message
    0x7e,              // 4013  ld a,(hl)
    0xb7,              // 4014  or a
    0x28, 0x06,        // 4015  jr z,401Dh
    0xcd, 0xa2, 0x00,  // 4017  call 00A2h    CHPUT
    0x23,              // 401A  inc hl
    0x18, 0xf6,        // 401B  jr 4013h
    0x18, 0xfe,        // 401D  jr 401Dh      stay here
};

static char *report_text(char *p, const char *s) {
    while(*s)
        *p++ = *s++;
    return p;
}

static char *report_number(char *p, uint32_t n) {
    char digits[10];
    int i = 0;
    do {
        digits[i++] = (char)('0' + n % 10);
        n /= 10;
    } while(n);
    while(i)
        *p++ = digits[--i];
    return p;
}

static void load_sd_report(void) {
    uint8_t *const cart = (uint8_t *)rom_image;
    memset(cart, 0xff, CART_MAX_BYTES);
    cart[0] = 'A';
    cart[1] = 'B';
    cart[2] = 0x10;  // INIT = 4010h
    cart[3] = 0x40;
    memset(cart + 4, 0, 12);
    memcpy(cart + 0x10, report_code, sizeof report_code);

    char *p = (char *)cart + 0x40;
    p = report_text(p, "\r\nMSX-VDP-ROM: nothing loaded from SD\r\nSD card: ");
    p = report_text(p, sd_result_name(rom_sd_result));
    if(rom_sd_result == SD_OK) {
        p = report_text(p, "\r\nDEFAULT.ROM: ");
        p = report_text(p, fat_result_name(rom_fat_result));
        if(rom_fat_result == FAT_ERR_TOO_BIG) {
            p = report_text(p, ", ");
            p = report_number(p, rom_sd_file_size);
            p = report_text(p, " bytes");
        }
    }
    p = report_text(p, "\r\n");
    *p = 0;
    rom_cart_setup(0x4000, MEGAROM_GENERIC);  // at $4000: its INIT is $4010
}
#endif

enum rom_source rom_load() {
    rom_source = ROM_FROM_NONE;
    memset(cart_ff, 0xff, sizeof cart_ff);
    rom_cart_setup(0, MEGAROM_GENERIC);  // an empty slot until something is loaded

#if SD_ROM
    if(load_from_sd())
        rom_source = ROM_FROM_SD;
#if SD_REPORT
    else {
        load_sd_report();
        rom_source = ROM_FROM_SD_REPORT;
    }
#endif
#endif

#if CART_EMBEDDED
    if(rom_source == ROM_FROM_NONE) {
        // The embedded cartridge (its file extension can name the mapper, as on the card)
        const uint32_t size = (uint32_t)(cart_rom_end - cart_rom);
        memset(rom_image, 0xff, CART_MAX_BYTES);
        memcpy(rom_image, cart_rom, size);
        rom_cart_setup(size, rom_cart_type(CART_ROM_EXT, (const uint8_t *)rom_image, size));
        rom_source = ROM_FROM_EMBEDDED;
    }
#endif

    if(rom_source == ROM_FROM_NONE) {
        // The image stored in flash: a whole 64K slot image, $0000-$FFFF. Also overwrites anything a
        // failed SD load left behind.
        copy_words(rom_image, (const uint32_t *)(XIP_BASE + CONFIG_ROM_FLASH_OFFSET), 0x10000 / 4);
        memset((uint8_t *)rom_image + 0x10000, 0xff, CART_MAX_BYTES - 0x10000);
        rom_cart_setup(0x10000, MEGAROM_GENERIC);
        rom_source = ROM_FROM_FLASH;
    }

    return rom_source;
}


void rom_init() {
    PIO pio = CONFIG_ROM_PIO;

    // Give every pin we read a function so the pad is not isolated (plain SIO input), then hand
    // D0-D7 to this PIO. The pins stay tri-state until generic_read_drive enables them.
    for(uint pin = CONFIG_PIN_BUS_FIRST; pin <= CONFIG_PIN_BUS_LAST; pin++) {
        gpio_init(pin);
    }
    for(uint pin = CONFIG_PIN_DATA_BASE; pin < CONFIG_PIN_DATA_BASE + 8; pin++) {
        gpio_set_function(pin, GPIO_FUNC_PIO0);
    }

    // The bus pins are all inside the GPIO 0-31 window, which is this PIO's default
    hard_assert(pio_get_gpio_base(pio) == 0);

    // Direction state machine: core 0 decides and sends a token (see core0_loop in main.c), for both
    // ROM and VDP reads uniformly (generic_read_drive gates on _RD, the signal common to both -- see
    // rom.pio).
    uint direction_offset = pio_add_program(pio, &generic_read_drive_program);
    pio_sm_claim(pio, ROM_DIRECTION_SM);
    pio_sm_config c = generic_read_drive_program_get_default_config(direction_offset);
    sm_config_set_jmp_pin(&c, CONFIG_PIN_RD);
    sm_config_set_out_pins(&c, CONFIG_PIN_DATA_BASE, 8);
    pio_sm_init(pio, ROM_DIRECTION_SM, direction_offset, &c);
    pio_sm_set_enabled(pio, ROM_DIRECTION_SM, true);

    // Data state machine
    uint data_offset = pio_add_program(pio, &rom_data_program);
    pio_sm_claim(pio, ROM_DATA_SM);
    c = rom_data_program_get_default_config(data_offset);
    sm_config_set_out_pins(&c, CONFIG_PIN_DATA_BASE, 8);
    sm_config_set_out_shift(&c, true, true, 8);  // shift right, autopull every byte
    pio_sm_init(pio, ROM_DATA_SM, data_offset, &c);
    pio_sm_set_enabled(pio, ROM_DATA_SM, true);

    // Give this core priority on the bus so the DMA engines and the other core cannot delay a
    // ROM lookup.
    busctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC0_BITS;

    // Read state machine last: from here on every Z80 read cycle (memory or I/O) is captured and
    // pushed, unconditionally -- core 0 classifies each one (see main.c's read_classify()).
    uint read_offset = pio_add_program(pio, &generic_read_program);
    pio_sm_claim(pio, GENERIC_READ_SM);
    c = generic_read_program_get_default_config(read_offset);
    sm_config_set_in_pins(&c, 0);  // GPIO n = bit n of the pushed word
    sm_config_set_in_shift(&c, false, true, 32);  // shift left, autopush every 32 bits
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);  // 8 deep: more headroom for the new volume
    pio_sm_init(pio, GENERIC_READ_SM, read_offset, &c);
    pio_sm_set_enabled(pio, GENERIC_READ_SM, true);
}
