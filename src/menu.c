// The menu protocol model. See menu.h for what it replicates and why.
#include "menu.h"

#include <string.h>

void menu_init(struct menu *m, uint8_t *listing) {
    memset(m, 0, sizeof *m);
    m->listing = listing;  // not cleared until a listing is made: in the firmware it is the image itself
}

void menu_write(struct menu *m, unsigned offset, uint8_t value) {
    if(offset >= MENU_WINDOW_BYTES)
        return;
    m->window[offset] = value;
    if(offset != 0)
        return;
    // Matches write_slot_memory in interrupt.S: LIST is checked first, LOAD only if LIST was not
    // set. kcmfs only ever writes exactly 0x80 or 0x40, never both at once, so the order never
    // actually matters in practice; it is kept anyway to be exactly what was read from the source.
    if(value & MENU_CMD_LIST)
        m->list_pending = true;
    else if(value & MENU_CMD_LOAD)
        m->load_pending = true;
}

uint8_t menu_read(const struct menu *m, unsigned offset) {
    if(offset < MENU_WINDOW_BYTES)
        return m->window[offset];
    const unsigned i = offset - MENU_WINDOW_BYTES;
    return i < MENU_LISTING_BYTES ? m->listing[i] : 0xff;
}

struct list_build_ctx {
    struct menu *m;
    uint32_t count;
};

static bool list_build_cb(void *vctx, const struct fat_dirent *e) {
    struct list_build_ctx *ctx = vctx;
    if(ctx->count >= MENU_MAX_FILES)
        return false;  // kcmfs cannot address any more pages than this

    // e->name is fat.c's own display name: the long name if the entry had one, "NAME.EXT" from the
    // 8.3 name otherwise -- already NUL terminated and already capped at FAT_MAX_NAME, which by
    // construction (see the _Static_assert in menu_bus.c) always fits one MENU_SLOT_BYTES slot.
    uint8_t *slot = ctx->m->listing + ctx->count * MENU_SLOT_BYTES;
    memset(slot, 0, MENU_SLOT_BYTES);
    const size_t len = strlen(e->name);
    memcpy(slot, e->name, len < MENU_SLOT_BYTES ? len : MENU_SLOT_BYTES - 1);
    ctx->count++;
    return true;
}

void menu_run_list(struct menu *m, fat_read_sector_fn read, void *ctx) {
    struct list_build_ctx lctx = { m, 0 };
    memset(m->listing, 0, MENU_LISTING_BYTES);  // slots past the new count read as zero

    struct fat_dir dir;
    if(fat_open_subdir(read, ctx, "MSX        ", &dir) == FAT_OK)
        fat_list_dir(read, ctx, &dir, list_build_cb, &lctx);
    // A missing directory, or a card error partway through, just leaves lctx.count at whatever it
    // reached (0 if nothing was listed): reported as an empty or partial listing, not a hang.

    m->file_count = lctx.count;
    m->window[2] = (uint8_t)(lctx.count > 255 ? 255 : lctx.count);
    m->window[3] = 0;              // so $8002/$8003 can be read as a 16 bit count, as kcmfs allows
    m->window[0] = 0x00;           // clear the busy bit kcmfs polls (mirrors the STM32's *p = 0x00)
    m->list_pending = false;
}

const char *menu_take_load(struct menu *m) {
    m->load_pending = false;
    // kcmfs always writes a NUL within the 127 bytes available (either the source string's own, or
    // a forced one after exactly 127 bytes); this is just a guard against a window driven directly
    // by a test rather than by kcmfs's own copy loop.
    m->window[MENU_WINDOW_BYTES - 1] = 0;
    return (const char *)&m->window[MENU_FNAME_OFFSET];
}
