// Picking a disk in a multi-disk game's directory. See disk_pick.h for the rules (msx-rom-and-floppy-emulator's).
#include "disk_pick.h"

#include <string.h>

static char lower(char c) {
    return (c >= 'A' && c <= 'Z') ? (char)(c + ('a' - 'A')) : c;
}

// util.c's suffix_match: the name must be longer than the suffix
static bool suffix_match(const char *name, const char *suffix) {
    const size_t nlen = strlen(name), slen = strlen(suffix);
    if(nlen <= slen)
        return false;
    for(size_t i = 0; i < slen; i++)
        if(lower(name[nlen - slen + i]) != lower(suffix[i]))
            return false;
    return true;
}

// util.c's find_disk_n
static bool find_disk_n(const char *p, uint32_t n) {
    for(; *p; p++) {
        if(lower(p[0]) == 'd' && lower(p[1]) == 'i' && lower(p[2]) == 's' && lower(p[3]) == 'k')
            return (p[4] == ' ' && p[5] == (char)(n + '0')) || p[4] == (char)(n + '0');
    }
    return false;
}

void disk_pick_begin(struct disk_pick *p, uint32_t index) {
    p->index = index;
    p->max = 0;
    p->want[0] = p->next[0] = p->fallback[0] = 0;
}

static void copy_name(char *dst, const char *src) {
    strncpy(dst, src, FAT_MAX_NAME);
    dst[FAT_MAX_NAME] = 0;
}

void disk_pick_add(struct disk_pick *p, const char *name) {
    if(!suffix_match(name, ".dsk"))
        return;
    if(p->index == 1 || p->index == 0) {
        p->index = 1;
        copy_name(p->fallback, name);
    }
    for(uint32_t i = 9; i > 0; i--) {
        if(find_disk_n(name, i) && i > p->max)
            p->max = i;
    }
    if(find_disk_n(name, p->index))
        copy_name(p->want, name);
    if(find_disk_n(name, p->index + 1))
        copy_name(p->next, name);
}

const char *disk_pick_end(struct disk_pick *p) {
    if(p->max == 0)
        p->max = 1;
    if(p->want[0])
        return p->want;
    if(p->next[0]) {
        p->index++;
        return p->next;
    }
    if(p->fallback[0])
        return p->fallback;
    return NULL;
}

static bool pick_cb(void *ctx, const struct fat_dirent *e) {
    if(!e->is_dir)
        disk_pick_add(ctx, e->name);
    return true;
}

const char *disk_pick_dir(fat_read_sector_fn read, void *ctx, const struct fat_dirent *dir_entry,
    struct disk_pick *p, uint32_t index) {
    disk_pick_begin(p, index);
    const struct fat_dir dir = { dir_entry->first_cluster };
    if(!dir_entry->is_dir || dir.cluster < 2 || fat_list_dir(read, ctx, &dir, pick_cb, p) != FAT_OK)
        return NULL;
    return disk_pick_end(p);
}
