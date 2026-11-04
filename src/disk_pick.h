#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "fat.h"

// A subdirectory of MSX is one multi-disk game: which of its .dsk files goes into drive A. Ported from
// msx-rom-and-floppy-emulator's util.c (find_disk_n, and load_disks' directory branch) with its rules unchanged:
//
//  * "Disk n" in a filename is "disk", case-insensitive, then n, optionally after one space ("Disk 2",
//    "disk2"), n one digit. Only the first "disk" in the name counts: if its number does not match, the name
//    does not match, so "Game (Disk 4 of 4)(Game Disk 1).dsk" is disk 4 only.
//  * Only names ending in .dsk (case-insensitive) count; .dsk2 (the STM32's drive B) does not, and there is no
//    drive B here.
//  * `index` in: the disk wanted. 0 or 1 also allow a fallback: the last .dsk seen in the directory (the
//    STM32 copies each one over the last), and index becomes 1 as soon as a .dsk is seen.
//  * Disk `index` if there is one (the last match, again); otherwise disk index+1, and index goes up by one
//    (for sets kept without their disk 1); otherwise the fallback; otherwise nothing.
//  * `max` out: the highest n any .dsk has, or 1 if none has one.

struct disk_pick {
    uint32_t index, max;
    char want[FAT_MAX_NAME + 1], next[FAT_MAX_NAME + 1], fallback[FAT_MAX_NAME + 1];
};

void disk_pick_begin(struct disk_pick *p, uint32_t index);
void disk_pick_add(struct disk_pick *p, const char *name);  // each plain file of the directory, in order
const char *disk_pick_end(struct disk_pick *p);             // the name picked (in `p`), or NULL; sets index, max

// All three over the directory `dir_entry` (a directory entry of MSX, from fat_find_by_name or fat_list_dir).
// NULL if it has no disk to pick or cannot be read.
const char *disk_pick_dir(fat_read_sector_fn read, void *ctx, const struct fat_dirent *dir_entry,
    struct disk_pick *p, uint32_t index);
