#include "fat.h"

#include <string.h>

#define SECTOR_SIZE 512

static inline uint16_t le16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}


struct volume {
    fat_read_sector_fn read;
    void *ctx;

    bool fat32;
    uint8_t sec_per_clus;
    uint32_t fat_lba;        // first sector of the first FAT
    uint32_t fat_secs;       // sectors in one FAT
    uint32_t root_lba;       // FAT16: first sector of the fixed root directory
    uint32_t root_secs;      // FAT16: its length in sectors
    uint32_t data_lba;       // first sector of cluster 2
    uint32_t clusters;       // number of data clusters (valid cluster numbers are 2 .. clusters + 1)
    uint32_t root_cluster;   // FAT32: first cluster of the root directory

    bool fat_cache_valid;
    uint32_t fat_cache_lba;
    uint8_t fat_cache[SECTOR_SIZE];
};

// Static so the callers (which run early at boot on a small stack) do not need ~1.5 KB of stack
static struct volume vol;
static uint8_t sector[SECTOR_SIZE];
static uint8_t tail[SECTOR_SIZE];


static bool read_sector(uint32_t lba, uint8_t *buf) {
    return vol.read(vol.ctx, lba, buf);
}


static bool has_boot_signature(const uint8_t *s) {
    return s[510] == 0x55 && s[511] == 0xaa;
}

static bool is_exfat(const uint8_t *s) {
    return has_boot_signature(s) && memcmp(s + 3, "EXFAT   ", 8) == 0;
}

// Does this sector look like a FAT boot sector (as opposed to an MBR or garbage)?
static bool looks_like_bpb(const uint8_t *s) {
    if(!has_boot_signature(s))
        return false;
    if(s[0] != 0xeb && s[0] != 0xe9)  // jump over the parameter block
        return false;

    const uint16_t bytes_per_sector = le16(s + 11);
    if(bytes_per_sector < 512 || bytes_per_sector > 4096 || (bytes_per_sector & (bytes_per_sector - 1)))
        return false;

    const uint8_t sec_per_clus = s[13];
    if(sec_per_clus == 0 || (sec_per_clus & (sec_per_clus - 1)))
        return false;

    if(le16(s + 14) == 0)  // reserved sectors
        return false;
    if(s[16] == 0 || s[16] > 2)  // number of FATs
        return false;
    if(le16(s + 22) == 0 && le32(s + 36) == 0)  // sectors per FAT
        return false;

    return true;
}


// Work out the layout of the volume from its boot sector. part_lba is where the volume starts.
static enum fat_result parse_bpb(const uint8_t *s, uint32_t part_lba) {
    if(le16(s + 11) != SECTOR_SIZE)
        return FAT_ERR_UNSUPPORTED;

    const uint8_t sec_per_clus = s[13];
    const uint32_t reserved = le16(s + 14);
    const uint32_t num_fats = s[16];
    const uint32_t root_entries = le16(s + 17);
    const uint32_t total_secs = le16(s + 19) ? le16(s + 19) : le32(s + 32);
    const uint32_t fat_secs = le16(s + 22) ? le16(s + 22) : le32(s + 36);
    const uint32_t root_secs = (root_entries * 32 + SECTOR_SIZE - 1) / SECTOR_SIZE;

    const uint64_t metadata = (uint64_t)reserved + (uint64_t)num_fats * fat_secs + root_secs;
    if(total_secs == 0 || metadata >= total_secs)
        return FAT_ERR_CORRUPT;

    const uint32_t clusters = (uint32_t)((total_secs - metadata) / sec_per_clus);

    // The number of clusters decides the type, as in Microsoft's FAT specification. The type
    // string in the boot sector is not to be trusted.
    if(clusters < 4085)
        return FAT_ERR_FAT12;

    vol.fat32 = clusters >= 65525;
    vol.sec_per_clus = sec_per_clus;
    vol.fat_lba = part_lba + reserved;
    vol.fat_secs = fat_secs;
    vol.root_lba = vol.fat_lba + num_fats * fat_secs;
    vol.root_secs = root_secs;
    vol.data_lba = vol.root_lba + root_secs;
    vol.clusters = clusters;
    vol.root_cluster = 0;

    if(vol.fat32) {
        vol.root_cluster = le32(s + 44);
        if(root_entries != 0 || vol.root_cluster < 2 || vol.root_cluster >= clusters + 2)
            return FAT_ERR_CORRUPT;
    } else {
        if(root_entries == 0)
            return FAT_ERR_CORRUPT;
    }

    vol.fat_cache_valid = false;
    return FAT_OK;
}


// Find the volume: a FAT boot sector at sector 0, or the first FAT partition of an MBR
static enum fat_result open_volume() {
    if(!read_sector(0, sector))
        return FAT_ERR_IO;

    if(is_exfat(sector))
        return FAT_ERR_EXFAT;

    if(looks_like_bpb(sector))
        return parse_bpb(sector, 0);

    if(!has_boot_signature(sector))
        return FAT_ERR_NO_VOLUME;

    // Looks like an MBR
    bool found = false, gpt = false;
    uint32_t fat_lba = 0, other_lba = 0;

    for(int i = 0; i < 4; i++) {
        const uint8_t *e = sector + 446 + 16 * i;
        const uint8_t type = e[4];
        const uint32_t start = le32(e + 8);
        const uint32_t count = le32(e + 12);

        if(type == 0xee)
            gpt = true;

        if(start == 0 || count == 0)
            continue;

        if(type == 0x01 || type == 0x04 || type == 0x06 || type == 0x0b || type == 0x0c || type == 0x0e) {
            if(!found) {
                found = true;
                fat_lba = start;
            }
        } else if(type == 0x07 && other_lba == 0) {
            other_lba = start;  // NTFS or exFAT: look at it below so exFAT gets its own error
        }
    }

    if(!found) {
        if(gpt)
            return FAT_ERR_GPT;
        if(other_lba) {
            if(!read_sector(other_lba, sector))
                return FAT_ERR_IO;
            if(is_exfat(sector))
                return FAT_ERR_EXFAT;
        }
        return FAT_ERR_NO_VOLUME;
    }

    if(!read_sector(fat_lba, sector))
        return FAT_ERR_IO;
    if(is_exfat(sector))
        return FAT_ERR_EXFAT;
    if(!looks_like_bpb(sector))
        return FAT_ERR_NO_VOLUME;

    return parse_bpb(sector, fat_lba);
}


static inline uint32_t cluster_lba(uint32_t cluster) {
    return vol.data_lba + (cluster - 2) * vol.sec_per_clus;
}

static inline bool valid_cluster(uint32_t cluster) {
    return cluster >= 2 && cluster < vol.clusters + 2;
}

enum next_kind {
    NEXT_CLUSTER,  // *next is the following cluster in the chain
    NEXT_END,      // end of chain
    NEXT_INVALID,  // free, reserved, bad or out of range: the chain is broken
};

// Look up what follows `cluster` in the FAT
static enum fat_result fat_next(uint32_t cluster, enum next_kind *kind, uint32_t *next) {
    const uint32_t offset = vol.fat32 ? cluster * 4 : cluster * 2;
    const uint32_t index = offset / SECTOR_SIZE;

    if(index >= vol.fat_secs)
        return FAT_ERR_CORRUPT;

    const uint32_t lba = vol.fat_lba + index;
    if(!vol.fat_cache_valid || vol.fat_cache_lba != lba) {
        vol.fat_cache_valid = false;
        if(!read_sector(lba, vol.fat_cache))
            return FAT_ERR_IO;
        vol.fat_cache_lba = lba;
        vol.fat_cache_valid = true;
    }

    const uint8_t *p = vol.fat_cache + (offset % SECTOR_SIZE);
    uint32_t entry;
    uint32_t end_marker;

    if(vol.fat32) {
        entry = le32(p) & 0x0fffffff;
        end_marker = 0x0ffffff8;
    } else {
        entry = le16(p);
        end_marker = 0xfff8;
    }

    if(entry >= end_marker) {
        *kind = NEXT_END;
    } else if(valid_cluster(entry)) {  // excludes free (0), reserved (1), bad (0xfff7) and out of range
        *kind = NEXT_CLUSTER;
        *next = entry;
    } else {
        *kind = NEXT_INVALID;
    }
    return FAT_OK;
}


struct entry {
    uint32_t first_cluster;
    uint32_t size;
};

enum scan_result {
    SCAN_CONTINUE,
    SCAN_FOUND,
    SCAN_END,  // reached the end-of-directory marker
};

// Look through one sector of directory entries for the file
static enum scan_result scan_sector(const uint8_t *s, const char *name11, struct entry *out) {
    for(int i = 0; i < SECTOR_SIZE; i += 32) {
        const uint8_t *e = s + i;

        if(e[0] == 0x00)
            return SCAN_END;
        if(e[0] == 0xe5)  // deleted
            continue;

        const uint8_t attr = e[11];
        if((attr & 0x3f) == 0x0f)  // long file name entry
            continue;
        if(attr & 0x18)  // volume label or directory
            continue;

        if(memcmp(e, name11, 11) != 0)
            continue;

        out->first_cluster = le16(e + 26);
        if(vol.fat32)
            out->first_cluster |= (uint32_t)le16(e + 20) << 16;
        out->size = le32(e + 28);
        return SCAN_FOUND;
    }
    return SCAN_CONTINUE;
}


static enum fat_result find_in_root(const char *name11, struct entry *out) {
    if(!vol.fat32) {
        for(uint32_t i = 0; i < vol.root_secs; i++) {
            if(!read_sector(vol.root_lba + i, sector))
                return FAT_ERR_IO;
            enum scan_result r = scan_sector(sector, name11, out);
            if(r == SCAN_FOUND)
                return FAT_OK;
            if(r == SCAN_END)
                return FAT_ERR_NOT_FOUND;
        }
        return FAT_ERR_NOT_FOUND;
    }

    // FAT32: the root directory is a cluster chain
    uint32_t cluster = vol.root_cluster;
    for(uint32_t steps = 0; steps <= vol.clusters; steps++) {
        for(uint32_t i = 0; i < vol.sec_per_clus; i++) {
            if(!read_sector(cluster_lba(cluster) + i, sector))
                return FAT_ERR_IO;
            enum scan_result r = scan_sector(sector, name11, out);
            if(r == SCAN_FOUND)
                return FAT_OK;
            if(r == SCAN_END)
                return FAT_ERR_NOT_FOUND;
        }

        enum next_kind kind;
        uint32_t next = 0;
        enum fat_result fr = fat_next(cluster, &kind, &next);
        if(fr != FAT_OK)
            return fr;
        if(kind == NEXT_END)
            return FAT_ERR_NOT_FOUND;
        if(kind == NEXT_INVALID)
            return FAT_ERR_CORRUPT;
        cluster = next;
    }
    return FAT_ERR_CORRUPT;
}


// ---- one subdirectory of the root (listing, and finding a file in it)
//
// Additive: none of the above is touched, so fat_open_file/fat_load_file's tested behaviour cannot
// change. walk_dir() is the same sector-or-cluster-chain loop as find_in_root() above, generalised
// to any directory and to a scan function that does not necessarily stop at the first match.

#define ATTR_DIR 0x10
#define ATTR_VOLUME 0x08
#define ATTR_LFN 0x0f

// Defined below (with struct entry and load_ext); forward declared so fat_file_extents can use it.
static enum fat_result build_extents(const struct entry *e, struct fat_extent *ext, uint32_t cap, uint32_t *n_ext,
    uint32_t *sectors);

// "GAME1   ROM" (11 bytes, on disk: space padded, no dot) -> "GAME1.ROM" (human readable, NUL
// terminated). A name with no extension (the last 3 bytes all spaces) is written without the dot.
// The fallback display name for an entry with no usable long-name sequence; moved here from
// menu.c so every caller of decode_dirent gets it uniformly, not just the menu's own listing.
static void format_short_name(const char name11[11], char *out) {
    unsigned n = 0;
    for(unsigned i = 0; i < 8 && name11[i] != ' '; i++)
        out[n++] = name11[i];
    if(name11[8] != ' ') {
        out[n++] = '.';
        for(unsigned i = 8; i < 11 && name11[i] != ' '; i++)
            out[n++] = name11[i];
    }
    out[n] = 0;
}

// Accumulates one VFAT long-name sequence: the run of ATTR_LFN entries immediately before the
// short entry that completes them. Entries arrive highest-ordinal first (the one with the 0x40
// "last" bit starts a sequence), so fragments are placed by ordinal, not append order. A sequence
// can span a sector or cluster boundary, so this is carried by the caller across scan() calls, not
// kept local to one sector -- hence a single static instance (fat.c is already not reentrant).
// 20 entries * 13 UTF-16 code units covers VFAT's maximum 255 character name with room to spare.
#define LFN_MAX_ENTRIES 20
struct lfn_state {
    bool valid;
    uint8_t checksum;
    uint8_t expected_count;
    uint32_t seen_mask;
    uint16_t chars[LFN_MAX_ENTRIES * 13];
};
static struct lfn_state lfn;

static void lfn_reset(void) {
    lfn.valid = false;
}

// Fold in one raw ATTR_LFN entry. Any entry that does not fit the sequence in progress (wrong
// checksum, wrong ordinal, or a corrupt count) invalidates it rather than guessing: the short
// entry that follows then just falls back to its own name, which is always present and correct.
static void lfn_entry(const uint8_t *e) {
    const uint8_t ord = e[0];
    const uint8_t checksum = e[13];

    if(ord & 0x40) {
        const uint8_t count = ord & 0x3f;
        lfn.valid = count >= 1 && count <= LFN_MAX_ENTRIES;
        if(!lfn.valid)
            return;
        lfn.checksum = checksum;
        lfn.expected_count = count;
        lfn.seen_mask = 0;
    }
    if(!lfn.valid)
        return;

    const uint8_t idx = ord & 0x3f;
    if(idx == 0 || idx > lfn.expected_count || checksum != lfn.checksum) {
        lfn.valid = false;
        return;
    }

    uint16_t *dst = lfn.chars + (idx - 1) * 13;
    for(unsigned i = 0; i < 5; i++)
        dst[i] = le16(e + 1 + i * 2);
    for(unsigned i = 0; i < 6; i++)
        dst[5 + i] = le16(e + 14 + i * 2);
    for(unsigned i = 0; i < 2; i++)
        dst[11 + i] = le16(e + 28 + i * 2);
    lfn.seen_mask |= 1u << (idx - 1);
}

// At a short entry: write the accumulated long name into out (NUL terminated, truncated to
// FAT_MAX_NAME, never rejected for being long) if one is available and valid for this entry.
// Returns false (leaving *out untouched) if there is nothing usable, so the caller can fall back
// to the short name: no sequence was in progress, it is missing a fragment, its checksum does not
// match this short name (an orphaned sequence from a deleted or overwritten entry -- a real,
// documented FAT corner case), or it contains a character this project has no use for displaying.
static bool lfn_finish(const char short_name[11], char *out) {
    if(!lfn.valid)
        return false;
    const uint32_t want_mask = lfn.expected_count >= 32 ? 0xffffffffu : (1u << lfn.expected_count) - 1;
    if((lfn.seen_mask & want_mask) != want_mask)
        return false;

    uint8_t sum = 0;
    for(unsigned i = 0; i < 11; i++)
        sum = (uint8_t)(((sum & 1) ? 0x80 : 0) + (sum >> 1) + (uint8_t)short_name[i]);
    if(sum != lfn.checksum)
        return false;

    const uint32_t total_chars = (uint32_t)lfn.expected_count * 13;
    uint32_t n = 0;
    for(uint32_t i = 0; i < total_chars; i++) {
        const uint16_t c = lfn.chars[i];
        if(c == 0x0000 || c == 0xffff)
            break;
        if(c < 0x20 || c > 0x7e)
            return false;  // not ASCII: fall back to the short name rather than mangle it
        if(n < FAT_MAX_NAME)
            out[n++] = (char)c;
    }
    out[n] = 0;
    return true;
}

// Decode one raw 32 byte entry, already known to be a real entry (not free, deleted, a long-name
// entry or the volume label). Must be called right after any preceding long-name entries were fed
// to lfn_entry, before anything else consumes or resets that state.
static void decode_dirent(const uint8_t *e, struct fat_dirent *out) {
    memcpy(out->name11, e, 11);
    out->is_dir = (e[11] & ATTR_DIR) != 0;
    out->first_cluster = le16(e + 26);
    if(vol.fat32)
        out->first_cluster |= (uint32_t)le16(e + 20) << 16;
    out->size = out->is_dir ? 0 : le32(e + 28);

    if(!lfn_finish(out->name11, out->name))
        format_short_name(out->name11, out->name);
}

typedef enum scan_result (*sector_scan_fn)(const uint8_t *s, void *ctx);

// Call `scan` for each sector of a directory's entries, stopping when it says so or at the end of
// the directory. `fixed_root` is the FAT16 fixed root region (ignoring `cluster`); anything else,
// including the FAT32 root and any subdirectory on either type, is a cluster chain.
static enum fat_result walk_dir(bool fixed_root, uint32_t cluster, sector_scan_fn scan, void *ctx) {
    if(fixed_root) {
        for(uint32_t i = 0; i < vol.root_secs; i++) {
            if(!read_sector(vol.root_lba + i, sector))
                return FAT_ERR_IO;
            enum scan_result r = scan(sector, ctx);
            if(r == SCAN_FOUND)
                return FAT_OK;
            if(r == SCAN_END)
                return FAT_ERR_NOT_FOUND;
        }
        return FAT_ERR_NOT_FOUND;
    }

    for(uint32_t steps = 0; steps <= vol.clusters; steps++) {
        for(uint32_t i = 0; i < vol.sec_per_clus; i++) {
            if(!read_sector(cluster_lba(cluster) + i, sector))
                return FAT_ERR_IO;
            enum scan_result r = scan(sector, ctx);
            if(r == SCAN_FOUND)
                return FAT_OK;
            if(r == SCAN_END)
                return FAT_ERR_NOT_FOUND;
        }

        enum next_kind kind;
        uint32_t next = 0;
        enum fat_result fr = fat_next(cluster, &kind, &next);
        if(fr != FAT_OK)
            return fr;
        if(kind == NEXT_END)
            return FAT_ERR_NOT_FOUND;
        if(kind == NEXT_INVALID)
            return FAT_ERR_CORRUPT;
        cluster = next;
    }
    return FAT_ERR_CORRUPT;
}

// Look through one sector of directory entries for a name, requiring (or refusing) a directory:
// fat_open_subdir wants only a directory called "MSX", fat_find_in_dir only a file or directory
// entry regardless of kind (want_dir is NULL then).
struct find_ctx {
    const char *name11;
    const bool *want_dir;
    struct fat_dirent *out;
};
static enum scan_result scan_find(const uint8_t *s, void *vctx) {
    struct find_ctx *ctx = vctx;
    for(int i = 0; i < SECTOR_SIZE; i += 32) {
        const uint8_t *e = s + i;
        if(e[0] == 0x00)
            return SCAN_END;
        if(e[0] == 0xe5) {  // deleted: also discards any long-name entries just before it
            lfn_reset();
            continue;
        }
        const uint8_t attr = e[11];
        if((attr & 0x3f) == ATTR_LFN) {
            lfn_entry(e);
            continue;
        }
        if(attr & ATTR_VOLUME) {
            lfn_reset();
            continue;
        }
        if(ctx->want_dir && ((attr & ATTR_DIR) != 0) != *ctx->want_dir) {
            lfn_reset();
            continue;
        }
        if(memcmp(e, ctx->name11, 11) != 0) {
            lfn_reset();
            continue;
        }
        decode_dirent(e, ctx->out);
        lfn_reset();
        return SCAN_FOUND;
    }
    return SCAN_CONTINUE;
}

// Call the caller's callback for every real entry (deleted, long-name, the volume label, "." and
// ".." skipped); stop when it returns false.
struct list_ctx {
    fat_dirent_fn cb;
    void *cb_ctx;
};
static enum scan_result scan_list(const uint8_t *s, void *vctx) {
    struct list_ctx *ctx = vctx;
    for(int i = 0; i < SECTOR_SIZE; i += 32) {
        const uint8_t *e = s + i;
        if(e[0] == 0x00)
            return SCAN_END;
        if(e[0] == 0xe5) {
            lfn_reset();
            continue;
        }
        const uint8_t attr = e[11];
        if((attr & 0x3f) == ATTR_LFN) {
            lfn_entry(e);
            continue;
        }
        if(attr & ATTR_VOLUME) {
            lfn_reset();
            continue;
        }
        if(e[0] == '.') {  // "." and ".."
            lfn_reset();
            continue;
        }
        struct fat_dirent d;
        decode_dirent(e, &d);
        lfn_reset();
        if(!ctx->cb(ctx->cb_ctx, &d))
            return SCAN_FOUND;  // the caller is done, not an error
    }
    return SCAN_CONTINUE;
}

// ASCII case-insensitive compare: FAT names decoded here are only ever ASCII (lfn_finish already
// falls back to the short name for anything else), so this avoids pulling in strings.h for the one
// function that needs it.
static bool names_match(const char *a, const char *b) {
    while(*a && *b) {
        char ca = *a, cb = *b;
        if(ca >= 'a' && ca <= 'z')
            ca = (char)(ca - ('a' - 'A'));
        if(cb >= 'a' && cb <= 'z')
            cb = (char)(cb - ('a' - 'A'));
        if(ca != cb)
            return false;
        a++;
        b++;
    }
    return *a == *b;  // both must end at the same time
}

// Every real entry gets decoded (its display name is needed to compare it, whichever way the
// match turns out), unlike scan_find's short-name search, which only ever needs to decode the one
// entry it already knows has matched.
struct find_name_ctx {
    const char *name;
    struct fat_dirent *out;
};
static enum scan_result scan_find_name(const uint8_t *s, void *vctx) {
    struct find_name_ctx *ctx = vctx;
    for(int i = 0; i < SECTOR_SIZE; i += 32) {
        const uint8_t *e = s + i;
        if(e[0] == 0x00)
            return SCAN_END;
        if(e[0] == 0xe5) {
            lfn_reset();
            continue;
        }
        const uint8_t attr = e[11];
        if((attr & 0x3f) == ATTR_LFN) {
            lfn_entry(e);
            continue;
        }
        if(attr & ATTR_VOLUME) {
            lfn_reset();
            continue;
        }
        if(e[0] == '.') {  // "." and ".."
            lfn_reset();
            continue;
        }
        decode_dirent(e, ctx->out);
        lfn_reset();
        if(names_match(ctx->out->name, ctx->name))
            return SCAN_FOUND;
    }
    return SCAN_CONTINUE;
}

enum fat_result fat_open_subdir(fat_read_sector_fn read, void *ctx, const char dirname11[11], struct fat_dir *out) {
    vol.read = read;
    vol.ctx = ctx;

    enum fat_result r = open_volume();
    if(r != FAT_OK)
        return r;

    struct fat_dirent d;
    const bool want_dir = true;
    struct find_ctx fctx = { dirname11, &want_dir, &d };
    lfn_reset();  // a fresh walk: no state from however the last one ended should carry over
    r = walk_dir(!vol.fat32, vol.fat32 ? vol.root_cluster : 0, scan_find, &fctx);
    if(r != FAT_OK)
        return r;

    if(!valid_cluster(d.first_cluster))
        return FAT_ERR_CORRUPT;
    out->cluster = d.first_cluster;
    return FAT_OK;
}

enum fat_result fat_list_dir(fat_read_sector_fn read, void *ctx, const struct fat_dir *dir, fat_dirent_fn cb,
    void *cb_ctx) {
    vol.read = read;
    vol.ctx = ctx;

    enum fat_result r = open_volume();
    if(r != FAT_OK)
        return r;

    struct list_ctx lctx = { cb, cb_ctx };
    lfn_reset();  // a fresh walk: no state from however the last one ended should carry over
    r = walk_dir(false, dir->cluster, scan_list, &lctx);
    // Running off the end without the callback asking to stop is a normal, complete listing
    return r == FAT_ERR_NOT_FOUND ? FAT_OK : r;
}

enum fat_result fat_find_in_dir(fat_read_sector_fn read, void *ctx, const struct fat_dir *dir,
    const char name11[11], struct fat_dirent *out) {
    vol.read = read;
    vol.ctx = ctx;

    enum fat_result r = open_volume();
    if(r != FAT_OK)
        return r;

    struct find_ctx fctx = { name11, NULL, out };
    lfn_reset();  // a fresh walk: no state from however the last one ended should carry over
    return walk_dir(false, dir->cluster, scan_find, &fctx);
}

enum fat_result fat_find_by_name(fat_read_sector_fn read, void *ctx, const struct fat_dir *dir, const char *name,
    struct fat_dirent *out) {
    vol.read = read;
    vol.ctx = ctx;

    enum fat_result r = open_volume();
    if(r != FAT_OK)
        return r;

    struct find_name_ctx fctx = { name, out };
    lfn_reset();  // a fresh walk: no state from however the last one ended should carry over
    return walk_dir(false, dir->cluster, scan_find_name, &fctx);
}

enum fat_result fat_file_extents(const struct fat_dirent *file, struct fat_extent *ext, uint32_t ext_cap,
    struct fat_file *out) {
    out->size = file->size;
    out->ext = ext;
    out->n_ext = 0;
    out->sectors = 0;
    if(file->is_dir)
        return FAT_ERR_NOT_FOUND;  // not a file
    if(file->size == 0)
        return FAT_ERR_EMPTY;

    // build_extents only needs first_cluster and size, which struct entry and struct fat_dirent
    // both carry; a local struct entry keeps its signature untouched.
    const struct entry e = { file->first_cluster, file->size };
    return build_extents(&e, ext, ext_cap, &out->n_ext, &out->sectors);
}


// The extents a file loaded by fat_load_file may need. A file that fits the buffer has at most 256
// clusters of 512 bytes (128 KB), so this is never too few for it.
#define LOAD_MAX_EXTENTS 512
static struct fat_extent load_ext[LOAD_MAX_EXTENTS];

// Walk the cluster chain of a file once and record where its sectors are, as runs of consecutive
// sectors. Every cluster of a file is different, so a cluster seen twice is a looping chain: each
// new cluster is checked against the runs so far (the number of runs is small, or the caller has
// given a bounded buffer).
static enum fat_result build_extents(const struct entry *e, struct fat_extent *ext, uint32_t cap, uint32_t *n_ext,
    uint32_t *sectors) {
    uint32_t cluster = e->first_cluster;
    if(!valid_cluster(cluster))
        return FAT_ERR_CORRUPT;

    // The file needs this many sectors, so a chain that is longer or loops cannot make us walk forever
    const uint32_t total = (e->size >> 9) + ((e->size & (SECTOR_SIZE - 1)) != 0);
    const uint32_t needed = (total + vol.sec_per_clus - 1) / vol.sec_per_clus;

    uint32_t n = 0;
    uint32_t done = 0;  // sectors of the file placed so far

    for(uint32_t used = 0; used < needed; used++) {
        const uint32_t lba = cluster_lba(cluster);

        for(uint32_t i = 0; i < n; i++) {
            const uint32_t count = (i + 1 < n ? ext[i + 1].first : done) - ext[i].first;
            if(lba - ext[i].lba < count)  // unsigned: false when lba is below the run
                return FAT_ERR_CORRUPT;
        }

        // Extend the last run if this cluster follows it directly on the card
        if(n > 0 && ext[n - 1].lba + (done - ext[n - 1].first) == lba) {
            // nothing to record
        } else {
            if(n == cap)
                return FAT_ERR_FRAGMENTED;
            ext[n].first = done;
            ext[n].lba = lba;
            n++;
        }

        const uint32_t left = total - done;
        done += left < vol.sec_per_clus ? left : vol.sec_per_clus;
        if(done == total)
            break;

        enum next_kind kind;
        uint32_t next = 0;
        enum fat_result fr = fat_next(cluster, &kind, &next);
        if(fr != FAT_OK)
            return fr;
        if(kind != NEXT_CLUSTER)  // the chain ended or broke before the file did
            return FAT_ERR_CORRUPT;
        cluster = next;
    }

    if(done != total)
        return FAT_ERR_CORRUPT;
    *n_ext = n;
    *sectors = total;
    return FAT_OK;
}


bool fat_file_lba(const struct fat_file *f, uint32_t sector, uint32_t *lba) {
    if(sector >= f->sectors || f->n_ext == 0)
        return false;

    // The last run that starts at or before `sector`
    uint32_t lo = 0, hi = f->n_ext - 1;
    while(lo < hi) {
        const uint32_t mid = (lo + hi + 1) / 2;
        if(f->ext[mid].first <= sector)
            lo = mid;
        else
            hi = mid - 1;
    }
    *lba = f->ext[lo].lba + (sector - f->ext[lo].first);
    return true;
}


enum fat_result fat_open_file(fat_read_sector_fn read, void *ctx, const char name11[11], struct fat_extent *ext,
    uint32_t ext_cap, struct fat_file *out) {
    vol.read = read;
    vol.ctx = ctx;

    enum fat_result r = open_volume();
    if(r != FAT_OK)
        return r;

    struct entry e;
    r = find_in_root(name11, &e);
    if(r != FAT_OK)
        return r;

    out->size = e.size;
    out->ext = ext;
    out->n_ext = 0;
    out->sectors = 0;
    if(e.size == 0)
        return FAT_ERR_EMPTY;

    // build_extents only writes n_ext and sectors when it succeeds, so an error leaves them at 0
    return build_extents(&e, ext, ext_cap, &out->n_ext, &out->sectors);
}


enum fat_result fat_load_file(fat_read_sector_fn read, void *ctx, const char name11[11], uint8_t *dst,
    uint32_t dst_size, uint32_t *file_size) {
    vol.read = read;
    vol.ctx = ctx;

    enum fat_result r = open_volume();
    if(r != FAT_OK)
        return r;

    struct entry e;
    r = find_in_root(name11, &e);
    if(r != FAT_OK)
        return r;

    if(file_size)
        *file_size = e.size;
    if(e.size == 0)
        return FAT_ERR_EMPTY;
    if(e.size > dst_size)
        return FAT_ERR_TOO_BIG;

    struct fat_file f;
    f.size = e.size;
    f.ext = load_ext;
    r = build_extents(&e, load_ext, LOAD_MAX_EXTENTS, &f.n_ext, &f.sectors);
    if(r != FAT_OK)
        return r;

    uint32_t remaining = e.size;
    uint32_t offset = 0;
    for(uint32_t sec = 0; sec < f.sectors; sec++) {
        uint32_t lba;
        if(!fat_file_lba(&f, sec, &lba))
            return FAT_ERR_CORRUPT;

        if(remaining >= SECTOR_SIZE) {
            if(!read_sector(lba, dst + offset))
                return FAT_ERR_IO;
            offset += SECTOR_SIZE;
            remaining -= SECTOR_SIZE;
        } else {
            // The last, partial sector: read it aside so nothing is written past the file
            if(!read_sector(lba, tail))
                return FAT_ERR_IO;
            memcpy(dst + offset, tail, remaining);
            offset += remaining;
            remaining = 0;
        }
    }

    memset(dst + e.size, FAT_PAD_BYTE, dst_size - e.size);
    return FAT_OK;
}


enum fat_result fat_load_dirent(fat_read_sector_fn read, void *ctx, const struct fat_dirent *file, uint8_t *dst,
    uint32_t dst_size) {
    vol.read = read;
    vol.ctx = ctx;

    if(file->is_dir)
        return FAT_ERR_NOT_FOUND;
    if(file->size == 0)
        return FAT_ERR_EMPTY;
    if(file->size > dst_size)
        return FAT_ERR_TOO_BIG;

    const struct entry e = { file->first_cluster, file->size };
    struct fat_file f;
    f.size = file->size;
    f.ext = load_ext;
    enum fat_result r = build_extents(&e, load_ext, LOAD_MAX_EXTENTS, &f.n_ext, &f.sectors);
    if(r != FAT_OK)
        return r;

    uint32_t remaining = file->size;
    uint32_t offset = 0;
    for(uint32_t sec = 0; sec < f.sectors; sec++) {
        uint32_t lba;
        if(!fat_file_lba(&f, sec, &lba))
            return FAT_ERR_CORRUPT;

        if(remaining >= SECTOR_SIZE) {
            if(!read_sector(lba, dst + offset))
                return FAT_ERR_IO;
            offset += SECTOR_SIZE;
            remaining -= SECTOR_SIZE;
        } else {
            if(!read_sector(lba, tail))
                return FAT_ERR_IO;
            memcpy(dst + offset, tail, remaining);
            offset += remaining;
            remaining = 0;
        }
    }

    memset(dst + file->size, FAT_PAD_BYTE, dst_size - file->size);
    return FAT_OK;
}


const char *fat_result_name(enum fat_result r) {
    switch(r) {
        case FAT_OK: return "ok";
        case FAT_ERR_IO: return "io";
        case FAT_ERR_NO_VOLUME: return "no-volume";
        case FAT_ERR_GPT: return "gpt";
        case FAT_ERR_EXFAT: return "exfat";
        case FAT_ERR_FAT12: return "fat12";
        case FAT_ERR_UNSUPPORTED: return "unsupported";
        case FAT_ERR_NOT_FOUND: return "not-found";
        case FAT_ERR_EMPTY: return "empty";
        case FAT_ERR_TOO_BIG: return "too-big";
        case FAT_ERR_CORRUPT: return "corrupt";
        case FAT_ERR_FRAGMENTED: return "fragmented";
    }
    return "?";
}
