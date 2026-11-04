#pragma once

#include <stdbool.h>
#include <stdint.h>

// A minimal read-only FAT16/FAT32 reader: it finds one file in the root directory by its 8.3
// name and reads it into a buffer. It can also list, or find a file in, one subdirectory of the
// root (this project's convention: an "MSX" directory of ROM and disk images, next to a couple of
// fixed files in the root). It knows nothing about the hardware: sectors come from a callback, so
// it runs unchanged against a disk image on a PC.
//
// Supported: 512 byte sectors, FAT16 and FAT32, a volume with no partition table or the first
// FAT partition of an MBR, one level of subdirectory, long file names (VFAT) when listing or
// finding an entry of that one subdirectory. Not supported: FAT12, exFAT, GPT, other sector
// sizes, subdirectories of subdirectories, long file names in the root directory itself (the
// fixed files this project loads from the root -- DEFAULT.ROM, MENU.ROM, DISK.ROM -- are always
// matched by their 8.3 name11, which is all fat_open_file/fat_load_file ever need).
//
// Not reentrant (it uses static buffers, so it does not need much stack).

// Read only: this code never writes to the card. A file that is written in place (a disk image)
// is located with fat_open_file and then addressed by sector through fat_file_lba.

// Read one 512 byte sector. Returns false on failure.
typedef bool (*fat_read_sector_fn)(void *ctx, uint32_t lba, uint8_t *buf);

enum fat_result {
    FAT_OK = 0,
    FAT_ERR_IO,           // the sector callback failed
    FAT_ERR_NO_VOLUME,    // no FAT boot sector or FAT partition found
    FAT_ERR_GPT,          // GPT partition table
    FAT_ERR_EXFAT,        // exFAT volume
    FAT_ERR_FAT12,        // FAT12 volume
    FAT_ERR_UNSUPPORTED,  // for example a sector size other than 512
    FAT_ERR_NOT_FOUND,    // no such file in the root directory
    FAT_ERR_EMPTY,        // the file has zero length
    FAT_ERR_TOO_BIG,      // the file is larger than the buffer
    FAT_ERR_CORRUPT,      // inconsistent volume or a broken cluster chain
    FAT_ERR_FRAGMENTED,   // the file needs more extents than the caller's buffer holds
};

// Byte used to fill the buffer after the end of a file that is shorter than the buffer
#define FAT_PAD_BYTE 0xff

// Read the file whose 8.3 name is `name11` (11 characters, the name padded with spaces to 8 and
// the extension padded to 3, upper case, no dot: "DEFAULT ROM") from the root directory into
// dst[0..dst_size). A file shorter than dst_size is padded with FAT_PAD_BYTE; a longer one is an
// error (nothing is truncated). *file_size (may be NULL) receives the file's length.
// A file may have at most 512 separate runs of clusters (a 128 KB file cannot exceed that).
// Chains that loop, leave the volume or end early are reported as FAT_ERR_CORRUPT.
enum fat_result fat_load_file(fat_read_sector_fn read, void *ctx, const char name11[11], uint8_t *dst,
    uint32_t dst_size, uint32_t *file_size);

// Where a file's sectors are. The chain is walked once, by fat_open_file, so that reading or
// writing a sector of the file at run time is a table lookup with no FAT access.
//
// A file is a list of extents: runs of consecutive sectors on the card, in file order. Extent i
// covers file sectors first[i] .. first[i+1]-1 (the last one up to `sectors`-1), which are on the
// card at lba[i], lba[i]+1, ... A contiguous file is one extent.
struct fat_extent {
    uint32_t first;  // index in the file of the first sector of the run
    uint32_t lba;    // sector on the card
};

struct fat_file {
    const struct fat_extent *ext;
    uint32_t n_ext;
    uint32_t sectors;  // sectors that hold file data (the size rounded up to whole sectors)
    uint32_t size;     // bytes
};

// Find the file `name11` in the root directory and record where its sectors are in ext[0..ext_cap).
// A file with more runs than ext_cap is FAT_ERR_FRAGMENTED (a fully scattered file of N clusters
// needs N). The same checks apply as for fat_load_file (loops, breaks, chains leaving the
// volume are FAT_ERR_CORRUPT). On any error out->n_ext and out->sectors are 0. `ext` must stay
// valid for as long as `out` is used.
enum fat_result fat_open_file(fat_read_sector_fn read, void *ctx, const char name11[11], struct fat_extent *ext,
    uint32_t ext_cap, struct fat_file *out);

// The card sector that holds sector `sector` of the file. False if the file has no such sector:
// the caller must never read or write anything else, so a bad request can not touch the card
// outside the file.
bool fat_file_lba(const struct fat_file *f, uint32_t sector, uint32_t *lba);

// A short name for a result, for messages
const char *fat_result_name(enum fat_result r);


// ---- one subdirectory of the root (listing, and finding a file in it)

// Where a directory's entries start: opaque, filled in by fat_open_subdir. The root itself is not
// represented by this (fat_open_file/fat_load_file already cover it); this is only for the one
// subdirectory this project needs, which (unlike the root on FAT16) is always a cluster chain.
struct fat_dir {
    uint32_t cluster;
};

// The longest display name fat_list_dir/fat_find_in_dir will reconstruct from a VFAT long-name
// entry sequence (chosen to match the kcmfs menu protocol's own 128 byte, NUL-terminated listing
// slots -- see menu.h's MENU_SLOT_BYTES); a longer name is truncated, not rejected.
#define FAT_MAX_NAME 127

// An entry as fat_list_dir or fat_find_in_dir hands it back: the raw 8.3 name (11 characters, not
// null terminated, exactly as stored: upper case, no dot, space padded), the display name (NUL
// terminated: the long name if the entry had a valid VFAT long-name sequence before it, otherwise
// name11 reformatted as "NAME.EXT"), whether it is itself a directory, and (for a file) its size
// and where its data starts.
struct fat_dirent {
    char name11[11];
    char name[FAT_MAX_NAME + 1];
    bool is_dir;
    uint32_t size;
    uint32_t first_cluster;
};

// Find the subdirectory `dirname11` of the root. Case as on disk (upper case, no dot, space
// padded): "MSX        " for "MSX". FAT_ERR_NOT_FOUND if there is no such directory (or a file of
// that name instead).
enum fat_result fat_open_subdir(fat_read_sector_fn read, void *ctx, const char dirname11[11], struct fat_dir *out);

// Call `cb` once for each entry of `dir` (deleted entries, long-name entries, the volume label,
// and "." / ".." are skipped). Stops early if `cb` returns false; otherwise runs to the end of the
// directory. Either way that is success: the return value is only for a card or volume error.
typedef bool (*fat_dirent_fn)(void *ctx, const struct fat_dirent *e);
enum fat_result fat_list_dir(fat_read_sector_fn read, void *ctx, const struct fat_dir *dir, fat_dirent_fn cb,
    void *cb_ctx);

// Find one entry of `dir` by its 8.3 name (a file or a directory; `out->is_dir` says which).
// FAT_ERR_NOT_FOUND if there is no such entry.
enum fat_result fat_find_in_dir(fat_read_sector_fn read, void *ctx, const struct fat_dir *dir,
    const char name11[11], struct fat_dirent *out);

// Find one entry of `dir` by its display name (`name`, as fat_list_dir would have shown it: the
// long name when it has one, the 8.3 name reformatted as "NAME.EXT" otherwise), matched ASCII
// case-insensitively -- FAT is a case-insensitive filesystem, so "zaxxon.dsk" and "Zaxxon.DSK" are
// the same entry. A file or a directory; `out->is_dir` says which. FAT_ERR_NOT_FOUND if there is no
// such entry (including one that exists only under a different name, such as a short alias).
enum fat_result fat_find_by_name(fat_read_sector_fn read, void *ctx, const struct fat_dir *dir, const char *name,
    struct fat_dirent *out);

// Record where a file's sectors are, the same job fat_open_file does, for a file already found
// with fat_find_in_dir or fat_find_by_name. Must be called right after it (same read/ctx, nothing
// else touching the card in between): it relies on the volume that find opened still being
// current. On any error out->n_ext and out->sectors are 0, as for fat_open_file.
enum fat_result fat_file_extents(const struct fat_dirent *file, struct fat_extent *ext, uint32_t ext_cap,
    struct fat_file *out);

// Load a file already found with fat_find_in_dir or fat_find_by_name whole into dst[0..dst_size),
// the same job fat_load_file does for a root file: a file shorter than dst_size is padded with
// FAT_PAD_BYTE; a longer one is FAT_ERR_TOO_BIG. Must be called right after that find, for the
// same reason as fat_file_extents.
enum fat_result fat_load_dirent(fat_read_sector_fn read, void *ctx, const struct fat_dirent *file, uint8_t *dst,
    uint32_t dst_size);
