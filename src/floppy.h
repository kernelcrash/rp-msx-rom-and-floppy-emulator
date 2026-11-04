#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "fat.h"
#include "fdc.h"

// Disk images for the floppy controller: a file on the SD card (located by fat_open_file) seen
// as cylinders. One operation runs at a time and advances one sector per floppy_poll() call, so a
// caller that polls never waits for the card for more than one sector. struct floppy_io's
// read_begin/read_step and write_begin/write_step let that one sector itself be fetched or stored
// in short steps too (see sd_read_step/sd_write_step), for a caller -- the render core -- that
// cannot block through a card operation either; plain read/write remain as the blocking fallback
// for anything that does not need that (tests, and the flash-backed disk variant).
//
// Geometry: an MSX disk file is laid out cylinder by cylinder, each cylinder holding side 0
// (9 sectors of 512 bytes) and then, for a double sided disk, side 1. 737280 bytes is 80 cylinders
// of two sides, 368640 is 80 cylinders of one side. Anything else is refused.

struct floppy_io {
    void *ctx;
    bool (*read)(void *ctx, uint32_t lba, uint8_t *buf);
    bool (*write)(void *ctx, uint32_t lba, const uint8_t *buf);
    // Optional: read a sector in short steps (see sd_read_step) so a poll never holds the caller up. When
    // read_begin is set it is used for loads instead of `read`: read_begin starts a sector (false if it cannot),
    // and each read_step returns 0 (not finished), 1 (done) or -1 (failed).
    bool (*read_begin)(void *ctx, uint32_t lba, uint8_t *buf);
    int (*read_step)(void *ctx);
    // The same, for writes (see sd_write_step): when write_begin is set it is used for write-backs
    // instead of `write`, with the same begin/step shape and return convention.
    bool (*write_begin)(void *ctx, uint32_t lba, const uint8_t *buf);
    int (*write_step)(void *ctx);
};

struct floppy_drive {
    bool present;
    bool read_only;
    uint8_t sides;        // 1 or 2
    uint16_t cylinders;
    struct fat_file file;
    uint8_t *cache;        // the first cache_cyls cylinders held in RAM (FDC_CYL_BYTES each), or NULL
    uint16_t cache_cyls;
};

enum floppy_status {
    FLOPPY_IDLE,     // nothing in progress
    FLOPPY_PENDING,  // more to do: poll again
    FLOPPY_OK,       // the operation finished
    FLOPPY_ERROR,    // the operation failed (card error, or a request outside the file)
};

enum floppy_op { OP_NONE, OP_LOAD, OP_WRITE };

struct floppy {
    struct floppy_io io;
    struct floppy_drive drive[FDC_MAX_DRIVES];

    enum floppy_op op;
    uint8_t op_drive;
    uint32_t op_first;     // file sector of the first sector of the operation
    uint8_t op_count, op_done;
    bool inflight;         // a stepwise sector operation (read or write) has been begun and not finished
    uint8_t *op_buf;
    const uint8_t *op_wbuf;
};

void floppy_init(struct floppy *f, const struct floppy_io *io);

// Make `file` drive `drive`. False (and the drive stays absent) if the size is not a known disk.
// The file (and its extent table) must stay valid.
bool floppy_attach(struct floppy *f, unsigned drive, const struct fat_file *file, bool read_only);

// Start loading cylinder `cyl` (both sides) into buf[0 .. 9216). False if busy or out of range.
// Keep the first `cyls` cylinders of drive `drive` in `mem` (cyls * FDC_CYL_BYTES bytes) and fill them now, one
// cylinder after another from the card (this BLOCKS: call it at start-up, before the machine runs). A load of a
// cylinder in the cache is then a copy from RAM, with no card access at all. Returns the number of cylinders
// that are now cached (fewer than asked if the card fails part way: the rest are read from the card as before).
// The cache is read only: it does not follow writes (writes are not enabled yet).
unsigned floppy_cache_preload(struct floppy *f, unsigned drive, uint8_t *mem, unsigned cyls);

bool floppy_start_load(struct floppy *f, unsigned drive, unsigned cyl, uint8_t *buf);

// Start writing back one sector: `index` is the sector's place in the cylinder buffer (0-8 side 0,
// 9-17 side 1), buf points at its 512 bytes. False if busy, read only, or outside the disk.
bool floppy_start_write(struct floppy *f, unsigned drive, unsigned cyl, unsigned index, const uint8_t *buf);

enum floppy_status floppy_poll(struct floppy *f);
