#include "floppy.h"

#include <stddef.h>

void floppy_init(struct floppy *f, const struct floppy_io *io) {
    f->io = *io;
    for(int i = 0; i < FDC_MAX_DRIVES; i++)
        f->drive[i].present = false;
    f->op = OP_NONE;
    f->inflight = false;
}

bool floppy_attach(struct floppy *f, unsigned drive, const struct fat_file *file, bool read_only) {
    if(drive >= FDC_MAX_DRIVES)
        return false;

    struct floppy_drive *d = &f->drive[drive];
    d->present = false;

    if(file->size == 737280 && file->sectors == 1440) {
        d->sides = 2;
    } else if(file->size == 368640 && file->sectors == 720) {
        d->sides = 1;
    } else {
        return false;
    }

    d->cylinders = 80;
    d->read_only = read_only;
    d->file = *file;
    d->cache = NULL;
    d->cache_cyls = 0;
    d->present = true;
    return true;
}

unsigned floppy_cache_preload(struct floppy *f, unsigned drive, uint8_t *mem, unsigned cyls) {
    if(drive >= FDC_MAX_DRIVES || !f->drive[drive].present || f->op != OP_NONE)
        return 0;
    struct floppy_drive *d = &f->drive[drive];
    if(cyls > d->cylinders)
        cyls = d->cylinders;
    d->cache = NULL;  // not used while it is being filled
    d->cache_cyls = 0;

    unsigned done = 0;
    while(done < cyls) {
        if(!floppy_start_load(f, drive, done, mem + (size_t)done * FDC_CYL_BYTES))
            break;
        enum floppy_status st;
        while((st = floppy_poll(f)) == FLOPPY_PENDING) {
        }
        if(st != FLOPPY_OK)
            break;
        done++;
    }

    d->cache = mem;
    d->cache_cyls = (uint16_t)done;
    return done;
}

bool floppy_start_load(struct floppy *f, unsigned drive, unsigned cyl, uint8_t *buf) {
    if(f->op != OP_NONE || drive >= FDC_MAX_DRIVES || !f->drive[drive].present)
        return false;

    const struct floppy_drive *d = &f->drive[drive];
    if(cyl >= d->cylinders)
        return false;

    f->op = OP_LOAD;
    f->op_drive = drive;
    f->op_count = d->sides * FDC_SECTORS_PER_SIDE;
    f->op_first = cyl * f->op_count;
    f->op_done = 0;
    f->op_buf = buf;
    return true;
}

bool floppy_start_write(struct floppy *f, unsigned drive, unsigned cyl, unsigned index, const uint8_t *buf) {
    if(f->op != OP_NONE || drive >= FDC_MAX_DRIVES || !f->drive[drive].present)
        return false;

    const struct floppy_drive *d = &f->drive[drive];
    const unsigned per_cyl = d->sides * FDC_SECTORS_PER_SIDE;
    if(d->read_only || cyl >= d->cylinders || index >= per_cyl)
        return false;

    f->op = OP_WRITE;
    f->op_drive = drive;
    f->op_count = 1;
    f->op_first = cyl * per_cyl + index;
    f->op_done = 0;
    f->op_wbuf = buf;
    return true;
}

enum floppy_status floppy_poll(struct floppy *f) {
    if(f->op == OP_NONE)
        return FLOPPY_IDLE;

    const struct floppy_drive *d = &f->drive[f->op_drive];
    uint32_t lba;
    // Every sector goes through the file's map, which refuses anything outside the file
    bool ok = fat_file_lba(&d->file, f->op_first + f->op_done, &lba);

    if(ok) {
        if(f->op == OP_LOAD) {
            uint8_t *dst = f->op_buf + (uint32_t)f->op_done * FDC_SECTOR_BYTES;
            const unsigned per_cyl = d->sides * FDC_SECTORS_PER_SIDE;
            const unsigned cyl = f->op_first / per_cyl;
            if(d->cache && cyl < d->cache_cyls && f->op_buf != d->cache + (size_t)cyl * FDC_CYL_BYTES) {
                // in the RAM cache: a copy, word by word, with no card access
                const uint32_t *s = (const uint32_t *)(d->cache + (size_t)cyl * FDC_CYL_BYTES +
                    (size_t)f->op_done * FDC_SECTOR_BYTES);
                uint32_t *w = (uint32_t *)dst;
                for(unsigned i = 0; i < FDC_SECTOR_BYTES / 4; i++)
                    w[i] = s[i];
            } else if(f->io.read_begin) {
                // a sector in short steps: this poll does one step of it
                if(!f->inflight) {
                    ok = f->io.read_begin(f->io.ctx, lba, dst);
                    f->inflight = ok;
                }
                if(ok) {
                    const int r = f->io.read_step(f->io.ctx);
                    if(r == 0)
                        return FLOPPY_PENDING;
                    f->inflight = false;
                    ok = r > 0;
                }
            } else {
                ok = f->io.read(f->io.ctx, lba, dst);
            }
        } else if(f->io.write_begin) {
            // a sector in short steps: this poll does one step of it (as the OP_LOAD case above)
            if(!f->inflight) {
                ok = f->io.write_begin(f->io.ctx, lba, f->op_wbuf);
                f->inflight = ok;
            }
            if(ok) {
                const int r = f->io.write_step(f->io.ctx);
                if(r == 0)
                    return FLOPPY_PENDING;
                f->inflight = false;
                ok = r > 0;
            }
        } else {
            ok = f->io.write(f->io.ctx, lba, f->op_wbuf);
        }
    }

    if(!ok) {
        f->op = OP_NONE;
        f->inflight = false;
        return FLOPPY_ERROR;
    }

    if(++f->op_done < f->op_count)
        return FLOPPY_PENDING;

    f->op = OP_NONE;
    return FLOPPY_OK;
}
