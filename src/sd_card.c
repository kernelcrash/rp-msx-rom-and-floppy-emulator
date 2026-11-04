#include "sd_card.h"

#define INIT_HZ 400000

// Timeouts, in milliseconds
#define READY_TIMEOUT_MS 500   // waiting for a busy card to let go of the bus
#define INIT_TIMEOUT_MS 1000   // waiting for ACMD41 to finish (the spec allows 1 s)
#define TOKEN_TIMEOUT_MS 200   // waiting for the data token after a read command

#define WRITE_BUSY_TIMEOUT_MS 500  // the spec allows a card up to 250 ms to program a block

#define READ_ATTEMPTS 3
#define WRITE_ATTEMPTS 3

// After the supply is up a card needs at least 1 ms and 74 clocks before it takes commands. Give it
// about 20 ms of clocks (1000 bytes at 400 kHz), because the card may only just have powered up
// alongside the microcontroller, and try the reset many times: some cards need several.
#define POWER_UP_BYTES 1000
#define CMD0_ATTEMPTS 50

// Commands
#define CMD0 0     // GO_IDLE_STATE
#define CMD8 8     // SEND_IF_COND
#define CMD13 13   // SEND_STATUS
#define CMD16 16   // SET_BLOCKLEN
#define CMD17 17   // READ_SINGLE_BLOCK
#define CMD24 24   // WRITE_BLOCK
#define CMD41 41   // SD_SEND_OP_COND (application command)
#define CMD55 55   // APP_CMD
#define CMD58 58   // READ_OCR

// The two commands whose CRC the card checks in SPI mode. Every other command gets a dummy CRC
// with the end bit set.
#define CRC_CMD0 0x95
#define CRC_CMD8 0x87
#define CRC_DUMMY 0x01

#define R1_IDLE 0x01
#define R1_ILLEGAL_COMMAND 0x04

#define TOKEN_DATA 0xfe

// The card's answer to a data block: xxx0sss1 with sss = 010 accepted, 101 CRC error, 110 write error
#define DATA_RESPONSE_MASK 0x1f
#define DATA_ACCEPTED 0x05


static inline uint8_t xfer(struct sd_card *c, uint8_t tx) {
    return c->hal->xfer(c->hal->ctx, tx);
}

static inline uint32_t millis(struct sd_card *c) {
    return c->hal->millis(c->hal->ctx);
}

// Let go of the card. The extra byte gives it clocks to release its output.
static void deselect(struct sd_card *c) {
    c->hal->select(c->hal->ctx, false);
    xfer(c, 0xff);
}

// A busy card holds its output low. Wait until it reads back as 0xff.
static bool wait_ready(struct sd_card *c, uint32_t timeout_ms) {
    const uint32_t start = millis(c);
    do {
        if(xfer(c, 0xff) == 0xff)
            return true;
    } while(millis(c) - start < timeout_ms);
    return false;
}

// Send a command and return its R1 response (0xff if there was none). The card stays selected so
// the caller can read the rest of the response or the data; call deselect() afterwards.
static uint8_t send_command(struct sd_card *c, uint8_t cmd, uint32_t arg, uint8_t crc) {
    c->hal->select(c->hal->ctx, true);
    c->busy_timeout = !wait_ready(c, READY_TIMEOUT_MS);
    if(c->busy_timeout)
        return 0xff;

    xfer(c, 0x40 | cmd);
    xfer(c, arg >> 24);
    xfer(c, arg >> 16);
    xfer(c, arg >> 8);
    xfer(c, arg);
    xfer(c, crc);

    // The response comes within 8 bytes; the top bit of R1 is always 0
    for(int i = 0; i < 10; i++) {
        const uint8_t r = xfer(c, 0xff);
        if(!(r & 0x80))
            return r;
    }
    return 0xff;
}

// Read the four bytes that follow an R3/R7 response
static void read_r37(struct sd_card *c, uint8_t out[4]) {
    for(int i = 0; i < 4; i++)
        out[i] = xfer(c, 0xff);
}

static enum sd_result r1_error(uint8_t r1) {
    return r1 == 0xff ? SD_ERR_NO_CARD : SD_ERR_CMD;
}


enum sd_result sd_init(struct sd_card *c, const struct sd_hal *hal, uint32_t run_hz) {
    c->hal = hal;
    c->block_addressing = false;
    c->busy_timeout = false;

    hal->set_speed(hal->ctx, INIT_HZ);

    // Clocks with the card deselected and MOSI high (at least 74) put it into SPI-ready state, and
    // this many of them also give a card that has only just powered up time to settle
    hal->select(hal->ctx, false);
    for(int i = 0; i < POWER_UP_BYTES; i++)
        xfer(c, 0xff);

    // CMD0: software reset into the idle state, which also selects SPI mode
    uint8_t r1 = 0xff;
    int stuck = 0;
    for(int i = 0; i < CMD0_ATTEMPTS; i++) {
        r1 = send_command(c, CMD0, 0, CRC_CMD0);
        deselect(c);
        if(r1 == R1_IDLE)
            break;

        // MISO held low for a long time, twice in a row: no point trying 50 times
        stuck = c->busy_timeout ? stuck + 1 : 0;
        if(stuck >= 2)
            return SD_ERR_BUS_LOW;

        xfer(c, 0xff);  // a couple of idle clocks between attempts
        xfer(c, 0xff);
    }
    if(r1 != R1_IDLE)
        return r1_error(r1);

    // CMD8: voltage check. SD v2 cards answer it; v1 cards call it an illegal command.
    bool v2 = false;
    r1 = send_command(c, CMD8, 0x1aa, CRC_CMD8);
    if(r1 == R1_IDLE) {
        uint8_t r7[4];
        read_r37(c, r7);
        deselect(c);
        if(r7[2] != 0x01 || r7[3] != 0xaa)  // voltage range not accepted, or the check pattern was not echoed
            return SD_ERR_UNSUPPORTED;
        v2 = true;
    } else if(r1 & R1_ILLEGAL_COMMAND) {
        deselect(c);
    } else {
        deselect(c);
        return r1_error(r1);
    }

    // ACMD41 until the card is ready. HCS tells a v2 card that we understand block addressing.
    const uint32_t start = millis(c);
    for(;;) {
        r1 = send_command(c, CMD55, 0, CRC_DUMMY);
        deselect(c);
        if(r1 & ~R1_IDLE)  // an MMC card does not know CMD55
            return r1 == 0xff ? SD_ERR_NO_CARD : SD_ERR_UNSUPPORTED;

        r1 = send_command(c, CMD41, v2 ? 0x40000000 : 0, CRC_DUMMY);
        deselect(c);
        if(r1 == 0x00)
            break;
        if(r1 != R1_IDLE)
            return r1 == 0xff ? SD_ERR_NO_CARD : SD_ERR_UNSUPPORTED;

        if(millis(c) - start >= INIT_TIMEOUT_MS)
            return SD_ERR_INIT_TIMEOUT;
    }

    // CMD58: the OCR tells whether this is a high capacity card (block addressing)
    if(v2) {
        r1 = send_command(c, CMD58, 0, CRC_DUMMY);
        if(r1 != 0x00) {
            deselect(c);
            return r1_error(r1);
        }
        uint8_t ocr[4];
        read_r37(c, ocr);
        deselect(c);
        c->block_addressing = (ocr[0] & 0x40) != 0;  // CCS
    }

    // A byte addressed card needs the block length set (high capacity cards are fixed at 512)
    if(!c->block_addressing) {
        r1 = send_command(c, CMD16, 512, CRC_DUMMY);
        deselect(c);
        if(r1 != 0x00)
            return r1_error(r1);
    }

    hal->set_speed(hal->ctx, run_hz);
    return SD_OK;
}


// The CRC the card sends after every data block (CRC-16/XMODEM: polynomial 0x1021, starting at 0, most
// significant byte first). The card does not check CRCs in SPI mode, but it always sends this one, so a sector
// damaged on the wires is caught here and read again instead of being passed on.
static uint16_t crc16(const uint8_t *p, unsigned n) {
    static uint16_t table[256];
    if(!table[1]) {
        for(unsigned i = 0; i < 256; i++) {
            uint16_t c = (uint16_t)(i << 8);
            for(int b = 0; b < 8; b++)
                c = (uint16_t)((c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1);
            table[i] = c;
        }
    }
    uint16_t crc = 0;
    while(n--)
        crc = (uint16_t)((crc << 8) ^ table[(crc >> 8) ^ *p++]);
    return crc;
}

// -DSD_READ_CRC=0 builds without the check, as before 3aa102f (for comparing on the rig)
#ifndef SD_READ_CRC
#define SD_READ_CRC 1
#endif
static bool crc_ok(const uint8_t *buf, const uint8_t crc[2]) {
#if SD_READ_CRC
    return crc16(buf, 512) == (uint16_t)(crc[0] << 8 | crc[1]);
#else
    (void)buf;
    (void)crc;
    return true;
#endif
}

// The states of a read
enum {
    SR_START,   // (re)start an attempt: select the card
    SR_READY,   // wait for a busy card to release MISO
    SR_CMD,     // send CMD17 and read its R1
    SR_TOKEN,   // wait for the data token
    SR_DATA,    // read the 512 bytes
    SR_CRC,     // the two CRC bytes, then done
    SR_DONE,
};

void sd_read_begin(struct sd_card *c, struct sd_read *op, uint32_t lba, uint8_t *buf) {
    op->buf = buf;
    op->attempt = 0;
    op->got = 0;
    op->result = SD_ERR_CMD;
    op->address = lba;
    op->state = SR_START;
    if(!c->block_addressing) {
        if(lba >= 0xffffffffu / 512) {  // the byte address would not fit
            op->result = SD_ERR_RANGE;
            op->state = SR_DONE;
            return;
        }
        op->address = lba * 512;
    }
}

// An attempt failed with `why`: try again, or give up after READ_ATTEMPTS
static enum sd_result read_failed(struct sd_card *c, struct sd_read *op, enum sd_result why) {
    deselect(c);
    op->result = why;
    if(++op->attempt < READ_ATTEMPTS) {
        op->state = SR_START;
        return SD_PENDING;
    }
    op->state = SR_DONE;
    return why;
}

enum sd_result sd_read_step(struct sd_card *c, struct sd_read *op) {
    switch(op->state) {
        case SR_START:
            c->hal->select(c->hal->ctx, true);
            op->start_ms = millis(c);
            op->state = SR_READY;
            return SD_PENDING;

        case SR_READY:
            // A busy card holds its output low; up to SD_STEP_BYTES clocks per step
            for(int i = 0; i < SD_STEP_BYTES; i++) {
                if(xfer(c, 0xff) == 0xff) {
                    op->state = SR_CMD;
                    return SD_PENDING;
                }
            }
            if(millis(c) - op->start_ms >= READY_TIMEOUT_MS)
                return read_failed(c, op, SD_ERR_NO_CARD);
            return SD_PENDING;

        case SR_CMD: {
            // 6 bytes of command and up to 10 for the response: 16 in all
            xfer(c, 0x40 | CMD17);
            xfer(c, op->address >> 24);
            xfer(c, op->address >> 16);
            xfer(c, op->address >> 8);
            xfer(c, op->address);
            xfer(c, CRC_DUMMY);
            uint8_t r1 = 0xff;
            for(int i = 0; i < 10; i++) {
                const uint8_t r = xfer(c, 0xff);
                if(!(r & 0x80)) {
                    r1 = r;
                    break;
                }
            }
            if(r1 != 0x00)
                return read_failed(c, op, r1_error(r1));
            op->start_ms = millis(c);
            op->state = SR_TOKEN;
            return SD_PENDING;
        }

        case SR_TOKEN: {
            // The data token comes after a variable number of idle bytes
            uint8_t token = 0xff;
            for(int i = 0; i < SD_STEP_BYTES && token == 0xff; i++)
                token = xfer(c, 0xff);
            if(token == TOKEN_DATA) {
                op->got = 0;
                op->state = SR_DATA;
                return SD_PENDING;
            }
            if(token != 0xff)
                return read_failed(c, op, SD_ERR_READ_TOKEN);
            if(millis(c) - op->start_ms >= TOKEN_TIMEOUT_MS)
                return read_failed(c, op, SD_ERR_READ_TIMEOUT);
            return SD_PENDING;
        }

        case SR_DATA: {
            unsigned n = 512 - op->got;
            if(n > SD_STEP_BYTES)
                n = SD_STEP_BYTES;
            c->hal->read(c->hal->ctx, op->buf + op->got, n);
            op->got += n;
            if(op->got >= 512)
                op->state = SR_CRC;
            return SD_PENDING;
        }

        case SR_CRC: {
            uint8_t crc[2];
            c->hal->read(c->hal->ctx, crc, 2);
            if(!crc_ok(op->buf, crc))
                return read_failed(c, op, SD_ERR_READ_CRC);
            deselect(c);
            op->result = SD_OK;
            op->state = SR_DONE;
            return SD_OK;
        }

        default:
            return op->result;
    }
}

// The original blocking read, kept as it was: the ROM loader at boot uses it and the confirmed firmware must not
// change. sd_read_begin/sd_read_step above do the same thing in short steps (tests check the two agree).
enum sd_result sd_read_sector(struct sd_card *c, uint32_t lba, uint8_t *buf) {
    uint32_t address = lba;
    if(!c->block_addressing) {
        if(lba >= 0xffffffffu / 512)  // the byte address would not fit
            return SD_ERR_RANGE;
        address = lba * 512;
    }

    enum sd_result result = SD_ERR_CMD;

    for(int attempt = 0; attempt < READ_ATTEMPTS; attempt++) {
        const uint8_t r1 = send_command(c, CMD17, address, CRC_DUMMY);
        if(r1 != 0x00) {
            deselect(c);
            result = r1_error(r1);
            continue;
        }

        // The data token comes after a variable number of idle bytes
        uint8_t token;
        const uint32_t start = millis(c);
        do {
            token = xfer(c, 0xff);
        } while(token == 0xff && millis(c) - start < TOKEN_TIMEOUT_MS);

        if(token != TOKEN_DATA) {
            deselect(c);
            result = token == 0xff ? SD_ERR_READ_TIMEOUT : SD_ERR_READ_TOKEN;
            continue;
        }

        c->hal->read(c->hal->ctx, buf, 512);

        uint8_t crc[2];
        c->hal->read(c->hal->ctx, crc, 2);

        deselect(c);
        if(!crc_ok(buf, crc)) {
            result = SD_ERR_READ_CRC;
            continue;
        }
        return SD_OK;
    }

    return result;
}


// The states of a write
enum {
    SW_START,   // (re)start an attempt: select the card
    SW_READY,   // wait for a busy card to release MISO
    SW_CMD,     // send CMD24 and read its R1
    SW_TOKEN,   // one idle byte, then the start token
    SW_DATA,    // send the 512 bytes
    SW_CRC,     // the two dummy CRC bytes, then the data response
    SW_BUSY,    // wait for the card to finish programming
    SW_STATUS_SELECT,  // re-select the card for CMD13 (sd_write_sector deselects first)
    SW_STATUS_READY,   // a reselect can itself find the card busy again: wait for it, chunked
    SW_STATUS_CMD,     // send CMD13, read its R1, then the status byte, then done
    SW_DONE,
};

void sd_write_begin(struct sd_card *c, struct sd_write *op, uint32_t lba, const uint8_t *buf) {
    op->buf = buf;
    op->attempt = 0;
    op->sent = 0;
    op->result = SD_ERR_CMD;
    op->address = lba;
    op->state = SW_START;
    if(!c->block_addressing) {
        if(lba >= 0xffffffffu / 512) {  // the byte address would not fit
            op->result = SD_ERR_RANGE;
            op->state = SW_DONE;
            return;
        }
        op->address = lba * 512;
    }
}

// An attempt failed with `why`: try again, or give up after WRITE_ATTEMPTS (mirrors read_failed)
static enum sd_result write_failed(struct sd_card *c, struct sd_write *op, enum sd_result why) {
    deselect(c);
    op->result = why;
    if(++op->attempt < WRITE_ATTEMPTS) {
        op->state = SW_START;
        return SD_PENDING;
    }
    op->state = SW_DONE;
    return why;
}

enum sd_result sd_write_step(struct sd_card *c, struct sd_write *op) {
    switch(op->state) {
        case SW_START:
            c->hal->select(c->hal->ctx, true);
            op->start_ms = millis(c);
            op->state = SW_READY;
            return SD_PENDING;

        case SW_READY:
            // A busy card holds its output low; up to SD_STEP_BYTES clocks per step
            for(int i = 0; i < SD_STEP_BYTES; i++) {
                if(xfer(c, 0xff) == 0xff) {
                    op->state = SW_CMD;
                    return SD_PENDING;
                }
            }
            if(millis(c) - op->start_ms >= READY_TIMEOUT_MS)
                return write_failed(c, op, SD_ERR_NO_CARD);
            return SD_PENDING;

        case SW_CMD: {
            // 6 bytes of command and up to 10 for the response: 16 in all, as SR_CMD
            xfer(c, 0x40 | CMD24);
            xfer(c, op->address >> 24);
            xfer(c, op->address >> 16);
            xfer(c, op->address >> 8);
            xfer(c, op->address);
            xfer(c, CRC_DUMMY);
            uint8_t r1 = 0xff;
            for(int i = 0; i < 10; i++) {
                const uint8_t r = xfer(c, 0xff);
                if(!(r & 0x80)) {
                    r1 = r;
                    break;
                }
            }
            if(r1 != 0x00)
                return write_failed(c, op, r1_error(r1));
            op->state = SW_TOKEN;
            return SD_PENDING;
        }

        case SW_TOKEN:
            xfer(c, 0xff);
            xfer(c, TOKEN_DATA);
            op->sent = 0;
            op->state = SW_DATA;
            return SD_PENDING;

        case SW_DATA: {
            unsigned n = 512 - op->sent;
            if(n > SD_STEP_BYTES)
                n = SD_STEP_BYTES;
            c->hal->write(c->hal->ctx, op->buf + op->sent, n);
            op->sent += n;
            if(op->sent >= 512)
                op->state = SW_CRC;
            return SD_PENDING;
        }

        case SW_CRC: {
            xfer(c, 0xff);  // two dummy CRC bytes: not checked, CRC checking is off in SPI mode
            xfer(c, 0xff);
            const uint8_t resp = xfer(c, 0xff);
            if((resp & DATA_RESPONSE_MASK) != DATA_ACCEPTED)
                return write_failed(c, op, SD_ERR_WRITE_REJECTED);
            op->start_ms = millis(c);
            op->state = SW_BUSY;
            return SD_PENDING;
        }

        case SW_BUSY:
            // As SW_READY: the card holds MISO low while it programs the block. Not retried on
            // timeout (nor is the blocking sd_write_sector's equivalent wait): a card this slow is
            // reported, not tried again with the same data three times over.
            for(int i = 0; i < SD_STEP_BYTES; i++) {
                if(xfer(c, 0xff) == 0xff) {
                    deselect(c);
                    op->state = SW_STATUS_SELECT;
                    return SD_PENDING;
                }
            }
            if(millis(c) - op->start_ms >= WRITE_BUSY_TIMEOUT_MS) {
                deselect(c);
                op->result = SD_ERR_WRITE_TIMEOUT;
                op->state = SW_DONE;
                return SD_ERR_WRITE_TIMEOUT;
            }
            return SD_PENDING;

        // CMD13 returns an R2: the card's status, of which the second byte is non-zero after a
        // write failure or an attempt to write a protected block. sd_write_sector deselects before
        // this (the card's own busy-wait for programming is already over), but reselecting can
        // itself find the card busy again, so this gets the same SW_START/SW_READY/SW_CMD
        // treatment as the write itself, not one unchunked send_command() call.
        case SW_STATUS_SELECT:
            c->hal->select(c->hal->ctx, true);
            op->start_ms = millis(c);
            op->state = SW_STATUS_READY;
            return SD_PENDING;

        case SW_STATUS_READY:
            for(int i = 0; i < SD_STEP_BYTES; i++) {
                if(xfer(c, 0xff) == 0xff) {
                    op->state = SW_STATUS_CMD;
                    return SD_PENDING;
                }
            }
            if(millis(c) - op->start_ms >= READY_TIMEOUT_MS)
                return write_failed(c, op, SD_ERR_NO_CARD);
            return SD_PENDING;

        case SW_STATUS_CMD: {
            xfer(c, 0x40 | CMD13);
            xfer(c, 0);
            xfer(c, 0);
            xfer(c, 0);
            xfer(c, 0);
            xfer(c, CRC_DUMMY);
            uint8_t s1 = 0xff;
            for(int i = 0; i < 10; i++) {
                const uint8_t r = xfer(c, 0xff);
                if(!(r & 0x80)) {
                    s1 = r;
                    break;
                }
            }
            const uint8_t s2 = xfer(c, 0xff);
            deselect(c);
            if(s1 & ~R1_IDLE) {
                op->result = r1_error(s1);
                op->state = SW_DONE;
                return op->result;
            }
            if(s2 != 0x00)
                return write_failed(c, op, SD_ERR_WRITE_STATUS);
            op->result = SD_OK;
            op->state = SW_DONE;
            return SD_OK;
        }

        default:
            return op->result;
    }
}

// The original blocking write, kept as it was (see sd_read_sector's own comment: nothing that
// works today should change). sd_write_begin/sd_write_step above do the same thing in short steps
// (tests check the two agree), for a caller -- the render core -- that cannot block through the
// up to ~250 ms a card is allowed to take programming a block.
enum sd_result sd_write_sector(struct sd_card *c, uint32_t lba, const uint8_t *buf) {
    uint32_t address = lba;
    if(!c->block_addressing) {
        if(lba >= 0xffffffffu / 512)
            return SD_ERR_RANGE;
        address = lba * 512;
    }

    enum sd_result result = SD_ERR_CMD;

    for(int attempt = 0; attempt < WRITE_ATTEMPTS; attempt++) {
        const uint8_t r1 = send_command(c, CMD24, address, CRC_DUMMY);
        if(r1 != 0x00) {
            deselect(c);
            result = r1_error(r1);
            continue;
        }

        // One idle byte, then the start token, the data and a dummy CRC (not checked in SPI mode)
        xfer(c, 0xff);
        xfer(c, TOKEN_DATA);
        c->hal->write(c->hal->ctx, buf, 512);
        xfer(c, 0xff);
        xfer(c, 0xff);

        // The data response comes straight after the block
        const uint8_t resp = xfer(c, 0xff);
        if((resp & DATA_RESPONSE_MASK) != DATA_ACCEPTED) {
            deselect(c);
            result = SD_ERR_WRITE_REJECTED;
            continue;
        }

        // The card holds MISO low while it programs the block
        if(!wait_ready(c, WRITE_BUSY_TIMEOUT_MS)) {
            deselect(c);
            return SD_ERR_WRITE_TIMEOUT;
        }
        deselect(c);

        // CMD13 returns an R2: the card's status, of which the second byte is non-zero after a
        // write failure or an attempt to write a protected block
        const uint8_t s1 = send_command(c, CMD13, 0, CRC_DUMMY);
        const uint8_t s2 = xfer(c, 0xff);
        deselect(c);
        if(s1 & ~R1_IDLE)
            return r1_error(s1);
        if(s2 != 0x00) {
            result = SD_ERR_WRITE_STATUS;
            continue;
        }
        return SD_OK;
    }

    return result;
}


const char *sd_result_name(enum sd_result r) {
    switch(r) {
        case SD_OK: return "ok";
        case SD_ERR_NO_CARD: return "no-card";
        case SD_ERR_BUS_LOW: return "bus-low";
        case SD_ERR_UNSUPPORTED: return "unsupported";
        case SD_ERR_INIT_TIMEOUT: return "init-timeout";
        case SD_ERR_CMD: return "cmd";
        case SD_ERR_READ_TIMEOUT: return "read-timeout";
        case SD_ERR_READ_TOKEN: return "read-token";
        case SD_ERR_RANGE: return "range";
        case SD_ERR_WRITE_REJECTED: return "write-rejected";
        case SD_ERR_WRITE_TIMEOUT: return "write-timeout";
        case SD_ERR_WRITE_STATUS: return "write-status";
        case SD_ERR_READ_CRC: return "read-crc";
        case SD_PENDING: return "pending";
    }
    return "?";
}
