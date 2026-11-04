#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Using an SD card in SPI mode: initialise it, then read and write 512 byte sectors. No
// hardware in here: the SPI bus, chip select and clock come through a small HAL, so the same code
// runs on the RP2350 and, against a mock card, in the host tests.
//
// Supports SD v1 (SDSC), SD v2 (SDSC and SDHC/SDXC). MMC cards are refused.

struct sd_hal {
    void *ctx;
    void (*select)(void *ctx, bool selected);           // chip select: true drives it low
    uint8_t (*xfer)(void *ctx, uint8_t tx);             // one byte, full duplex
    void (*read)(void *ctx, uint8_t *dst, size_t n);    // n bytes, sending 0xff
    void (*write)(void *ctx, const uint8_t *src, size_t n);  // n bytes, ignoring what comes back
    void (*set_speed)(void *ctx, uint32_t hz);          // SPI clock
    uint32_t (*millis)(void *ctx);                      // free running milliseconds
};

enum sd_result {
    SD_OK = 0,
    SD_ERR_NO_CARD,        // nothing answers (MISO idles high)
    SD_ERR_BUS_LOW,        // the card's output (MISO) is held low: a short, or a wrong or unconnected pin
    SD_ERR_UNSUPPORTED,    // an MMC card, or one with an unsupported voltage range
    SD_ERR_INIT_TIMEOUT,   // the card never left the idle state
    SD_ERR_CMD,            // a command was refused or answered unexpectedly
    SD_ERR_READ_TIMEOUT,   // no data token after a read command
    SD_ERR_READ_TOKEN,     // the card answered a read with an error token
    SD_ERR_RANGE,          // sector number out of range for this card
    SD_ERR_WRITE_REJECTED, // the card refused the data (CRC or write error response)
    SD_ERR_WRITE_TIMEOUT,  // the card stayed busy programming for too long
    SD_ERR_WRITE_STATUS,   // the card reported a write failure afterwards (CMD13)
    SD_ERR_READ_CRC,       // a sector arrived with a CRC that does not match its data (noise on the wires)
    SD_PENDING,            // (sd_read_step only) not finished: call it again
};

struct sd_card {
    const struct sd_hal *hal;
    bool block_addressing;  // SDHC/SDXC: commands take sector numbers, SDSC: byte addresses
    bool busy_timeout;      // the last command gave up waiting for the card to release MISO
};

// Bring up the card. The bus runs at 400 kHz during initialisation, then at run_hz.
enum sd_result sd_init(struct sd_card *card, const struct sd_hal *hal, uint32_t run_hz);

// Reading a sector in SHORT STEPS, for a caller that must not be held up (the render core: a stall of more
// than about 250 us loses the VGA picture). sd_read_begin() starts a read; each sd_read_step() then clocks
// at most SD_STEP_BYTES bytes over the bus (16 us at 8 MHz) and returns SD_PENDING until the sector is in
// `buf` (SD_OK) or the read has failed (an error, after the same retries as sd_read_sector). Nothing else may
// use the card between begin and the last step.
#define SD_STEP_BYTES 16

struct sd_read {
    uint8_t state;
    uint8_t attempt;
    uint16_t got;
    uint32_t address;
    uint32_t start_ms;
    uint8_t *buf;
    enum sd_result result;
};

void sd_read_begin(struct sd_card *card, struct sd_read *op, uint32_t lba, uint8_t *buf);
enum sd_result sd_read_step(struct sd_card *card, struct sd_read *op);

// Read one 512 byte sector (blocking: the steps above run back to back)
enum sd_result sd_read_sector(struct sd_card *card, uint32_t lba, uint8_t *buf);

// Write one 512 byte sector and wait until the card has finished programming it. A rejected
// write is retried (writing the same data again is harmless); after the last attempt the error is
// reported. Nothing but the one sector is touched.
enum sd_result sd_write_sector(struct sd_card *card, uint32_t lba, const uint8_t *buf);

// Writing a sector in SHORT STEPS, for the same reason and with the same shape as sd_read_begin/
// sd_read_step: a card is allowed up to about 250 ms to program a block (WRITE_BUSY_TIMEOUT_MS in
// sd_card.c), which sd_write_sector simply blocks through -- unusable from the render core. Each
// sd_write_step clocks at most SD_STEP_BYTES bytes (or, while the card is busy programming,
// polls once) and returns SD_PENDING until the write has finished (SD_OK) or failed (an error,
// after the same retries as sd_write_sector). Nothing else may use the card between begin and the
// last step. `buf` must stay valid until then.
struct sd_write {
    uint8_t state;
    uint8_t attempt;
    uint16_t sent;
    uint32_t address;
    uint32_t start_ms;
    const uint8_t *buf;
    enum sd_result result;
};

void sd_write_begin(struct sd_card *card, struct sd_write *op, uint32_t lba, const uint8_t *buf);
enum sd_result sd_write_step(struct sd_card *card, struct sd_write *op);

const char *sd_result_name(enum sd_result r);
