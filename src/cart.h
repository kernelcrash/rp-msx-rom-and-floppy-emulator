#pragma once

#include <stdbool.h>
#include <stdint.h>

// The cartridge in the slot (msx-cart branch): which bytes a read of any of the slot's 64K returns, and
// what a write to a mapper's bank registers does. Pure logic on plain arrays, so it is tested on a PC
// (tests/cart_test.c); main.c and rom.c use it on the real buffers.
//
// The slot's 64K is eight 8K regions ($0000, $2000, ... $E000). bank[1 + r] points at the 8K of data
// region r shows: inside the cartridge buffer, or at an 8K block of 0xff where the cartridge has nothing.
// A read of `address` is bank[1 + (address >> 13)][address & 0x1fff]: one table lookup for every kind
// of cartridge, plain or mapper. bank[0] is never read: a mapper write that changes nothing (a write to
// a fixed or undecoded address) is aimed at it, so the write path never has to test for that case.

#define CART_MAX_BYTES (256u * 1024u)  // the largest cartridge the buffer holds (32 banks of 8K)
#define CART_REGION_BYTES 0x2000u
#define CART_BANK_SLOTS 9               // bank[0] (the write sink) + 8 regions

enum megarom_type { MEGAROM_GENERIC, MEGAROM_KONAMI4, MEGAROM_KONAMI5, MEGAROM_ASCII8, MEGAROM_ASCII16 };

typedef const uint8_t *volatile cart_bank_t;

static inline uint8_t cart_read(const cart_bank_t *bank, uint32_t address) {
    return bank[1 + (address >> 13)][address & 0x1fff];
}

// The bank mask for a cartridge of `size` bytes: bank numbers wrap at the next power of two, as on a
// real cartridge whose ROM does not decode the upper bank bits.
static inline uint32_t cart_bank_mask(uint32_t size) {
    uint32_t banks = 1;
    while(banks * CART_REGION_BYTES < size && banks * CART_REGION_BYTES < CART_MAX_BYTES)
        banks <<= 1;
    return banks - 1;
}

// Where a plain (mapper-less) cartridge goes in the slot, from its size and header:
//  * up to 16K: $4000, or $8000 when its header says it runs from page 2 (INIT, or for a BASIC cartridge
//    TEXT, in $8000-$BFFF)
//  * up to 32K: $4000 ($4000-$BFFF)
//  * up to 48K with its header at the start: $4000 ($4000-$FFFF)
//  * anything else up to 64K: $0000 (it fills the slot from the bottom)
static inline uint32_t cart_plain_base(const uint8_t *rom, uint32_t size) {
    if(size > 0x8000)
        return (size <= 0xc000 && rom[0] == 'A' && rom[1] == 'B') ? 0x4000 : 0x0000;
    if(size <= 0x4000 && rom[0] == 'A' && rom[1] == 'B') {
        const uint32_t init = rom[2] | (uint32_t)rom[3] << 8;
        const uint32_t text = rom[8] | (uint32_t)rom[9] << 8;
        const uint32_t start = init ? init : text;
        if(start >= 0x8000 && start < 0xc000)
            return 0x8000;
    }
    return 0x4000;
}

// Set up bank[] for a cartridge of `size` bytes at rom[] (padded with 0xff to a whole number of 8K
// regions), as `type` starts out after a reset. ff is an 8K block of 0xff.
static inline void cart_set_banks(cart_bank_t *bank, const uint8_t *rom, uint32_t size, enum megarom_type type,
    const uint8_t *ff) {
    bank[0] = ff;
    for(unsigned r = 0; r < 8; r++)
        bank[1 + r] = ff;
    switch(type) {
    case MEGAROM_GENERIC: {
        const uint32_t base = cart_plain_base(rom, size);
        for(unsigned r = 0; r < 8; r++) {
            const uint32_t start = r * CART_REGION_BYTES;
            if(start >= base && start - base < size)
                bank[1 + r] = rom + (start - base);
        }
        break;
    }
    case MEGAROM_KONAMI4:
    case MEGAROM_KONAMI5:
        // banks 0-3 in $4000-$BFFF (openMSX's reset state for both)
        for(unsigned r = 2; r < 6; r++)
            bank[1 + r] = rom + ((r - 2) & cart_bank_mask(size)) * CART_REGION_BYTES;
        break;
    case MEGAROM_ASCII8:
    case MEGAROM_ASCII16:
        // bank 0 everywhere in $4000-$BFFF (ASCII16: 16K bank 0 in both pages)
        for(unsigned r = 2; r < 6; r++)
            bank[1 + r] = rom + ((type == MEGAROM_ASCII16 ? (r & 1) : 0) * CART_REGION_BYTES);
        break;
    }
}

// Which bank[] slot a write to `address` sets, per type, indexed by address >> 11 (2K steps over the
// 64K). 0 = not a bank register (the write sink). The value is 1 + the 8K region it switches; CART_PAIR
// means a 16K bank (ASCII16): the region and the next one get the two halves.
//  Konami4:  $6000-$7FFF, $8000-$9FFF, $A000-$BFFF switch their own region; $4000-$5FFF is fixed.
//  Konami5:  $5000-$57FF, $7000-$77FF, $9000-$97FF, $B000-$B7FF switch $4000, $6000, $8000, $A000 (no SCC).
//  ASCII8:   $6000, $6800, $7000, $7800 (2K each) switch $4000, $6000, $8000, $A000.
//  ASCII16:  $6000-$67FF switches $4000-$7FFF, $7000-$77FF switches $8000-$BFFF.
#define CART_PAIR 0x10
static const uint8_t cart_write_target[5][32] = {
    [MEGAROM_GENERIC] = { 0 },
    [MEGAROM_KONAMI4] = { [12 ... 15] = 1 + 3, [16 ... 19] = 1 + 4, [20 ... 23] = 1 + 5 },
    [MEGAROM_KONAMI5] = { [10] = 1 + 2, [14] = 1 + 3, [18] = 1 + 4, [22] = 1 + 5 },
    [MEGAROM_ASCII8] = { [12] = 1 + 2, [13] = 1 + 3, [14] = 1 + 4, [15] = 1 + 5 },
    [MEGAROM_ASCII16] = { [12] = (1 + 2) | CART_PAIR, [14] = (1 + 4) | CART_PAIR },
};

// The slot's 64K as one flat image (offset = Z80 address): the disk ROM and the floppy controller's
// registers, which the controller stages into all four 16K pages
static inline void cart_set_flat(cart_bank_t *bank, const uint8_t *rom) {
    bank[0] = rom;  // never read
    for(unsigned r = 0; r < 8; r++)
        bank[1 + r] = rom + r * CART_REGION_BYTES;
}

// A mapper write is decoded in two halves, because the address is readable well before the data is
// valid (see bus_write.pio): cart_stage_write works out which slot from the address, cart_finish_write
// stores the bank once the data byte is there. The staged value packs the cart_write_target entry with
// the data shift (bit 5: ASCII16's data is a 16K bank number, so it is doubled into 8K units).
static inline uint32_t cart_stage_write(enum megarom_type type, uint32_t address) {
    const uint32_t t = cart_write_target[type][(address >> 11) & 0x1f];
    return t | (type == MEGAROM_ASCII16 ? 0x20u : 0u);
}

static inline void cart_finish_write(cart_bank_t *bank, const uint8_t *rom, uint32_t mask, uint32_t staged,
    uint8_t data) {
    const uint32_t n = ((uint32_t)data << ((staged >> 5) & 1)) & mask;
    const uint8_t *p = rom + n * CART_REGION_BYTES;
    const uint32_t slot = staged & 0xf;
    bank[slot] = p;
    if(staged & CART_PAIR)
        bank[slot + 1] = p + CART_REGION_BYTES;
}

// Guess the mapper of a cartridge from its code, for a file whose name does not say (openMSX's
// heuristic): count the LD (nnnn),A instructions that store to each type's bank registers. Up to 32K
// (and up to 64K when nothing points at a mapper) is a plain ROM.
static inline enum megarom_type cart_guess_type(const uint8_t *rom, uint32_t size) {
    if(size <= 0x8000)
        return MEGAROM_GENERIC;
    unsigned k4 = 0, k5 = 0, a8 = 0, a16 = 0;
    for(uint32_t i = 0; i + 2 < size; i++) {
        if(rom[i] != 0x32)
            continue;
        switch(rom[i + 1] | (uint32_t)rom[i + 2] << 8) {
        case 0x5000: case 0x9000: case 0xb000: k5++; break;
        case 0x4000: case 0x8000: case 0xa000: k4++; break;
        case 0x6800: case 0x7800: a8++; break;
        case 0x6000: k4++; a8++; a16++; break;
        case 0x7000: k5++; a8++; a16++; break;
        case 0x77ff: a16++; break;
        }
    }
    if(a8)
        a8--;  // as openMSX: on a tie, the others win over ASCII8
    if(!k4 && !k5 && !a8 && !a16)
        return size <= 0x10000 ? MEGAROM_GENERIC : MEGAROM_ASCII8;
    enum megarom_type best = MEGAROM_ASCII8;
    unsigned n = a8;
    if(a16 > n) { best = MEGAROM_ASCII16; n = a16; }
    if(k5 > n) { best = MEGAROM_KONAMI5; n = k5; }
    if(k4 > n) { best = MEGAROM_KONAMI4; n = k4; }
    return best;
}
