#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_config.h"

// Deciding what a captured controller write means: a pure function on the single 32 pin snapshot that
// generic_write pushes (GPIO n = bit n), so it is tested on a PC. Pin layout: board_config.h.

#define FDCD_MREQ (1u << CONFIG_PIN_MREQ)
#define FDCD_SLTSL (1u << CONFIG_PIN_SLTSL)
#define FDCD_A(n) (1u << (CONFIG_PIN_ADDR_BASE + (n)))
#define FDCD_IORQ (1u << CONFIG_PIN_IORQ)
#define FDCD_WR (1u << CONFIG_PIN_WR)
#define FDCD_RD (1u << CONFIG_PIN_RD)

#define FDCD_WINDOW (0x7ffu << (CONFIG_PIN_ADDR_BASE + 3))   // A3..A13

// Is this a write to the controller? If so return true and set *reg (0-7) and *data.
//  - _MREQ, _SLTSL and _WR must all be low (a memory write to this slot, still in progress)
//  - A3..A13 all ones (A14 and A15 are free: the four mirrors, $3FF8, $7FF8, $BFF8, $FFF8)
// generic_write captures every write in the machine, so every check here is load-bearing.
static inline bool __attribute__((always_inline)) fdc_decode_write(uint32_t s, unsigned *reg, uint8_t *data) {
    if(s & (FDCD_MREQ | FDCD_SLTSL | FDCD_WR))
        return false;  // not a memory write to this slot
    if((s & FDCD_WINDOW) != FDCD_WINDOW)
        return false;  // not the last 8 bytes of a 16 KB page
    *reg = (s >> CONFIG_PIN_ADDR_BASE) & 7;  // A0-A2
    *data = BUS_DATA(s);
    return true;
}
