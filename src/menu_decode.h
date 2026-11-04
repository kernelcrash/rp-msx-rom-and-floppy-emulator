#pragma once

#include <stdbool.h>
#include <stdint.h>

// Deciding what a captured menu-window write means: a pure function on the single 32 pin snapshot
// that generic_write pushes (GPIO n = bit n), so it is tested on a PC. Pin layout: board_config.h.

#include "board_config.h"

#define MWD_MREQ (1u << CONFIG_PIN_MREQ)
#define MWD_SLTSL (1u << CONFIG_PIN_SLTSL)
#define MWD_WR (1u << CONFIG_PIN_WR)
#define MWD_A(n) (1u << (CONFIG_PIN_ADDR_BASE + (n)))

#define MWD_WINDOW_HIGH (0x7fu << (CONFIG_PIN_ADDR_BASE + 8))  // A8-A14

// Is this a write to the menu window ($8000-$80FF of this slot)? If so return true and set *offset
// (0-255, the byte within the window) and *data. _MREQ, _SLTSL and _WR must be low, A15 high (page 2,
// not menu.rom's own code in page 1), and A8-A14 all low (the 256 byte window within page 2):
// generic_write captures every write in the machine, so every check here is load-bearing.
static inline bool __attribute__((always_inline)) menu_decode_write(uint32_t s, unsigned *offset, uint8_t *data) {
    if(s & (MWD_MREQ | MWD_SLTSL | MWD_WR))
        return false;
    if(!(s & MWD_A(15)))
        return false;  // not page 2
    if(s & MWD_WINDOW_HIGH)
        return false;  // A8-A14 not all low: outside the 256 byte window
    *offset = (s >> CONFIG_PIN_ADDR_BASE) & 0xff;  // A0-A7
    *data = BUS_DATA(s);
    return true;
}
