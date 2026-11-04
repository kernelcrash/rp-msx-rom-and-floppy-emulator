#pragma once

#include <stdint.h>

// The single, always-resident write-capture state machine (CONFIG_VDP_PIO SM0) that replaces
// vdp_bus.pio's vdp_bus_program plus the menu_write/fdc_write/mapper_write swappable slot:
// generic_write (bus_write.pio) captures every Z80 write cycle, memory or I/O, unconditionally,
// pushing an EARLY snapshot (address/control, data not yet valid) and then a LATE one (data
// valid) for every write -- see refactor-pio-sm.md. This file only owns the state machine itself;
// classification and dispatch live in main.c's core0_loop (write_classify_early()/the late
// dispatch), exactly where the equivalent per-feature logic already lived before this refactor
// (menu/fdc/mapper write handling has always been directly in core0_loop or folded into main.c --
// see folded-megarom.md).

enum {
    GENERIC_WRITE_SM = 0,  // on CONFIG_VDP_PIO -- vdp_bus_program's old VDP_WRITE_SM slot, repurposed
};

void bus_write_init(void);
