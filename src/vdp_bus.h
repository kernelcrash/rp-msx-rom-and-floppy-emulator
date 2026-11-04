#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <hardware/pio.h>
#include "board_config.h"
#include "rom.h"
#include "tms9918a.h"

// Z80 side of the TMS9918A emulation. Everything here runs on core 0 (the bus core), which is the
// only core that touches the VDP state.
//
//  * Writes: generic_write (bus_write.pio/.c) captures every Z80 write cycle, memory or I/O, and
//    main.c's write dispatch calls vdp_bus_apply_write_sample() for I/O writes, with the LATE
//    (data valid) snapshot.
//  * Reads (VDP_READS=1 only): generic_read (rom.pio, PIO0) captures every Z80 read cycle, memory
//    or I/O, and main.c's read_classify() dispatches VDP-port reads to vdp_bus_handle_read() with
//    the already-popped snapshot, after first draining any writes still queued on generic_write's
//    FIFO so the read sees them. It also drives the open drain /INT line.
//
// With VDP_READS=0 the chip only listens (no read capture, no /INT, the data pins are never driven
// for a VDP cycle), so it can sit next to a real VDP that answers reads itself.

#ifndef VDP_READS
#define VDP_READS 0
#endif

// The CONFIG_ROM_PIO state machine a VDP read's go-token goes to: generic_read_drive, which serves
// both read kinds
#define VDP_DRIVE_SM ROM_DIRECTION_SM

void vdp_bus_init();

// Apply one write to the emulated VDP, given generic_write's LATE (data valid) 32 bit snapshot.
// Rejects anything that is not actually a VDP port itself (main.c's write dispatch calls this for
// every I/O write, not just confirmed VDP ones -- same division of labour fdc_bus_step/
// menu_bus_step already have).
void vdp_bus_apply_write_sample(uint32_t sample);

#if VDP_READS
// Handle one VDP-port read cycle, given its already-popped 32-bit generic_read snapshot. The
// caller (main.c's read_classify()) must have already drained any writes still queued on
// generic_write's FIFO, so this read sees them.
void vdp_bus_handle_read(uint32_t sample);
#endif
#if VDP_READ_TRACK
// VDP_READS off: apply a VDP-port read's side effects without answering it (same caller rules)
void vdp_bus_track_read(uint32_t sample);
#endif

// Debug counters, readable from a debugger
extern volatile uint32_t vdp_bus_captured;    // I/O writes seen, any port
extern volatile uint32_t vdp_bus_vdp_writes;  // of those, writes to the VDP ports
extern volatile uint32_t vdp_bus_vdp_reads;   // VDP reads answered
extern volatile uint32_t vdp_bus_late_reads;  // VDP reads that were too late to answer (cycle already over)
extern volatile uint8_t vdp_bus_last_value;   // TEMPORARY DIAGNOSTIC
