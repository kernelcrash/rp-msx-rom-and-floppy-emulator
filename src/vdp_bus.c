#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/structs/sio.h>
#include "board_config.h"
#include "rom.h"
#include "rom.pio.h"
#include "tms9918a.h"
#include "vdp_bus.h"


// The PIO programs are written in terms of these, so they must agree with board_config.h
_Static_assert(ROM_IORQ_GPIO == CONFIG_PIN_IORQ, "rom.pio _IORQ pin does not match board_config.h");
_Static_assert(ROM_RD_GPIO == CONFIG_PIN_RD, "rom.pio _RD pin does not match board_config.h");
_Static_assert(CONFIG_PIN_ADDR_BASE + 8 <= 32, "sample_port assumes A0-A7 are inside the 32 bit snapshot");

volatile uint32_t vdp_bus_captured;
volatile uint32_t vdp_bus_vdp_writes;
volatile uint32_t vdp_bus_vdp_reads;
volatile uint32_t vdp_bus_late_reads;
volatile uint8_t vdp_bus_last_value;  // TEMPORARY DIAGNOSTIC: last VDP read answer computed


// The sample layout: GPIO n = bit n (generic_read's and generic_write's shared convention -- see
// refactor-pio-sm.md), so D0-D7 are bits 8:1 and A0-A7 are bits 23:16 regardless of which capture
// program pushed the word.
static inline uint_fast16_t sample_port(uint32_t sample) {
    return (sample >> CONFIG_PIN_ADDR_BASE) & 0xff;
}

static inline bool is_vdp_port(uint_fast16_t port) {
#if V9938
    // All four V9938 ports are wired now (see RPROM/plan-v9938-base9918.md's phase 3): data,
    // control, palette, indirect-register -- address bits 1:0 select. Conditional on V9938
    // specifically: tms9918a.c's own dispatch only ever checks address bit 0, so widening this
    // mask unconditionally would alias $9A/$9B onto the existing two ports for a plain TMS9918A
    // build, where those two addresses were never meant to be reachable at all.
    return (port & 0xfc) == CONFIG_VDP_PORT;
#else
    // The VDP answers on two ports: data, then control (address bit 0 selects)
    return (port & 0xfe) == CONFIG_VDP_PORT;
#endif
}


// The chip that owns the /INT wire is open drain: a pull-up on the bus holds it high, and we pull
// it low by making the pin an output (the output value is left at 0).
void tms9918a_set_int(bool asserted) {
#if VDP_READS
    gpio_set_dir(CONFIG_PIN_INT, asserted);
#else
    (void)asserted;
#endif
}


void vdp_bus_init() {
#if VDP_READS
    // VDP reads and writes are both captured on PIO0/PIO1's generic state machines now (rom.c's
    // rom_init(), bus_write.c's bus_write_init()), along with every other Z80 bus cycle -- there is
    // nothing left to set up here for capture. /INT still needs its pin.

    // /INT: held high by a pull-up on the bus. The output value stays 0, and tms9918a_set_int()
    // switches the direction, so the pin is never driven high.
    gpio_init(CONFIG_PIN_INT);
    gpio_put(CONFIG_PIN_INT, 0);
    gpio_set_dir(CONFIG_PIN_INT, GPIO_IN);
#endif
}


// sample is generic_write's LATE 32 bit snapshot (data valid); main.c's write dispatch calls this
// once it has classified the write as an I/O cycle, for every I/O write regardless of port -- same
// division of labour fdc_bus_step/menu_bus_step already have (their own decode function rejects
// anything outside their own window).
void vdp_bus_apply_write_sample(uint32_t sample) {
#if !VDPW_NO_COUNTERS
    vdp_bus_captured++;
#endif

    const uint_fast16_t port = sample_port(sample);
    if(is_vdp_port(port)) {
#if !VDPW_NO_COUNTERS
        vdp_bus_vdp_writes++;
#endif
        tms9918a_write(port, BUS_DATA(sample));
    }
}


#if VDP_READS
// sample is generic_read's already-popped 32-bit snapshot; main.c's read_classify() has already
// confirmed this is an I/O read (_IORQ low) before calling this, but not which port -- same
// division of labour as vdp_bus_apply_write_sample() already has for writes. main.c's read
// dispatch has already drained every write still queued on generic_write's FIFO before calling
// this, so the writes the CPU made before this read are already applied.
void vdp_bus_handle_read(uint32_t sample) {
    const uint_fast16_t port = sample_port(sample);

    if(!is_vdp_port(port))
        return;

    const uint8_t value = tms9918a_read_value(port);
    vdp_bus_last_value = value;  // TEMPORARY DIAGNOSTIC

    // Only drive the bus if this read cycle is still going (_IORQ and _RD both low). If we were
    // too slow and the cycle is over, a late answer could land in the middle of a later cycle,
    // for example a ROM read, so it is dropped instead.
    const uint32_t cycle_mask = (1u << CONFIG_PIN_IORQ) | (1u << CONFIG_PIN_RD);
    if((sio_hw->gpio_in & cycle_mask) == 0) {
        // The answer goes into the data state machine first, then the drive state machine turns
        // the pins into outputs, so the pins never show an old value while they are outputs.
        pio_sm_put(CONFIG_ROM_PIO, ROM_DATA_SM, value);
        pio_sm_put(CONFIG_ROM_PIO, VDP_DRIVE_SM, 0);
        vdp_bus_vdp_reads++;
    } else {
        vdp_bus_late_reads++;
    }

    tms9918a_read_done(port);
}
#endif

#if VDP_READ_TRACK
// The real VDP answers this read; we only apply what it does to the VDP's state, as AppleII-VGA's
// abus_loop does with its v9938_read(): a VRAM read advances the address, a status read resets the
// $99 first-byte latch. Without this, every write after a VRAM read lands at the wrong address.
// main.c has already drained the writes the Z80 made before this read.
void vdp_bus_track_read(uint32_t sample) {
    const uint_fast16_t port = sample_port(sample);
    if(is_vdp_port(port))
        tms9918a_read_done(port);
}
#endif
