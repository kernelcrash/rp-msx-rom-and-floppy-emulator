#include "bus_write.h"

#include <hardware/gpio.h>
#include <hardware/pio.h>
#include "board_config.h"
#include "bus_write.pio.h"

// The PIO program is written in terms of these, so they must agree with board_config.h
_Static_assert(GW_WR_GPIO == CONFIG_PIN_WR, "bus_write.pio _WR pin does not match board_config.h");
_Static_assert(GW_IORQ_GPIO == CONFIG_PIN_IORQ, "bus_write.pio _IORQ pin does not match board_config.h");

void bus_write_init(void) {
    PIO pio = CONFIG_VDP_PIO;

    // Every bus pin this program reads (it captures the full 32 bit GPIO word) needs its pad
    // isolation released; most of these are also released by rom.c's rom_init() and vdp_bus.c's
    // vdp_bus_init(), but gpio_init() is idempotent and each init function stays self-contained.
    for(uint pin = CONFIG_PIN_BUS_FIRST; pin <= CONFIG_PIN_BUS_LAST; pin++) {
        gpio_init(pin);
    }

    uint offset = pio_add_program(pio, &generic_write_program);
    pio_sm_claim(pio, GENERIC_WRITE_SM);

    pio_sm_config c = generic_write_program_get_default_config(offset);
    sm_config_set_jmp_pin(&c, CONFIG_PIN_IORQ);
    sm_config_set_in_pins(&c, 0);  // GPIO n = bit n of the pushed word
    sm_config_set_in_shift(&c, false, true, 32);  // shift left, autopush every 32 bits
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);  // nothing is sent to this SM: 8 deep RX FIFO
    pio_sm_init(pio, GENERIC_WRITE_SM, offset, &c);

    pio_sm_set_enabled(pio, GENERIC_WRITE_SM, true);
}
