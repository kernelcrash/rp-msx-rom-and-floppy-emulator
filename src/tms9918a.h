#pragma once

#include <stdint.h>
#include <stdbool.h>


// Bus core (core 0): the programmer visible state and everything that touches it is only ever
// used from one core, in bus order. The render core only reads VRAM and the registers, and
// reports frame/sprite events with the calls marked "render core".

extern void tms9918a_init();

// Set the backdrop colour (register 7) directly. The display is blanked after init, so the whole
// picture area shows this colour until the machine initialises the VDP.
extern void tms9918a_set_backdrop(uint8_t colour);

// address bit 0 selects data (0) or control/register (1), as on the MSX port pair 0x98/0x99
extern void tms9918a_write(uint_fast16_t address, uint8_t data);
extern uint8_t tms9918a_read(uint_fast16_t address);

// A read in two steps: the value to put on the bus, then the side effects (address increment,
// read-ahead, clearing status flags, updating /INT). Call them in that order for every read.
extern uint8_t tms9918a_read_value(uint_fast16_t address);
extern void tms9918a_read_done(uint_fast16_t address);

// Call regularly: raises /INT when a new frame has ended
extern void tms9918a_service();

// Implemented by the bus code: drive (true) or release (false) the open drain /INT output
extern void tms9918a_set_int(bool asserted);

extern uint8_t tms9918a_vram_read();
extern void tms9918a_vram_write(uint8_t data);
extern uint8_t tms9918a_register_read();
extern void tms9918a_register_write(uint8_t data);
extern void tms9918a_change_register(uint8_t reg, uint8_t val);
extern void tms9918a_update_table_masks();
extern void tms9918a_update_backdrop();

// Render core: draw one frame, and report the end of the active display
extern void render_tms9918a();
extern void tms9918a_frame_event();
