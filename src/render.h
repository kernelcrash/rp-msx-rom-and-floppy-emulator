#pragma once

// Define RENDER_TEST_PATTERN (cmake -DRENDER_TEST_PATTERN=ON) to draw the VGA test pattern
// instead of the TMS9918A image.

extern void render_init();
extern void render_loop();

extern void render_vga_testpattern();
