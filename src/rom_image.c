#include <stdint.h>
#include "board_config.h"

// The emulated ROM contents. Lives in its own SRAM region (see linker/), outside the normal
// RAM region, and is not initialised by the C runtime. It is filled from flash at boot.
__attribute__((section(".rom_image"), used))
uint32_t rom_image[CONFIG_ROM_IMAGE_SIZE / 4];
