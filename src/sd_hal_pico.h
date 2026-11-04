#pragma once

#include "sd_card.h"

// Bind the SD card driver to SPI1 on GPIO 44/46/47 with GPIO 45 as chip select. The card is only
// used while the ROM image is loaded at boot: init before, deinit after.
const struct sd_hal *sd_hal_pico_init();

// Stop the SPI block and give the pins back (all inputs, no pulls), so nothing is left running or
// driving after loading.
void sd_hal_pico_deinit();
