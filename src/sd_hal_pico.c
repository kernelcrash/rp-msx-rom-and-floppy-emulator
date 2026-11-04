#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <pico/stdlib.h>
#include "board_config.h"
#include "sd_hal_pico.h"

// GPIO 40-47 belong to SPI1; within each group of four the functions are RX, CSn, SCK, TX. The
// card's chip select is driven as a plain GPIO so it stays low for a whole command.
_Static_assert(((CONFIG_PIN_SD_MISO / 8) & 1) == 1 && ((CONFIG_PIN_SD_SCK / 8) & 1) == 1 && ((CONFIG_PIN_SD_MOSI / 8) & 1) == 1,
    "SD pins must be on SPI1 (GPIO 8-15, 24-31 or 40-47)");
_Static_assert(CONFIG_PIN_SD_MISO % 4 == 0, "MISO must be an SPI RX pin");
_Static_assert(CONFIG_PIN_SD_SCK % 4 == 2, "SCK must be an SPI SCK pin");
_Static_assert(CONFIG_PIN_SD_MOSI % 4 == 3, "MOSI must be an SPI TX pin");

#define SD_SPI spi1


static void hal_select(void *ctx, bool selected) {
    (void)ctx;
    gpio_put(CONFIG_PIN_SD_CS, !selected);
}

static uint8_t hal_xfer(void *ctx, uint8_t tx) {
    (void)ctx;
    uint8_t rx;
    spi_write_read_blocking(SD_SPI, &tx, &rx, 1);
    return rx;
}

static void hal_read(void *ctx, uint8_t *dst, size_t n) {
    (void)ctx;
    spi_read_blocking(SD_SPI, 0xff, dst, n);
}

static void hal_write(void *ctx, const uint8_t *src, size_t n) {
    (void)ctx;
    spi_write_blocking(SD_SPI, src, n);
}

static void hal_set_speed(void *ctx, uint32_t hz) {
    (void)ctx;
    spi_set_baudrate(SD_SPI, hz);
}

static uint32_t hal_millis(void *ctx) {
    (void)ctx;
    return to_ms_since_boot(get_absolute_time());
}

static const struct sd_hal hal = {
    .ctx = NULL,
    .select = hal_select,
    .xfer = hal_xfer,
    .read = hal_read,
    .write = hal_write,
    .set_speed = hal_set_speed,
    .millis = hal_millis,
};


const struct sd_hal *sd_hal_pico_init() {
    // Deselected before anything else, so the card never sees a stray select
    gpio_init(CONFIG_PIN_SD_CS);
    gpio_put(CONFIG_PIN_SD_CS, 1);
    gpio_set_dir(CONFIG_PIN_SD_CS, GPIO_OUT);

    spi_init(SD_SPI, 400 * 1000);
    spi_set_format(SD_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);  // SPI mode 0
    gpio_set_function(CONFIG_PIN_SD_SCK, GPIO_FUNC_SPI);
    gpio_set_function(CONFIG_PIN_SD_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(CONFIG_PIN_SD_MISO, GPIO_FUNC_SPI);

    // The pads default to a pull-down, which holds MISO low (it looks like a shorted card) unless
    // there is an external pull-up. Pull up the lines the SD spec wants pulled up (DAT0, CMD, DAT3/CS):
    // the internal ~50k is enough on short wires, and harmless next to external resistors.
    gpio_pull_up(CONFIG_PIN_SD_MISO);
    gpio_pull_up(CONFIG_PIN_SD_MOSI);
    gpio_pull_up(CONFIG_PIN_SD_CS);

    return &hal;
}

void sd_hal_pico_deinit() {
    spi_deinit(SD_SPI);

    // Back to plain inputs. The card's chip select is left to whatever pull-up is on the board.
    const uint pins[] = { CONFIG_PIN_SD_MISO, CONFIG_PIN_SD_CS, CONFIG_PIN_SD_SCK, CONFIG_PIN_SD_MOSI };
    for(unsigned i = 0; i < sizeof pins / sizeof pins[0]; i++) {
        gpio_init(pins[i]);
        gpio_disable_pulls(pins[i]);
    }
}
