cmake_minimum_required(VERSION 3.16)

set(PICO_BOARD none)
set(PICO_PLATFORM rp2350-arm-s)
set(PICO_FLASH_SIZE_BYTES 4194304)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include(pico_sdk_import.cmake)

project(rp_msx_rom_and_floppy_emulator C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Flash setup: these must be set globally, before pico_sdk_init(). Flash clock 252 MHz / 3 = 84 MHz.
add_compile_definitions(
    PICO_EMBED_XIP_SETUP=1
    PICO_BOOT_STAGE2_CHOOSE_W25Q080=1
    PICO_FLASH_SPI_CLKDIV=3
)

pico_sdk_init()

add_compile_options(-Wall -Wextra -Wnull-dereference -Wno-unused-parameter -Wno-unused-function)

set(FW rp_msx_rom_and_floppy_emulator)
set(SRC ${CMAKE_CURRENT_LIST_DIR}/src)

add_executable(${FW}
    ${SRC}/main.c
    ${SRC}/vga.c
    ${SRC}/colors.c
    ${SRC}/render.c
    ${SRC}/render_testpat.c
    ${SRC}/rom.c
    ${SRC}/rom_image.c
    ${SRC}/fat.c
    ${SRC}/sd_card.c
    ${SRC}/sd_hal_pico.c
    ${SRC}/vdp_bus.c
    ${SRC}/bus_write.c
    ${SRC}/v9938.c
    ${SRC}/fdc_fast.c
    ${SRC}/fdc_cmd.c
    ${SRC}/floppy.c
    ${SRC}/fdc_bus.c
    ${SRC}/menu.c
    ${SRC}/menu_bus.c
    ${SRC}/disk_pick.c
)

pico_generate_pio_header(${FW} ${SRC}/vga.pio)
pico_generate_pio_header(${FW} ${SRC}/bus_write.pio)
pico_generate_pio_header(${FW} ${SRC}/rom.pio)

# The configuration tested on hardware (Omega MSX2, Panasonic FS-A1)
target_compile_definitions(${FW} PRIVATE
    CONFIG_SYSCLOCK=252.0
    PICO_RP2350A=0
    PICO_HEAP_SIZE=0
    PICO_TIME_DEFAULT_ALARM_POOL_DISABLED=1
    ROM_EMULATION=1
    ROM_CORE_GATE=1
    BUS_PULLS=1
    SCRATCH_HOT=1
    SD_ROM=1
    CONFIG_SD_SPI_HZ=8000000
    SD_REPORT=1
    MENU=1
    BUTTONS=1
    MEGAROM=1
    FLOPPY=1
    FLOPPY_WRITE=1
    FLOPPY_STAGES=7
    FLOPPY_DIAG=0
    FDC_SEEK_MIN_MS=0
    V9938=1
    MSX2_HOST=1
    VDP_WRITES=1
    VDP_READ_TRACK=1
)

# The whole program runs from RAM; flash is only read at boot
pico_set_binary_type(${FW} copy_to_ram)

# Move the normal RAM region up by 256K and put the cartridge buffer at the start of SRAM
pico_add_linker_script_override_path(${FW} ${SRC}/linker
    FILES memory_extra.incl section_extra_post_data.incl)
target_link_options(${FW} PRIVATE
    -Wl,--defsym=RAM_ORIGIN=0x20040000
    -Wl,--defsym=RAM_LENGTH=0x40000)

target_include_directories(${FW} PRIVATE ${SRC})

pico_enable_stdio_usb(${FW} 0)
pico_enable_stdio_uart(${FW} 0)

target_link_libraries(${FW} PRIVATE
    pico_stdlib
    pico_multicore
    hardware_adc
    hardware_dma
    hardware_flash
    hardware_pio
    hardware_spi
    hardware_vreg
)

pico_add_extra_outputs(${FW})
