cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(crankcam_testbench C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Single firmware image, command-driven over USB serial (see src/main.c)
# -- generation and capture are entered/left and RPM changed live, no
# reset or reflash needed to change anything.
add_executable(crankcam_testbench
    src/main.c
    src/profiles.c
    src/event_table.c
    src/gen_fire.c
    src/capture_analysis.c
    src/engine.c
)

pico_generate_pio_header(crankcam_testbench
    ${CMAKE_CURRENT_LIST_DIR}/src/event_gen.pio
)

pico_generate_pio_header(crankcam_testbench
    ${CMAKE_CURRENT_LIST_DIR}/src/capture.pio
)

target_link_libraries(crankcam_testbench
    pico_stdlib
    hardware_pio
    hardware_dma
)

pico_enable_stdio_usb(crankcam_testbench 1)
pico_enable_stdio_uart(crankcam_testbench 0)

pico_add_extra_outputs(crankcam_testbench)
