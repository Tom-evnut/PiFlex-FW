cmake_minimum_required(VERSION 3.13)

# --- Board / SDK -----------------------------------------------------------
# The bench hardware is a plain Raspberry Pi Pico 2 (RP2350), non-W.
set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(piflex_fw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Build the MITM bridge one-way (FR2->FR1 only) for isolated forward-path bench
# verification: cmake -DFR_BRIDGE_ONE_WAY=ON ...
option(FR_BRIDGE_ONE_WAY "Build the FlexRay MITM bridge one-way (FR2->FR1 only)" OFF)

add_executable(piflex_fw
    src/main.c
    src/flexray_rx.c
    src/flexray_tx.c
    src/flexray_decode.c
    src/flexray_record.c
    src/flexray_injector.c
    src/usb_descriptors.c
)

# Assemble the FlexRay RX and TX PIO programs into generated headers.
pico_generate_pio_header(piflex_fw ${CMAKE_CURRENT_LIST_DIR}/src/flexray_rx.pio)
pico_generate_pio_header(piflex_fw ${CMAKE_CURRENT_LIST_DIR}/src/flexray_tx.pio)

target_include_directories(piflex_fw PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

if(FR_BRIDGE_ONE_WAY)
    target_compile_definitions(piflex_fw PRIVATE FR_BRIDGE_ONE_WAY=1)
endif()

target_link_libraries(piflex_fw
    pico_stdlib
    pico_unique_id
    hardware_pio
    hardware_dma
    tinyusb_device
    tinyusb_board
)

# USB belongs to TinyUSB (Panda-compatible), not to stdio.
pico_enable_stdio_usb(piflex_fw 0)
pico_enable_stdio_uart(piflex_fw 0)

pico_add_extra_outputs(piflex_fw)
