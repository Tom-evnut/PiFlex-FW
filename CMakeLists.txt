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

# Build the phase-4 injector: regenerate FR2->FR1 frames (splicing a host
# override) instead of the verbatim bit-mirror, so a payload byte can be changed
# with a recomputed CRC. cmake -DFR_INJECT=ON ...  (see src/flexray_inject_tx.c).
option(FR_INJECT "Build the FlexRay injector (regenerate FR2->FR1, splice overrides)" OFF)

# Build the phase-5 source / generator: instead of the MITM bridge, the board
# actively transmits its own static FlexRay schedule (flexray_source_schedule.h)
# on FR1 + FR2 with a free-running 0..63 cycle counter - a standalone bench
# signal source. cmake -DFR_SOURCE=ON ...  (see src/flexray_source_tx.c). The
# source TU is only compiled in this build, so the default / -DFR_BRIDGE_ONE_WAY
# / -DFR_INJECT builds are unchanged.
option(FR_SOURCE "Build the FlexRay source/generator (standalone static schedule)" OFF)

add_executable(piflex_fw
    src/main.c
    src/flexray_rx.c
    src/flexray_tx.c
    src/flexray_decode.c
    src/flexray_record.c
    src/flexray_injector.c
    src/flexray_source_cfg.c
    src/flexray_inject_build.c
    src/flexray_inject_tx.c
    src/usb_descriptors.c
)

# Assemble the FlexRay RX, TX-mirror and TX-inject PIO programs into headers.
pico_generate_pio_header(piflex_fw ${CMAKE_CURRENT_LIST_DIR}/src/flexray_rx.pio)
pico_generate_pio_header(piflex_fw ${CMAKE_CURRENT_LIST_DIR}/src/flexray_tx.pio)
pico_generate_pio_header(piflex_fw ${CMAKE_CURRENT_LIST_DIR}/src/flexray_inject.pio)

target_include_directories(piflex_fw PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

if(FR_BRIDGE_ONE_WAY)
    target_compile_definitions(piflex_fw PRIVATE FR_BRIDGE_ONE_WAY=1)
endif()

if(FR_INJECT)
    target_compile_definitions(piflex_fw PRIVATE FR_INJECT=1)
endif()

if(FR_SOURCE)
    # Generator = schedule builder (SDK-free, shared with the host test) + the
    # hardware TX/scheduler glue. Compiled only here so other builds are unchanged.
    target_sources(piflex_fw PRIVATE
        src/flexray_source.c
        src/flexray_source_tx.c
    )
    target_compile_definitions(piflex_fw PRIVATE FR_SOURCE=1)
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
