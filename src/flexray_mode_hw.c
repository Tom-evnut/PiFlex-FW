// PiFlex-FW - hardware hooks for the runtime data-path mode manager (phase 7).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Only one TX mode drives the bus at a time, so each mode's PIO program / SMs /
// DMA are loaded on entry and torn down on leave (the dedicated *_init / *_deinit
// functions). RX capture (pio0) is never touched. The mode core guarantees the
// order safe_bus -> teardown(old) -> enter(new), so both TXEN lines are released
// (transceivers back in RX) before pio1 is reprogrammed and the swap cannot
// glitch the bus.
#include "flexray_mode_hw.h"
#include "flexray_mode.h"
#include "flexray_tx.h"
#include "flexray_inject_tx.h"
#include "flexray_source_tx.h"
#include "flexray_source_cfg.h"
#include "config.h"
#include "protocol.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

// Make the bus safe regardless of the current mode: stop every pio1 TX state
// machine and drive both TXEN lines inactive HIGH (so each NCV7383 stays in
// receive and drives its RXD). Reclaiming the TXEN pins to SIO makes the level
// take effect immediately even if a PIO program was driving them.
static void hw_safe_bus(void) {
    for (uint sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++)
        pio_sm_set_enabled(pio1, sm, false);

    gpio_init(FR1_TXEN_PIN); gpio_set_dir(FR1_TXEN_PIN, GPIO_OUT); gpio_put(FR1_TXEN_PIN, 1);
    gpio_init(FR2_TXEN_PIN); gpio_set_dir(FR2_TXEN_PIN, GPIO_OUT); gpio_put(FR2_TXEN_PIN, 1);
}

// Free the given mode's pio1 program / SMs / DMA. SNIFF holds nothing.
static void hw_teardown(fr_mode_t mode) {
    switch (mode) {
        case PIFLEX_MODE_BRIDGE: flexray_tx_deinit();        break;
        case PIFLEX_MODE_INJECT: flexray_inject_tx_deinit(); break;
        case PIFLEX_MODE_SOURCE: flexray_source_deinit();
                                 fr_source_cfg_set_active(false); break;
        case PIFLEX_MODE_SNIFF:
        default:                 break;
    }
}

// Start the given mode's data path, exactly as that mode's dedicated build main()
// did. SNIFF starts nothing - RX (pio0) is already running and the bus is safe.
static void hw_enter(fr_mode_t mode) {
    switch (mode) {
        case PIFLEX_MODE_BRIDGE: flexray_tx_init();        break;
        case PIFLEX_MODE_INJECT: flexray_inject_tx_init(); break;
        case PIFLEX_MODE_SOURCE: flexray_source_init();
                                 fr_source_cfg_set_active(true); break;
        case PIFLEX_MODE_SNIFF:
        default:                 break;
    }
}

static const fr_mode_hooks_t s_hw_hooks = {
    .safe_bus = hw_safe_bus,
    .teardown = hw_teardown,
    .enter    = hw_enter,
};

void flexray_mode_hw_init(fr_mode_t initial) {
    fr_mode_init(&s_hw_hooks, initial);
}
