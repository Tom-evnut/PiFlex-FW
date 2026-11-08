// PiFlex-FW - hardware hooks for the runtime data-path mode manager (phase 7).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Wires the SDK-free mode core (flexray_mode.c) to the real PIO/DMA/GPIO: it
// provides the safe_bus / teardown / enter callbacks that stop + release the bus,
// free each mode's pio1 resources, and run each mode's *_init. Keeps all the
// hardware specifics out of main.c and out of the host-testable core.
#ifndef PIFLEX_FLEXRAY_MODE_HW_H
#define PIFLEX_FLEXRAY_MODE_HW_H

#include "flexray_mode.h"   // fr_mode_t

// Install the hardware hooks and bring up `initial` (see fr_mode_init). Call once
// at start-up after flexray_rx_init(); thereafter drive mode changes with
// fr_mode_set() from the control callback / main loop.
void flexray_mode_hw_init(fr_mode_t initial);

#endif // PIFLEX_FLEXRAY_MODE_HW_H
