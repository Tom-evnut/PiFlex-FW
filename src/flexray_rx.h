// PiFlex-FW - FlexRay channel capture (hardware glue).
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef PIFLEX_FLEXRAY_RX_H
#define PIFLEX_FLEXRAY_RX_H

#include "flexray_decode.h"   // fr_frame_t and the SDK-free decode core

// Claim PIO resources and start both channels' decoders.
void flexray_rx_init(void);

// Pump decoded frames out of the channel FIFOs. Fills *out and returns true
// when a complete frame is ready, else false.
bool flexray_rx_poll(fr_frame_t *out);

#endif // PIFLEX_FLEXRAY_RX_H
