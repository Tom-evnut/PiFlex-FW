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

// Number of times the capture DMA ring overflowed (lapped the reader, dropping
// captured words) for a channel - PIFLEX_SOURCE_FR1 or PIFLEX_SOURCE_FR2. Exposed
// for diagnostics; the dropped span is already visible to the host as a frame gap.
uint32_t flexray_rx_overflow_count(uint8_t channel);

#endif // PIFLEX_FLEXRAY_RX_H
