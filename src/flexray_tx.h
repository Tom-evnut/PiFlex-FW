// PiFlex-FW - FlexRay MITM bridge (TX path, hardware glue).
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef PIFLEX_FLEXRAY_TX_H
#define PIFLEX_FLEXRAY_TX_H

// Claim the TX PIO and start the MITM bridge state machines. Each mirrors one
// channel's RXD onto the other channel's TXD and drives that channel's TXEN for
// the duration of a frame. After this returns the bridge runs autonomously in
// PIO - there is nothing for the main loop to poll. Takes over the TXD/TXEN
// pins, so main must not drive those GPIOs itself once this has been called.
//
// Builds both directions by default; with FR_BRIDGE_ONE_WAY defined it starts
// only the FR2 -> FR1 forwarder (no reverse path, no echo loop) for isolated
// forward-path bench verification.
void flexray_tx_init(void);

// Tear the bridge down for a runtime mode switch: stop + unclaim the forwarder
// SMs and free the pio1 program (TXEN is released by the mode manager first).
// After this the pio1 instruction memory and SMs are free for the next mode.
void flexray_tx_deinit(void);

#endif // PIFLEX_FLEXRAY_TX_H
