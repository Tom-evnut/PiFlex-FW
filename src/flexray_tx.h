// PiFlex-FW - FlexRay MITM bridge (TX path, hardware glue).
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef PIFLEX_FLEXRAY_TX_H
#define PIFLEX_FLEXRAY_TX_H

// Claim the TX PIO and start both MITM bridge state machines. Each mirrors one
// channel's RXD onto the other channel's TXD and drives that channel's TXEN for
// the duration of a frame. After this returns the bridge runs autonomously in
// PIO - there is nothing for the main loop to poll. Takes over the TXD/TXEN
// pins, so main must not drive those GPIOs itself once this has been called.
void flexray_tx_init(void);

#endif // PIFLEX_FLEXRAY_TX_H
