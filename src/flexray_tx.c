// PiFlex-FW - FlexRay MITM bridge (TX path, hardware glue).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Starts the two forwarder state machines that make up the MITM bit-mirror
// bridge. One repeats FR1 RXD onto FR2 TXD, the other FR2 RXD onto FR1 TXD,
// each driving the destination channel's TXEN for the frame. The replay and
// TXEN arm/release logic live in flexray_tx.pio.
//
// The forwarders run on a separate PIO instance (pio1) from the RX decoders
// (pio0), so capture and bridging run at the same time on the same pins: the
// RX decoder keeps reading each RXD input while the bridge reads it too.
#include "flexray_tx.h"
#include "flexray_tx.pio.h"
#include "config.h"
#include "hardware/pio.h"

static PIO  s_pio = pio1;
static uint s_sm_to_fr2, s_sm_to_fr1, s_offset;

void flexray_tx_init(void) {
    s_offset = pio_add_program(s_pio, &flexray_tx_program);

    s_sm_to_fr2 = pio_claim_unused_sm(s_pio, true);
    s_sm_to_fr1 = pio_claim_unused_sm(s_pio, true);

    // FR1 -> FR2: listen on FR1 RXD, drive FR2 TXD + FR2 TXEN.
    flexray_tx_program_init(s_pio, s_sm_to_fr2, s_offset,
                            FR1_RXD_PIN, FR2_TXD_PIN, FR2_TXEN_PIN, FR_TX_IDLE_BITS);
    // FR2 -> FR1: listen on FR2 RXD, drive FR1 TXD + FR1 TXEN.
    flexray_tx_program_init(s_pio, s_sm_to_fr1, s_offset,
                            FR2_RXD_PIN, FR1_TXD_PIN, FR1_TXEN_PIN, FR_TX_IDLE_BITS);
}
