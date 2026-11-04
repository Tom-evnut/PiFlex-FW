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
#include "hardware/gpio.h"
#include "hardware/pio.h"

static PIO  s_pio = pio1;
static uint s_sm_to_fr2, s_sm_to_fr1, s_offset;

void flexray_tx_init(void) {
    s_offset = pio_add_program(s_pio, &flexray_tx_program);

#ifdef FR_BRIDGE_ONE_WAY
    // One-way bridge: FR2 -> FR1 only. FR2 is the listened input, so keep its
    // TXEN released (high) directly - the reverse forwarder is not started, so
    // nothing drives FR2 and there is no echo loop to oscillate. Lets the bench
    // verify forward-path fidelity (generator on FR2, BM listener on FR1).
    gpio_init(FR2_TXEN_PIN); gpio_put(FR2_TXEN_PIN, 1); gpio_set_dir(FR2_TXEN_PIN, GPIO_OUT);

    s_sm_to_fr1 = pio_claim_unused_sm(s_pio, true);
    flexray_tx_program_init(s_pio, s_sm_to_fr1, s_offset,
                            FR2_RXD_PIN, FR1_TXD_PIN, FR1_TXEN_PIN,
                            FR_TX_IDLE_BITS, FR_TX_MAX_FRAME_BITS);
#else
    s_sm_to_fr2 = pio_claim_unused_sm(s_pio, true);
    s_sm_to_fr1 = pio_claim_unused_sm(s_pio, true);

    // FR1 -> FR2: listen on FR1 RXD, drive FR2 TXD + FR2 TXEN.
    flexray_tx_program_init(s_pio, s_sm_to_fr2, s_offset,
                            FR1_RXD_PIN, FR2_TXD_PIN, FR2_TXEN_PIN,
                            FR_TX_IDLE_BITS, FR_TX_MAX_FRAME_BITS);
    // FR2 -> FR1: listen on FR2 RXD, drive FR1 TXD + FR1 TXEN.
    flexray_tx_program_init(s_pio, s_sm_to_fr1, s_offset,
                            FR2_RXD_PIN, FR1_TXD_PIN, FR1_TXEN_PIN,
                            FR_TX_IDLE_BITS, FR_TX_MAX_FRAME_BITS);
#endif
}
