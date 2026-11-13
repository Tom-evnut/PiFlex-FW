// PiFlex-FW - FlexRay MITM bridge (TX path, hardware glue).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Starts the two forwarder state machines that make up the MITM bit-mirror
// bridge. One repeats FR1 RXD onto FR2 TXD, the other FR2 RXD onto FR1 TXD,
// each driving the destination channel's TXEN for the frame. The replay, TXEN
// arm/release, half-duplex gate and watchdog logic live in flexray_tx.pio.
//
// The forwarders run on a separate PIO instance (pio1) from the RX decoders
// (pio0), so capture and bridging run at the same time on the same pins: the
// RX decoder keeps reading each RXD input while the bridge reads it too.
//
// Half-duplex gate (see flexray_tx.pio): each SM reads the pin at IN_base + 1
// at arm and mutes if it is low (other direction driving). For FR1->FR2 that
// pin is FR1_RXD(26)+1 = FR1_TXEN(27) - the FR2->FR1 forwarder's own TXEN, so
// the gate works directly. For FR2->FR1 that pin is FR2_RXD(6)+1 = GP7, a spare
// GPIO this code pulls high (inactive) so that direction forwards unmuted;
// driving GP7 from the FR1->FR2 TXEN would complete the symmetric reverse gate.
#include "flexray_tx.h"
#include "flexray_tx.pio.h"
#include "config.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

static PIO  s_pio = pio1;
static uint s_sm_to_fr2, s_sm_to_fr1, s_offset;

// Cross-SM gate flag for the FR2->FR1 forwarder, read at FR2_RXD + 1 = GP7.
// In the bidirectional build the FR1->FR2 forwarder drives GP7 (via side-set) to
// mirror its own TXEN, so FR2->FR1 is gated on "is FR1->FR2 driving FR2?" - the
// symmetric counterpart of FR1->FR2's gate, which lands naturally on FR1_TXEN
// (FR1_RXD+1 == FR1_TXEN). GP7 must be a free GPIO (unused on this board).
#define FR_TX_GATE_FR2_TO_FR1  (FR2_RXD_PIN + 1u)   // GP7

void flexray_tx_init(void) {
    s_offset = pio_add_program(s_pio, &flexray_tx_program);

    // Default GP7 high (= "FR1->FR2 not driving") so the FR2->FR1 gate reads a
    // defined level before FR1->FR2 starts, and permanently in the one-way build
    // where FR1->FR2 is absent. In the bidirectional build FR1->FR2's side-set
    // then takes over driving GP7.
    gpio_init(FR_TX_GATE_FR2_TO_FR1);
    gpio_pull_up(FR_TX_GATE_FR2_TO_FR1);
    gpio_set_dir(FR_TX_GATE_FR2_TO_FR1, GPIO_IN);

#ifdef FR_BRIDGE_ONE_WAY
    // One-way bridge: FR2 -> FR1 only. FR2 is the listened input, so keep its
    // TXEN released (high) directly - the reverse forwarder is not started, so
    // nothing drives FR2 and there is no echo loop to oscillate. Lets the bench
    // verify forward-path fidelity (generator on FR2, BM listener on FR1). GP7
    // stays pulled high, so this direction is ungated (there is no echo source).
    gpio_init(FR2_TXEN_PIN); gpio_put(FR2_TXEN_PIN, 1); gpio_set_dir(FR2_TXEN_PIN, GPIO_OUT);

    s_sm_to_fr1 = pio_claim_unused_sm(s_pio, true);
    // flag_pin = its own TXEN (redundant side-set; no second SM reads it here).
    flexray_tx_program_init(s_pio, s_sm_to_fr1, s_offset,
                            FR2_RXD_PIN, FR1_TXD_PIN, FR1_TXEN_PIN, FR1_TXEN_PIN,
                            FR_TX_IDLE_BITS);
#else
    s_sm_to_fr2 = pio_claim_unused_sm(s_pio, true);
    s_sm_to_fr1 = pio_claim_unused_sm(s_pio, true);

    // FR1 -> FR2: listen FR1 RXD, drive FR2 TXD + FR2 TXEN. Gate = FR1_RXD+1 =
    // FR1_TXEN (FR2->FR1's TXEN). flag_pin = GP7: side-set mirrors this SM's TXEN
    // onto GP7 so FR2->FR1 can gate on it.
    flexray_tx_program_init(s_pio, s_sm_to_fr2, s_offset,
                            FR1_RXD_PIN, FR2_TXD_PIN, FR2_TXEN_PIN,
                            FR_TX_GATE_FR2_TO_FR1, FR_TX_IDLE_BITS);
    // FR2 -> FR1: listen FR2 RXD, drive FR1 TXD + FR1 TXEN. Gate = FR2_RXD+1 =
    // GP7 (driven above). flag_pin = its own TXEN (redundant; FR1->FR2 reads the
    // real FR1_TXEN at FR1_RXD+1).
    flexray_tx_program_init(s_pio, s_sm_to_fr1, s_offset,
                            FR2_RXD_PIN, FR1_TXD_PIN, FR1_TXEN_PIN, FR1_TXEN_PIN,
                            FR_TX_IDLE_BITS);
#endif
}

void flexray_tx_deinit(void) {
    // Stop and release the forwarder SM(s) this build claimed, then free the pio1
    // program. The caller (mode manager) has already driven both TXEN high, so the
    // transceivers are back in RX before the program is removed.
#ifdef FR_BRIDGE_ONE_WAY
    pio_sm_set_enabled(s_pio, s_sm_to_fr1, false);
    pio_sm_unclaim(s_pio, s_sm_to_fr1);
#else
    pio_sm_set_enabled(s_pio, s_sm_to_fr2, false);
    pio_sm_set_enabled(s_pio, s_sm_to_fr1, false);
    pio_sm_unclaim(s_pio, s_sm_to_fr2);
    pio_sm_unclaim(s_pio, s_sm_to_fr1);
#endif
    pio_remove_program(s_pio, &flexray_tx_program, s_offset);

    // Return the cross-SM gate GPIO (GP7) to a defined, non-driving input so a
    // later mode does not inherit a driven level.
    gpio_init(FR_TX_GATE_FR2_TO_FR1);
    gpio_pull_up(FR_TX_GATE_FR2_TO_FR1);
    gpio_set_dir(FR_TX_GATE_FR2_TO_FR1, GPIO_IN);
}
