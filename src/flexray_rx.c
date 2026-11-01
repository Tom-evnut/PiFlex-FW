// PiFlex-FW - FlexRay channel capture (hardware glue).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Starts the two PIO decoders (one per NCV7383 channel) and drains their FIFOs
// into the SDK-free decode core (flexray_decode.c). The PIO program recovers
// byte framing; the core does the frame assembly and CRC checks.
#include "flexray_rx.h"
#include "flexray_decode.h"
#include "flexray_rx.pio.h"
#include "config.h"
#include "protocol.h"
#include "hardware/pio.h"

static PIO  s_pio = pio0;
static uint s_sm_fr1, s_sm_fr2, s_offset;

static fr_decode_t s_dec_fr1, s_dec_fr2;

void flexray_rx_init(void) {
    s_offset = pio_add_program(s_pio, &flexray_rx_program);

    s_sm_fr1 = pio_claim_unused_sm(s_pio, true);
    s_sm_fr2 = pio_claim_unused_sm(s_pio, true);

    flexray_rx_program_init(s_pio, s_sm_fr1, s_offset, FR1_RXD_PIN, FR_BITRATE_HZ, FR_OVERSAMPLE);
    flexray_rx_program_init(s_pio, s_sm_fr2, s_offset, FR2_RXD_PIN, FR_BITRATE_HZ, FR_OVERSAMPLE);

    fr_decode_reset(&s_dec_fr1);
    fr_decode_reset(&s_dec_fr2);

    // TODO: move the RX FIFOs onto DMA ring buffers so bursts aren't dropped
    // while the main loop services USB.
}

bool flexray_rx_poll(fr_frame_t *out) {
    while (!pio_sm_is_rx_fifo_empty(s_pio, s_sm_fr1)) {
        if (fr_decode_feed(&s_dec_fr1, pio_sm_get(s_pio, s_sm_fr1), PIFLEX_SOURCE_FR1, out)) return true;
    }
    while (!pio_sm_is_rx_fifo_empty(s_pio, s_sm_fr2)) {
        if (fr_decode_feed(&s_dec_fr2, pio_sm_get(s_pio, s_sm_fr2), PIFLEX_SOURCE_FR2, out)) return true;
    }
    return false;
}
