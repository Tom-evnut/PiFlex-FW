// PiFlex-FW - FlexRay channel capture (SKELETON)
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Clean-room implementation of the FlexRay RX path. The genuinely novel work
// lives here: recovering byte framing from the oversampled RXD line, then the
// 5-byte header (slot ID, cycle, indicators, 11-bit header CRC), the payload,
// and the 24-bit frame CRC. None of this is derived from third-party firmware;
// it follows the FlexRay line coding and the onsemi NCV7383 datasheet.
#include "flexray_rx.h"
#include "flexray_rx.pio.h"
#include "hardware/pio.h"

static PIO  s_pio = pio0;
static uint s_sm_fr1, s_sm_fr2, s_offset;

void flexray_rx_init(void) {
    s_offset = pio_add_program(s_pio, &flexray_rx_program);

    s_sm_fr1 = pio_claim_unused_sm(s_pio, true);
    s_sm_fr2 = pio_claim_unused_sm(s_pio, true);

    flexray_rx_program_init(s_pio, s_sm_fr1, s_offset, FR1_RXD_PIN, FR_BITRATE_HZ, FR_OVERSAMPLE);
    flexray_rx_program_init(s_pio, s_sm_fr2, s_offset, FR2_RXD_PIN, FR_BITRATE_HZ, FR_OVERSAMPLE);

    // TODO phase 1: attach each SM's RX FIFO to a DMA ring buffer so no samples
    // are lost while the main loop services USB.
}

bool flexray_rx_poll(fr_frame_t *out) {
    (void)out;
    (void)s_sm_fr1;
    (void)s_sm_fr2;

    // TODO phase 1 - the core decoder:
    //   1. Drain oversampled bits from the DMA ring (per channel).
    //   2. Detect a TSS: >= 3 low bits after a run of idle-high.
    //   3. Lock to the Frame Start Sequence, then sample each byte at its
    //      centre, stripping byte-start sequences.
    //   4. Parse the 5-byte header -> slot_id, cycle, indicators, header_crc.
    //   5. Read payload_len*2 payload bytes, then the 3-byte frame CRC.
    //   6. Verify the 11-bit header CRC and 24-bit frame CRC; set *_crc_ok.
    //   7. Fill *out (channel = PIFLEX_SOURCE_FR1/FR2) and return true.
    return false;
}
