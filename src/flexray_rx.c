// PiFlex-FW - FlexRay channel capture.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Clean-room FlexRay RX path. The PIO program (flexray_rx.pio) recovers byte
// framing from the line and pushes one 32-bit word per byte (bit 8 = frame
// start, bits 7..0 = data byte, MSB first). This file reassembles frames, parses
// the 5-byte header, and verifies the header and frame CRCs. Nothing here is
// derived from third-party firmware; the CRC parameters are FlexRay spec values:
//   header CRC: 11-bit, poly 0x385, init 0x1A, over {sync, startup, id[11], len[7]}
//   frame CRC : 24-bit, poly 0x5D6DCB, init 0xFEDCBA (ch A / FR1) or
//               0xABCDEF (ch B / FR2), over the header + payload bytes
// (Header-CRC init 0x1A should be re-checked against the FlexRay spec v3.0.1.)
#include "flexray_rx.h"
#include "flexray_rx.pio.h"
#include "protocol.h"
#include "hardware/pio.h"
#include <string.h>

static PIO  s_pio = pio0;
static uint s_sm_fr1, s_sm_fr2, s_offset;

// Per-channel reassembly state.
typedef struct {
    uint8_t  buf[5 + FR_MAX_PAYLOAD + 3];
    int      have;      // bytes collected so far
    int      need;      // total bytes expected once the header is parsed (0 = not yet)
    bool     in_frame;  // inside a frame (seen the start flag, not yet complete)
} fr_decode_t;

static fr_decode_t s_dec_fr1, s_dec_fr2;

// --- FlexRay CRCs ----------------------------------------------------------

// 11-bit header CRC over the 20 protected bits, MSB first.
static uint16_t flexray_header_crc(uint8_t sync, uint8_t startup,
                                   uint16_t frame_id, uint8_t len_words) {
    uint32_t data = ((uint32_t)(sync & 1) << 19) |
                    ((uint32_t)(startup & 1) << 18) |
                    ((uint32_t)(frame_id & 0x7FF) << 7) |
                    (uint32_t)(len_words & 0x7F);
    uint16_t crc = 0x1A;                 // FlexRay header-CRC init
    for (int i = 19; i >= 0; i--) {
        uint16_t in_bit = (data >> i) & 1;
        uint16_t msb = (crc >> 10) & 1;
        crc = (uint16_t)((crc << 1) & 0x7FF);
        if (msb ^ in_bit) crc ^= 0x385;  // header-CRC polynomial
    }
    return crc & 0x7FF;
}

// 24-bit frame CRC over the header + payload bytes, MSB first.
static uint32_t flexray_frame_crc(const uint8_t *data, int len, uint8_t channel) {
    uint32_t crc = (channel == PIFLEX_SOURCE_FR2) ? 0xABCDEFu : 0xFEDCBAu;
    for (int i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 16;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x800000u) crc = (crc << 1) ^ 0x5D6DCBu; // frame-CRC polynomial
            else                 crc <<= 1;
        }
    }
    return crc & 0xFFFFFFu;
}

// --- framing ---------------------------------------------------------------

// Finish a complete frame in st->buf: parse fields, check CRCs, fill *out.
static void finish_frame(const fr_decode_t *st, uint8_t channel, fr_frame_t *out) {
    const uint8_t *b = st->buf;

    out->channel     = channel;
    out->indicators  = (uint8_t)(b[0] >> 3);
    out->slot_id     = (uint16_t)(((b[0] & 0x07) << 8) | b[1]);
    const int words  = (b[2] >> 1) & 0x7F;
    out->payload_len = (uint8_t)(words * 2);
    out->header_crc  = (uint16_t)(((b[2] & 1) << 10) | (b[3] << 2) | (b[4] >> 6));
    out->cycle       = (uint8_t)(b[4] & 0x3F);

    memcpy(out->payload, &b[5], out->payload_len);

    const uint8_t *crc = &b[5 + out->payload_len];
    out->frame_crc = ((uint32_t)crc[0] << 16) | ((uint32_t)crc[1] << 8) | crc[2];

    const uint8_t sync    = (out->indicators >> 1) & 1;  // sync-frame indicator
    const uint8_t startup = out->indicators & 1;          // startup-frame indicator
    out->header_crc_ok = (flexray_header_crc(sync, startup, out->slot_id, words) == out->header_crc);
    out->frame_crc_ok  = (flexray_frame_crc(b, 5 + out->payload_len, channel) == out->frame_crc);
}

// Feed one PIO word into a channel's state machine. Returns true (and fills
// *out) when a complete frame is ready.
static bool feed(fr_decode_t *st, uint32_t word, uint8_t channel, fr_frame_t *out) {
    const uint8_t byte  = (uint8_t)(word & 0xFF);
    const bool    start = (word & 0x100u) != 0;

    if (start) { st->have = 0; st->need = 0; st->in_frame = true; }
    if (!st->in_frame) return false;                       // junk between frames

    if (st->have < (int)sizeof st->buf) st->buf[st->have++] = byte;

    if (st->need == 0 && st->have == 5) {
        const int words = (st->buf[2] >> 1) & 0x7F;
        st->need = 5 + words * 2 + 3;
        if (st->need > (int)sizeof st->buf) { st->in_frame = false; return false; }
    }

    if (st->need != 0 && st->have >= st->need) {
        finish_frame(st, channel, out);
        st->in_frame = false;
        return true;
    }
    return false;
}

// --- public API ------------------------------------------------------------

void flexray_rx_init(void) {
    s_offset = pio_add_program(s_pio, &flexray_rx_program);

    s_sm_fr1 = pio_claim_unused_sm(s_pio, true);
    s_sm_fr2 = pio_claim_unused_sm(s_pio, true);

    flexray_rx_program_init(s_pio, s_sm_fr1, s_offset, FR1_RXD_PIN, FR_BITRATE_HZ, FR_OVERSAMPLE);
    flexray_rx_program_init(s_pio, s_sm_fr2, s_offset, FR2_RXD_PIN, FR_BITRATE_HZ, FR_OVERSAMPLE);

    memset(&s_dec_fr1, 0, sizeof s_dec_fr1);
    memset(&s_dec_fr2, 0, sizeof s_dec_fr2);

    // TODO: move the RX FIFOs onto DMA ring buffers so bursts aren't dropped
    // while the main loop services USB.
}

bool flexray_rx_poll(fr_frame_t *out) {
    while (!pio_sm_is_rx_fifo_empty(s_pio, s_sm_fr1)) {
        if (feed(&s_dec_fr1, pio_sm_get(s_pio, s_sm_fr1), PIFLEX_SOURCE_FR1, out)) return true;
    }
    while (!pio_sm_is_rx_fifo_empty(s_pio, s_sm_fr2)) {
        if (feed(&s_dec_fr2, pio_sm_get(s_pio, s_sm_fr2), PIFLEX_SOURCE_FR2, out)) return true;
    }
    return false;
}
