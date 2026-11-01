// PiFlex-FW - FlexRay frame decode core. SDK-free so it can be unit-tested on a
// host (see test/decode_test.c). Clean-room; CRC parameters are FlexRay spec
// values (header-CRC init 0x1A to be re-checked against spec v3.0.1).
// SPDX-License-Identifier: GPL-3.0-or-later
#include "flexray_decode.h"
#include "protocol.h"
#include <string.h>

// --- FlexRay CRCs ----------------------------------------------------------

// 11-bit header CRC over the 20 protected bits, MSB first.
uint16_t flexray_header_crc(uint8_t sync, uint8_t startup,
                            uint16_t frame_id, uint8_t len_words) {
    uint32_t data = ((uint32_t)(sync & 1) << 19) |
                    ((uint32_t)(startup & 1) << 18) |
                    ((uint32_t)(frame_id & 0x7FF) << 7) |
                    (uint32_t)(len_words & 0x7F);
    uint16_t crc = 0x1A;                  // FlexRay header-CRC init
    for (int i = 19; i >= 0; i--) {
        uint16_t in_bit = (data >> i) & 1;
        uint16_t msb = (crc >> 10) & 1;
        crc = (uint16_t)((crc << 1) & 0x7FF);
        if (msb ^ in_bit) crc ^= 0x385;   // header-CRC polynomial
    }
    return crc & 0x7FF;
}

// 24-bit frame CRC over the header + payload bytes, MSB first.
uint32_t flexray_frame_crc(const uint8_t *data, int len, uint8_t channel) {
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

void fr_decode_reset(fr_decode_t *st) {
    memset(st, 0, sizeof *st);
}

// Parse a complete frame in st->buf, check CRCs, fill *out.
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

    const uint8_t sync    = (out->indicators >> 1) & 1;   // sync-frame indicator
    const uint8_t startup = out->indicators & 1;           // startup-frame indicator
    out->header_crc_ok = (flexray_header_crc(sync, startup, out->slot_id, words) == out->header_crc);
    out->frame_crc_ok  = (flexray_frame_crc(b, 5 + out->payload_len, channel) == out->frame_crc);
#if FR_FRAME_CRC_ANY_CHANNEL
    if (!out->frame_crc_ok) {   // single-source rig: also accept the other channel's init
        const uint8_t other = (channel == PIFLEX_SOURCE_FR2) ? PIFLEX_SOURCE_FR1 : PIFLEX_SOURCE_FR2;
        out->frame_crc_ok = (flexray_frame_crc(b, 5 + out->payload_len, other) == out->frame_crc);
    }
#endif
}

bool fr_decode_feed(fr_decode_t *st, uint32_t word, uint8_t channel, fr_frame_t *out) {
    const uint8_t byte  = (uint8_t)(word & 0xFF);
    const bool    start = (word & FR_START_FLAG) != 0;

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
