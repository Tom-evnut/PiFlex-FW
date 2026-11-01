// PiFlex-FW - host round-trip test for the SDK-free FlexRay decode core.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Builds known-good FlexRay frames, serialises them into the PIO words the
// decoder expects (bit 8 = frame start, bits 7..0 = byte), feeds them through
// the real fr_decode_feed() / CRC code, and checks the results.
//
// Build & run (host compiler, no Pico SDK needed):
//   gcc -I ../src decode_test.c ../src/flexray_decode.c -o decode_test && ./decode_test
#include "flexray_decode.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

// Build the on-wire byte stream for a frame into buf; returns its length.
static int build_frame(uint8_t *buf, uint8_t indicators, uint16_t slot_id,
                       uint8_t cycle, const uint8_t *payload, int payload_len,
                       uint8_t channel) {
    const int words = payload_len / 2;
    const uint8_t sync    = (indicators >> 1) & 1;
    const uint8_t startup = indicators & 1;
    const uint16_t hcrc = flexray_header_crc(sync, startup, slot_id, (uint8_t)words);

    buf[0] = (uint8_t)((indicators << 3) | ((slot_id >> 8) & 7));
    buf[1] = (uint8_t)(slot_id & 0xFF);
    buf[2] = (uint8_t)((words << 1) | ((hcrc >> 10) & 1));
    buf[3] = (uint8_t)((hcrc >> 2) & 0xFF);
    buf[4] = (uint8_t)(((hcrc & 3) << 6) | (cycle & 0x3F));
    if (payload_len) memcpy(&buf[5], payload, payload_len);

    const uint32_t fcrc = flexray_frame_crc(buf, 5 + payload_len, channel);
    uint8_t *c = &buf[5 + payload_len];
    c[0] = (uint8_t)((fcrc >> 16) & 0xFF);
    c[1] = (uint8_t)((fcrc >> 8) & 0xFF);
    c[2] = (uint8_t)(fcrc & 0xFF);
    return 5 + payload_len + 3;
}

// Feed a byte stream through the decoder (frame-start flag on byte 0).
static bool decode_stream(const uint8_t *buf, int len, uint8_t channel, fr_frame_t *out) {
    fr_decode_t st;
    fr_decode_reset(&st);
    bool got = false;
    for (int i = 0; i < len; i++) {
        uint32_t w = buf[i] | (i == 0 ? FR_START_FLAG : 0);
        if (fr_decode_feed(&st, w, channel, out)) got = true;
    }
    return got;
}

int main(void) {
    uint8_t buf[300];
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02};
    const int len = build_frame(buf, 0x00, 0x123, 9, payload, (int)sizeof payload, PIFLEX_SOURCE_FR1);

    // 1. A good frame on FR1 decodes with all fields intact and both CRCs OK.
    fr_frame_t f; memset(&f, 0, sizeof f);
    CHECK(decode_stream(buf, len, PIFLEX_SOURCE_FR1, &f), "frame completes");
    CHECK(f.slot_id == 0x123, "slot id");
    CHECK(f.cycle == 9, "cycle");
    CHECK(f.payload_len == sizeof payload, "payload length");
    CHECK(memcmp(f.payload, payload, sizeof payload) == 0, "payload bytes");
    CHECK(f.channel == PIFLEX_SOURCE_FR1, "channel");
    CHECK(f.header_crc_ok, "header CRC ok");
    CHECK(f.frame_crc_ok, "frame CRC ok");

    // 2. Junk words before the start flag are ignored; the frame still parses.
    {
        fr_decode_t st; fr_decode_reset(&st);
        fr_frame_t g; memset(&g, 0, sizeof g);
        bool got = false;
        fr_decode_feed(&st, 0x55, PIFLEX_SOURCE_FR1, &g);   // no start flag -> dropped
        fr_decode_feed(&st, 0xAA, PIFLEX_SOURCE_FR1, &g);
        for (int i = 0; i < len; i++) {
            uint32_t w = buf[i] | (i == 0 ? FR_START_FLAG : 0);
            if (fr_decode_feed(&st, w, PIFLEX_SOURCE_FR1, &g)) got = true;
        }
        CHECK(got && g.slot_id == 0x123 && g.frame_crc_ok, "resync after junk");
    }

    // 3. A corrupted payload byte is caught by the frame CRC (header still OK).
    {
        uint8_t bad[300]; memcpy(bad, buf, len); bad[6] ^= 0xFF;
        fr_frame_t h; memset(&h, 0, sizeof h);
        CHECK(decode_stream(bad, len, PIFLEX_SOURCE_FR1, &h), "corrupt frame completes");
        CHECK(h.header_crc_ok, "header ok under payload corruption");
        CHECK(!h.frame_crc_ok, "frame CRC detects corruption");
    }

    // 4. Per-channel init: an FR1-built frame must not validate as FR2.
    {
        fr_frame_t b; memset(&b, 0, sizeof b);
        CHECK(decode_stream(buf, len, PIFLEX_SOURCE_FR2, &b), "completes decoded as FR2");
        CHECK(!b.frame_crc_ok, "FR1 frame fails FR2 CRC (per-channel init)");
    }

    // 5. A zero-payload (null) frame round-trips.
    {
        uint8_t nb[32];
        const int nl = build_frame(nb, 0x00, 0x001, 0, NULL, 0, PIFLEX_SOURCE_FR1);
        fr_frame_t n; memset(&n, 0, sizeof n);
        CHECK(decode_stream(nb, nl, PIFLEX_SOURCE_FR1, &n), "empty frame completes");
        CHECK(n.payload_len == 0 && n.header_crc_ok && n.frame_crc_ok, "empty frame ok");
    }

    printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
    return failures ? 1 : 0;
}
