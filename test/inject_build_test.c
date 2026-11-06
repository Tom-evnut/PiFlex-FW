// PiFlex-FW - host test for the injected-frame builder (flexray_inject_build.c).
//
// Builds a valid FlexRay frame, injects a replace window, and feeds the rebuilt
// bytes back through the decode core (fr_decode_feed) to prove the injected
// frame is well-formed: the spliced payload is present and BOTH CRCs validate
// (the 24-bit frame CRC was recomputed; the header is untouched). This is the
// maths of Stage B checked end to end without a device.
//
// Build & run (host compiler, no Pico SDK needed):
//   gcc -I ../src inject_build_test.c ../src/flexray_inject_build.c ../src/flexray_decode.c -o inject_build_test && ./inject_build_test
#include "flexray_inject_build.h"
#include "flexray_decode.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

// Fill *f with a self-consistent frame (correct header + frame CRC), like
// test/record_test.c's make_frame, for the given channel and payload.
static void make_frame(fr_frame_t *f, uint8_t channel, uint16_t slot, uint8_t cycle,
                       const uint8_t *payload, int plen) {
    memset(f, 0, sizeof *f);
    f->channel     = channel;
    f->indicators  = 0;
    f->slot_id     = slot;
    f->cycle       = cycle;
    f->payload_len = (uint8_t)plen;
    memcpy(f->payload, payload, (size_t)plen);
    f->header_crc  = flexray_header_crc(0, 0, f->slot_id, (uint8_t)(plen / 2));

    uint8_t hdr[5 + FR_MAX_PAYLOAD];
    hdr[0] = (uint8_t)((f->indicators << 3) | ((f->slot_id >> 8) & 0x07));
    hdr[1] = (uint8_t)(f->slot_id & 0xFF);
    hdr[2] = (uint8_t)(((plen / 2) << 1) | ((f->header_crc >> 10) & 0x01));
    hdr[3] = (uint8_t)((f->header_crc >> 2) & 0xFF);
    hdr[4] = (uint8_t)(((f->header_crc & 0x03) << 6) | (f->cycle & 0x3F));
    memcpy(&hdr[5], payload, (size_t)plen);
    f->frame_crc   = flexray_frame_crc(hdr, 5 + plen, channel);
    f->header_crc_ok = true;
    f->frame_crc_ok  = true;
}

// Decode a built byte sequence back into a frame: set the frame-start flag on
// byte 0, feed every byte, and expect a completed frame.
static bool decode_built(const uint8_t *bytes, int n, uint8_t channel, fr_frame_t *out) {
    fr_decode_t st;
    fr_decode_reset(&st);
    bool done = false;
    for (int i = 0; i < n; i++) {
        uint32_t word = bytes[i] | (i == 0 ? FR_START_FLAG : 0u);
        if (fr_decode_feed(&st, word, channel, out)) done = true;
    }
    return done;
}

// Inject and verify for one channel: splice window at offset, prove the rebuilt
// frame decodes with the new bytes and valid CRCs, and that the frame CRC moved.
static void inject_case(uint8_t channel, uint16_t slot, uint8_t cycle,
                        const uint8_t *payload, int plen,
                        const uint8_t *window, uint8_t offset, uint8_t wlen) {
    fr_frame_t f;
    make_frame(&f, channel, slot, cycle, payload, plen);
    const uint32_t orig_crc = f.frame_crc;

    uint8_t out[FR_INJECT_FRAME_MAX];
    const int n = fr_inject_build_frame(out, &f, window, offset, wlen);
    CHECK(n == 5 + plen + 3, "built frame length = header + payload + CRC");

    fr_frame_t got;
    CHECK(decode_built(out, n, channel, &got), "rebuilt frame decodes to a complete frame");
    CHECK(got.slot_id == slot, "slot id preserved");
    CHECK(got.cycle == cycle, "cycle preserved");
    CHECK(got.payload_len == plen, "payload length preserved");
    CHECK(got.header_crc_ok, "header CRC still valid (header untouched)");
    CHECK(got.frame_crc_ok, "recomputed frame CRC validates on the rebuilt frame");

    // The window bytes are spliced; the rest of the payload is unchanged.
    CHECK(memcmp(&got.payload[offset], window, wlen) == 0, "window bytes spliced");
    for (int i = 0; i < plen; i++)
        if (i < offset || i >= offset + wlen)
            CHECK(got.payload[i] == payload[i], "non-window payload byte unchanged");

    // If the window actually changed the payload, the frame CRC must differ.
    bool changed = memcmp(&payload[offset], window, wlen) != 0;
    if (changed) CHECK(got.frame_crc != orig_crc, "frame CRC changes when payload changes");
}

// Extract on-wire bit i (0 = first bit emitted), MSB-first from the serialized
// words (words[1..]; words[0] is the count word).
static int bit_at(const uint32_t *w, int i) {
    return (int)((w[1 + (i >> 5)] >> (31 - (i & 31))) & 1u);
}

// Serialize a built frame and decode the bit stream straight back per the
// FlexRay line coding, pinning the full on-wire framing - the TSS/FSS/BSS/FES
// structure, the header length field as it reaches the wire, and the emitted
// word count. This is the layer the bench found broken; a regression here now
// fails on the host.
static void test_serialize(void) {
    const uint8_t payload[8] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11}; // bench-like
    fr_frame_t f;
    make_frame(&f, PIFLEX_SOURCE_FR2, 0x001, 5, payload, (int)sizeof payload);

    uint8_t frame[FR_INJECT_FRAME_MAX];
    const int n = fr_inject_build_frame(frame, &f, NULL, 0, 0);   // verbatim rebuild
    CHECK(n == 5 + 8 + 3, "built frame is header + 8 payload + CRC");
    CHECK(((frame[2] >> 1) & 0x7F) == 4, "header length field encodes 4 words (8-byte payload)");

    uint32_t words[FR_INJECT_WORDS_MAX];
    const int nw = fr_inject_serialize(words, frame, n);
    const int total_bits = (int)words[0] + 1;
    const int exp_bits   = FR_INJECT_TSS_BITS + 1 + 10 * n + 2;
    CHECK(total_bits == exp_bits, "total bit count = TSS + FSS + 10/byte + FES");
    CHECK(nw == 1 + (exp_bits + 31) / 32, "emitted word count matches the bit count");

    int p = 0;
    for (int i = 0; i < FR_INJECT_TSS_BITS; i++) CHECK(bit_at(words, p++) == 0, "TSS low bit");
    CHECK(bit_at(words, p++) == 1, "FSS high bit");

    uint8_t got[FR_INJECT_FRAME_MAX];
    for (int b = 0; b < n; b++) {
        CHECK(bit_at(words, p++) == 1, "BSS high bit");
        CHECK(bit_at(words, p++) == 0, "BSS low bit");
        uint8_t v = 0;
        for (int k = 0; k < 8; k++) v = (uint8_t)((v << 1) | bit_at(words, p++));   // MSB first
        got[b] = v;
    }
    CHECK(bit_at(words, p++) == 0, "FES low bit");
    CHECK(bit_at(words, p++) == 1, "FES high bit");
    CHECK(p == total_bits, "parsed exactly the emitted bits");
    CHECK(memcmp(got, frame, (size_t)n) == 0, "bit stream decodes back to the exact frame bytes");

    // The on-wire header bytes must match the built header - this is the length
    // field (got[2]) the bench saw read as half; pin it explicitly.
    CHECK(got[0] == frame[0] && got[1] == frame[1], "slot-ID header bytes on the wire");
    CHECK(got[2] == frame[2], "length/header-CRC byte on the wire");
}

int main(void) {
    const uint8_t payload[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
    const uint8_t window[]  = {0xAA, 0xBB};

    // Bench slots on both channels, window at the front (the sample rules).
    inject_case(PIFLEX_SOURCE_FR2, 0x001, 5, payload, (int)sizeof payload, window, 0, 2);
    inject_case(PIFLEX_SOURCE_FR1, 0x002, 0, payload, (int)sizeof payload, window, 0, 2);

    // Window in the middle of the payload.
    const uint8_t mid[] = {0xDE, 0xAD};
    inject_case(PIFLEX_SOURCE_FR2, 0x004, 63, payload, (int)sizeof payload, mid, 2, 2);

    // Single-source bench rig: a frame carrying channel-A (FR1) CRC but delivered
    // on channel B (FR2). The builder must recompute under the A init so the
    // rebuilt frame still validates the way the original did.
    {
        fr_frame_t f;
        make_frame(&f, PIFLEX_SOURCE_FR1, 0x001, 1, payload, (int)sizeof payload); // A-init CRC
        f.channel = PIFLEX_SOURCE_FR2;                                             // but on bus B
        uint8_t out[FR_INJECT_FRAME_MAX];
        const int n = fr_inject_build_frame(out, &f, window, 0, 2);
        fr_frame_t got;
        decode_built(out, n, PIFLEX_SOURCE_FR1, &got);   // far side reads it as A
        CHECK(got.frame_crc_ok, "cross-channel rig: rebuilt CRC validates under the original init");
    }

    // A window that does not fit the payload -> builder refuses (returns 0).
    {
        fr_frame_t f;
        make_frame(&f, PIFLEX_SOURCE_FR2, 0x002, 2, payload, (int)sizeof payload);
        uint8_t out[FR_INJECT_FRAME_MAX];
        CHECK(fr_inject_build_frame(out, &f, window, 5, 2) == 0, "window past payload end is refused");
    }

    test_serialize();

    printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
    return failures ? 1 : 0;
}
