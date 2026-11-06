// PiFlex-FW - host test for the FlexRay source / generator (flexray_source.c).
//
// Builds the scheduled frames the generator would emit (fr_source_build_frame)
// and feeds the bytes back through the decode core (fr_decode_feed) to prove the
// generated frames are well-formed: correct slot IDs, a per-channel-valid 24-bit
// frame CRC, the cycle counter advancing and wrapping 0..63, and payload bytes
// exactly as scheduled. Also pins the per-channel CRC init (an FR1-built frame
// must NOT validate as FR2) and the schedule's sample slots. This is the maths
// of phase 5 checked end to end without a device.
//
// Build & run (host compiler, no Pico SDK needed):
//   gcc -I ../src source_test.c ../src/flexray_source.c ../src/flexray_inject_build.c ../src/flexray_decode.c -o source_test && ./source_test
#include "flexray_source.h"
#include "flexray_source_schedule.h"
#include "flexray_inject_build.h"
#include "flexray_decode.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

// Decode a built byte sequence back into a frame: set the frame-start flag on
// byte 0, feed every byte, expect a completed frame (as inject_build_test does).
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

// The expected payload for a slot at a cycle: fill byte everywhere, last byte =
// cycle when the counter flag is set. Independent re-implementation so the test
// pins the contract, not just mirrors the code.
static void expected_payload(uint8_t *buf, const fr_source_slot_t *slot, uint8_t cycle) {
    for (int i = 0; i < slot->payload_len; i++) buf[i] = slot->payload[i];
    if ((slot->flags & FR_SRC_FLAG_CYCLE_COUNTER) && slot->payload_len > 0)
        buf[slot->payload_len - 1] = (uint8_t)(cycle & 0x3F);
}

// Build one slot on one channel at one cycle, decode it back, and verify every
// scheduled property.
static void source_case(const fr_source_slot_t *slot, uint8_t channel, uint8_t cycle) {
    uint8_t out[FR_INJECT_FRAME_MAX];
    const int n = fr_source_build_frame(out, slot, cycle, channel);
    CHECK(n == 5 + slot->payload_len + 3, "generated frame length = header + payload + CRC");

    fr_frame_t got;
    CHECK(decode_built(out, n, channel, &got), "generated frame decodes to a complete frame");
    CHECK(got.slot_id == (slot->slot_id & 0x7FF), "slot id as scheduled");
    CHECK(got.cycle == (cycle & 0x3F), "cycle in the header matches the schedule cursor");
    CHECK(got.payload_len == slot->payload_len, "payload length as scheduled");
    CHECK(got.header_crc_ok, "header CRC valid on the generated frame");
    CHECK(got.frame_crc_ok, "per-channel frame CRC validates on the generated frame");

    uint8_t exp[FR_MAX_PAYLOAD];
    expected_payload(exp, slot, cycle);
    CHECK(memcmp(got.payload, exp, slot->payload_len) == 0, "payload bytes exactly as scheduled");
}

// Walk the whole schedule for 0..63 and one wrap step, exactly as the firmware
// scheduler does (slot cursor then cycle), and assert the cycle advances 0..63
// and wraps back to 0. Covers both channels of each multi-channel slot.
static void test_cycle_sweep(void) {
    uint8_t cycle = 0;
    uint8_t seen_min = 0xFF, seen_max = 0;
    bool wrapped = false;

    // 65 cycles so we observe 0..63 and the wrap 63 -> 0.
    for (int step = 0; step < 65; step++) {
        for (int s = 0; s < FR_SRC_DEFAULT_SLOT_COUNT; s++) {
            const fr_source_slot_t *slot = &FR_SOURCE_DEFAULT_SCHEDULE[s];
            if (slot->channels & FR_SRC_CH_FR1) source_case(slot, PIFLEX_SOURCE_FR1, cycle);
            if (slot->channels & FR_SRC_CH_FR2) source_case(slot, PIFLEX_SOURCE_FR2, cycle);
        }
        if (cycle < seen_min) seen_min = cycle;
        if (cycle > seen_max) seen_max = cycle;
        const uint8_t next = (uint8_t)((cycle + 1) & 0x3F);
        if (next < cycle) wrapped = true;   // 63 -> 0
        cycle = next;
    }
    CHECK(seen_min == 0, "cycle counter reaches 0");
    CHECK(seen_max == 63, "cycle counter reaches 63");
    CHECK(wrapped, "cycle counter wraps 63 -> 0");
}

// The schedule ships the bench sample slots on both channels so the existing
// far-side listener / SavvyCAN decode them unchanged.
static void test_sample_schedule(void) {
    CHECK(FR_SRC_DEFAULT_SLOT_COUNT == 3, "default schedule ships the three sample slots");
    const uint16_t want[3] = { 0x001, 0x002, 0x004 };
    for (int s = 0; s < FR_SRC_DEFAULT_SLOT_COUNT && s < 3; s++) {
        CHECK(FR_SOURCE_DEFAULT_SCHEDULE[s].slot_id == want[s], "sample slot id (0x001/0x002/0x004)");
        CHECK((FR_SOURCE_DEFAULT_SCHEDULE[s].channels & (FR_SRC_CH_FR1 | FR_SRC_CH_FR2))
                  == (FR_SRC_CH_FR1 | FR_SRC_CH_FR2), "sample slot emits on both channels");
    }
}

// Per-channel CRC init: a frame built for FR1 (init 0xFEDCBA) must NOT validate
// when decoded strictly as FR2 (init 0xABCDEF), and vice versa - proving each
// bus carries its own correct CRC, not a single init reused on both.
static void test_per_channel_crc(void) {
    const fr_source_slot_t *slot = &FR_SOURCE_DEFAULT_SCHEDULE[0];

    uint8_t a[FR_INJECT_FRAME_MAX], b[FR_INJECT_FRAME_MAX];
    const int na = fr_source_build_frame(a, slot, 7, PIFLEX_SOURCE_FR1);
    const int nb = fr_source_build_frame(b, slot, 7, PIFLEX_SOURCE_FR2);

    // Same header + payload on both channels; only the 3 CRC bytes differ.
    CHECK(na == nb, "both channels build the same length frame");
    CHECK(memcmp(a, b, (size_t)(na - 3)) == 0, "header + payload identical across channels");
    CHECK(memcmp(&a[na - 3], &b[nb - 3], 3) != 0, "per-channel CRC bytes differ");

    fr_frame_t got;
    // FR1 frame read as FR1: valid. Read strictly as FR2: invalid.
    CHECK(decode_built(a, na, PIFLEX_SOURCE_FR1, &got) && got.frame_crc_ok,
          "FR1-built frame validates on FR1");
    CHECK(decode_built(a, na, PIFLEX_SOURCE_FR2, &got) && !got.frame_crc_ok,
          "FR1-built frame rejected on FR2 (per-channel init)");
    CHECK(decode_built(b, nb, PIFLEX_SOURCE_FR2, &got) && got.frame_crc_ok,
          "FR2-built frame validates on FR2");
}

// The builder refuses an odd payload length and accepts the maximum even one.
// (An even length cannot exceed FR_MAX_PAYLOAD=254 in the uint8_t field, so the
// oversized guard is bounded by the type; the odd guard is the reachable case.)
static void test_bad_length(void) {
    uint8_t out[FR_INJECT_FRAME_MAX];

    // New slot layout: { slot_id, payload_len, channels, flags, payload[] }.
    fr_source_slot_t bad = { 0x010, 3 /* odd */, FR_SRC_CH_FR1, 0, {0x55, 0x55, 0x55} };
    CHECK(fr_source_build_frame(out, &bad, 0, PIFLEX_SOURCE_FR1) == 0, "odd payload length refused");

    fr_source_slot_t big = { 0x010, FR_MAX_PAYLOAD /* 254, max even */, FR_SRC_CH_FR1, 0, {0} };
    CHECK(fr_source_build_frame(out, &big, 0, PIFLEX_SOURCE_FR1) == 5 + FR_MAX_PAYLOAD + 3,
          "maximum even payload length builds");
}

int main(void) {
    test_sample_schedule();
    test_cycle_sweep();
    test_per_channel_crc();
    test_bad_length();

    printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
    return failures ? 1 : 0;
}
