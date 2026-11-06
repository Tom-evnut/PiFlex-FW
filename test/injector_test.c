// PiFlex-FW - host test for the SDK-free injector control plane
// (flexray_injector.c). Exercises the AUTOSAR E2E CRC-8, the bulk-OUT framing
// of ops 0x90/0x91, rule matching against flexray_injector_rules.h, and the
// TX-path query - all without a Pico SDK or a device, so a protocol mistake is
// caught before the firmware reaches the bench.
//
// The op-0x90 bytes built here are exactly what SavvyCAN's
// PicoFlexRay::overridePayloadFor() writes to EP 0x03 (connections/picoflexray.cpp
// and tests/tst_picoflexray_inject.cpp), so this is the firmware side of that
// interop contract.
//
// Build & run (host compiler, no Pico SDK needed):
//   gcc -I ../src injector_test.c ../src/flexray_injector.c ../src/flexray_source_cfg.c -o injector_test && ./injector_test
// (flexray_source_cfg.c is linked because the shared EP 0x03 framer / reset now
//  also drives the generator config; this test still only exercises 0x90/0x91.)
#include "flexray_injector.h"
#include "flexray_injector_rules.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

// Independent reimplementation of the AUTOSAR E2E CRC-8 (poly 0x1D), matching
// the reference in SavvyCAN's tst_picoflexray_inject.cpp, to check fr_e2e_crc8.
static uint8_t ref_crc8(const uint8_t *data, int len, uint8_t init) {
    uint8_t crc = init;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x1Du) : (uint8_t)(crc << 1);
    }
    return crc;
}

// Build an op-0x90 override exactly as SavvyCAN's overridePayloadFor() does:
//   [0x90][id lo][id hi][cycle_base][sliceLen lo][sliceLen hi][E2E CRC-8][payload]
// slice = CRC byte + payload; sliceLen = 1 + pay_len; CRC over the payload only.
// Returns the total message length.
static int build_override(uint8_t *out, uint16_t id, uint8_t cycle,
                          const uint8_t *payload, int pay_len) {
    const int slice_len = 1 + pay_len;
    out[0] = PIFLEX_OUT_OVERRIDE;
    out[1] = (uint8_t)(id & 0xFF);
    out[2] = (uint8_t)((id >> 8) & 0xFF);
    out[3] = cycle;
    out[4] = (uint8_t)(slice_len & 0xFF);
    out[5] = (uint8_t)((slice_len >> 8) & 0xFF);
    out[6] = fr_e2e_crc8(payload, pay_len, FR_INJ_E2E_INIT);
    memcpy(&out[7], payload, (size_t)pay_len);
    return 6 + slice_len;
}

static void test_crc8(void) {
    // Vectors from SavvyCAN's tst_picoflexray_inject.cpp::crc8MatchesFirmware.
    CHECK(fr_e2e_crc8(NULL, 0, 0xF1) == 0xF1, "E2E CRC-8 of no data is the init value");
    const uint8_t one[] = {0x01};
    CHECK(fr_e2e_crc8(one, 1, 0x00) == 0x1D, "E2E CRC-8 poly first entry (0x01, init 0 -> 0x1D)");
    const uint8_t d[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    CHECK(fr_e2e_crc8(d, 5, 0xF1) == ref_crc8(d, 5, 0xF1), "E2E CRC-8 matches reference");
}

static void test_enable(void) {
    fr_inject_reset();
    CHECK(!fr_inject_enabled(), "injector defaults OFF after reset");

    const uint8_t on[]  = {PIFLEX_OUT_ENABLE, 1};
    const uint8_t off[] = {PIFLEX_OUT_ENABLE, 0};
    CHECK(fr_inject_consume(on, 2) == 2, "0x91 enable consumes 2 bytes");
    CHECK(fr_inject_enabled(), "0x91 [1] enables");
    CHECK(fr_inject_consume(off, 2) == 2, "0x91 disable consumes 2 bytes");
    CHECK(!fr_inject_enabled(), "0x91 [0] disables");

    // Incomplete / unknown framing.
    CHECK(fr_inject_consume(on, 1) == 0, "0x91 with only the op byte asks for more");
    const uint8_t junk[] = {0x42, 0x00};
    CHECK(fr_inject_consume(junk, 2) < 0, "unknown op asks the caller to resync");
}

static void test_override_accept_and_match(void) {
    fr_inject_reset();

    // Rule 0 is slot 0x001, cycle ANY, replace offset 0 len 2. Submit a 4-byte
    // payload on cycle 5; the window (first 2 bytes) should be stored.
    const uint8_t payload[] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t msg[64];
    const int n = build_override(msg, 0x001, 5, payload, (int)sizeof payload);
    CHECK(fr_inject_consume(msg, n) == n, "valid 0x90 consumes the whole message");

    uint8_t off = 0xFF, len = 0xFF;
    const uint8_t *data = NULL;

    // Injector still OFF -> no match even though the override is stored.
    CHECK(!fr_inject_match(0x001, 5, &off, &len, &data), "no match while injector disabled");

    const uint8_t on[] = {PIFLEX_OUT_ENABLE, 1};
    fr_inject_consume(on, 2);

    CHECK(fr_inject_match(0x001, 5, &off, &len, &data), "match once enabled");
    CHECK(off == 0 && len == 2, "replace window is offset 0 len 2");
    CHECK(data && data[0] == 0xAA && data[1] == 0xBB, "stored window is the first 2 payload bytes");

    // Wrong cycle / wrong slot do not match.
    CHECK(!fr_inject_match(0x001, 6, &off, &len, &data), "cycle 6 does not match a cycle-5 override");
    CHECK(!fr_inject_match(0x003, 5, &off, &len, &data), "unlisted slot 0x003 never matches");
}

static void test_override_rejections(void) {
    fr_inject_reset();
    const uint8_t on[] = {PIFLEX_OUT_ENABLE, 1};
    fr_inject_consume(on, 2);

    uint8_t off, len; const uint8_t *data;
    const uint8_t payload[] = {0x10, 0x20, 0x30};

    // Bad E2E CRC -> dropped (still consumed so the stream stays framed).
    uint8_t bad[64];
    int n = build_override(bad, 0x002, 3, payload, (int)sizeof payload);
    bad[6] ^= 0xFF;                               // corrupt the CRC byte
    CHECK(fr_inject_consume(bad, n) == n, "CRC-mismatch 0x90 is still fully consumed");
    CHECK(!fr_inject_match(0x002, 3, &off, &len, &data), "CRC-mismatch override is not stored");

    // Unlisted slot -> dropped.
    uint8_t other[64];
    n = build_override(other, 0x555, 0, payload, (int)sizeof payload);
    CHECK(fr_inject_consume(other, n) == n, "override for an unlisted slot is consumed");
    CHECK(!fr_inject_match(0x555, 0, &off, &len, &data), "override for an unlisted slot is not stored");

    // Payload shorter than the rule's replace window (rule needs >=2) -> dropped.
    const uint8_t one[] = {0x77};
    uint8_t shortmsg[64];
    n = build_override(shortmsg, 0x004, 0, one, 1);
    CHECK(fr_inject_consume(shortmsg, n) == n, "too-short override is consumed");
    CHECK(!fr_inject_match(0x004, 0, &off, &len, &data), "too-short override is not stored");
}

static void test_fragmented_stream(void) {
    fr_inject_reset();
    const uint8_t on[] = {PIFLEX_OUT_ENABLE, 1};
    fr_inject_consume(on, 2);

    const uint8_t payload[] = {0x01, 0x02};
    uint8_t msg[64];
    const int n = build_override(msg, 0x002, 7, payload, (int)sizeof payload);

    // Delivered one byte short, then complete: first call asks for more, second consumes.
    CHECK(fr_inject_consume(msg, n - 1) == 0, "partial 0x90 asks for more bytes");
    CHECK(fr_inject_consume(msg, n) == n, "complete 0x90 is then consumed");

    uint8_t off, len; const uint8_t *data;
    CHECK(fr_inject_match(0x002, 7, &off, &len, &data), "fragmented override stored once complete");
    CHECK(data && data[0] == 0x01 && data[1] == 0x02, "fragmented override has the right bytes");
}

int main(void) {
    test_crc8();
    test_enable();
    test_override_accept_and_match();
    test_override_rejections();
    test_fragmented_stream();

    printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
    return failures ? 1 : 0;
}
