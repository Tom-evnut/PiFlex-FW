// PiFlex-FW - host test for the runtime host-config control plane (phase 6):
// the RAM injection rule table + generator schedule and their bulk-OUT ops
// (0x92-0x96) and the 0xA3 status, all parsed through the REAL fr_inject_consume
// framer so a protocol mistake is caught before the firmware reaches the bench.
//
// These bytes are exactly what SavvyCAN's PicoFlexRay host config writes to EP
// 0x03 (and reads back from control-IN 0xA3), so this is the firmware side of
// that interop contract. No Pico SDK or device needed.
//
// Build & run (host compiler, no Pico SDK needed):
//   gcc -I ../src hostcfg_test.c ../src/flexray_injector.c ../src/flexray_source_cfg.c -o hostcfg_test && ./hostcfg_test
#include "flexray_injector.h"
#include "flexray_injector_rules.h"
#include "flexray_source_cfg.h"
#include "flexray_source_schedule.h"
#include "protocol.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

// --- op builders (exactly the frozen protocol.h byte layouts) ---------------

static int build_rule(uint8_t *out, uint16_t slot, uint8_t cbase, uint8_t cmask,
                      uint8_t off, uint8_t len) {
    out[0] = PIFLEX_OUT_SET_INJECT_RULE;
    out[1] = (uint8_t)(slot & 0xFF);
    out[2] = (uint8_t)((slot >> 8) & 0xFF);
    out[3] = cbase; out[4] = cmask; out[5] = off; out[6] = len;
    return 7;
}

// op-0x90 override, as in injector_test.c: [0x90][id][cycle][sliceLen][CRC][payload].
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

static int build_src_slot(uint8_t *out, uint16_t slot, uint8_t chan, uint8_t flags,
                          const uint8_t *payload, int n) {
    out[0] = PIFLEX_OUT_SET_SOURCE_SLOT;
    out[1] = (uint8_t)(slot & 0xFF);
    out[2] = (uint8_t)((slot >> 8) & 0xFF);
    out[3] = chan; out[4] = (uint8_t)n; out[5] = flags;
    memcpy(&out[6], payload, (size_t)n);
    return 6 + n;
}

static int build_params(uint8_t *out, uint32_t period, uint32_t gap, uint8_t enable) {
    out[0] = PIFLEX_OUT_SET_SOURCE_PARAMS;
    out[1] = (uint8_t)(period); out[2] = (uint8_t)(period >> 8);
    out[3] = (uint8_t)(period >> 16); out[4] = (uint8_t)(period >> 24);
    out[5] = (uint8_t)(gap); out[6] = (uint8_t)(gap >> 8);
    out[7] = (uint8_t)(gap >> 16); out[8] = (uint8_t)(gap >> 24);
    out[9] = enable;
    return 10;
}

static void enable_injector(void) {
    const uint8_t on[] = { PIFLEX_OUT_ENABLE, 1 };
    fr_inject_consume(on, 2);
}

// --- tests -----------------------------------------------------------------

static void test_default_seed(void) {
    fr_inject_reset();
    CHECK(fr_inject_rule_count() == FR_INJ_DEFAULT_RULE_COUNT, "boot seeds the default inject rules");
    CHECK(fr_inject_rule_count() == 3, "three default inject rules (0x001/0x002/0x004)");
    CHECK(fr_source_cfg_slot_count() == FR_SRC_DEFAULT_SLOT_COUNT, "boot seeds the default schedule");
    CHECK(fr_source_cfg_slot_count() == 3, "three default source slots");
    CHECK(fr_source_cfg_period_us() == FR_SRC_DEFAULT_PERIOD_US, "default frame period seeded");
    CHECK(fr_source_cfg_enabled(), "generator enabled by default (phase-5 behaviour)");
    CHECK(fr_inject_match_count() == 0 && fr_inject_drop_count() == 0, "counters zero at boot");
}

// A rule configured over 0x92 splices exactly the bytes the old compile-time rule
// did: first two payload bytes on the matched cycle.
static void test_set_rule_matches_like_compiletime(void) {
    fr_inject_reset();
    enable_injector();
    const uint8_t pay[] = { 0xDE, 0xAD, 0xBE, 0xEF };

    // The seeded default for 0x001 (offset 0, len 2) is the compile-time baseline.
    uint8_t msg[64];
    int n = build_override(msg, 0x001, 5, pay, (int)sizeof pay);
    CHECK(fr_inject_consume(msg, n) == n, "override for a default rule is consumed");
    uint8_t off = 0xFF, len = 0xFF; const uint8_t *data = NULL;
    CHECK(fr_inject_match(0x001, 5, &off, &len, &data), "default (compile-time) rule matches");
    CHECK(off == 0 && len == 2 && data && data[0] == 0xDE && data[1] == 0xAD,
          "default rule splices the first two bytes");

    // A NEW slot configured at runtime with the same window behaves identically.
    n = build_rule(msg, 0x123, 0, FR_INJ_CYCLE_ANY_MASK, 0, 2);
    CHECK(fr_inject_consume(msg, n) == 7, "0x92 SET_INJECT_RULE consumes 7 bytes");
    CHECK(fr_inject_rule_count() == 4, "a new rule was added");
    n = build_override(msg, 0x123, 9, pay, (int)sizeof pay);
    CHECK(fr_inject_consume(msg, n) == n, "override for the new rule is consumed");
    off = 0xFF; len = 0xFF; data = NULL;
    CHECK(fr_inject_match(0x123, 9, &off, &len, &data), "runtime-configured rule matches");
    CHECK(off == 0 && len == 2 && data && data[0] == 0xDE && data[1] == 0xAD,
          "runtime rule splices the same bytes as the compile-time rule");

    // Replacing the same key updates the window and drops the pending override.
    n = build_rule(msg, 0x123, 0, FR_INJ_CYCLE_ANY_MASK, 1, 3);
    CHECK(fr_inject_consume(msg, n) == 7, "replace rule consumed");
    CHECK(fr_inject_rule_count() == 4, "replacing a key does not grow the table");
    CHECK(!fr_inject_match(0x123, 9, &off, &len, &data), "replacing a rule clears its armed override");
}

// cycle_mask semantics: a specific-cycle rule rejects a 0x90 for a non-matching
// cycle (counted as a drop), an any-cycle rule accepts a 0x90 for any cycle.
static void test_cycle_mask_semantics(void) {
    fr_inject_reset();
    fr_inject_clear_rules();
    enable_injector();
    const uint8_t pay[] = { 0x01, 0x02 };
    uint8_t msg[64];
    uint8_t off, len; const uint8_t *data;

    // Specific cycle 7 (full mask): accepts a 0x90 on cycle 7, drops one on cycle 8.
    int n = build_rule(msg, 0x200, 7, 0x3F, 0, 2);
    fr_inject_consume(msg, n);
    n = build_override(msg, 0x200, 7, pay, 2);
    CHECK(fr_inject_consume(msg, n) == n, "specific-cycle rule accepts its cycle");
    CHECK(fr_inject_match(0x200, 7, &off, &len, &data), "specific-cycle rule fires on cycle 7");
    CHECK(!fr_inject_match(0x200, 8, &off, &len, &data), "specific-cycle rule does not fire on cycle 8");

    const uint32_t drops_before = fr_inject_drop_count();
    n = build_override(msg, 0x200, 8, pay, 2);
    CHECK(fr_inject_consume(msg, n) == n, "a non-matching-cycle override is still consumed");
    CHECK(fr_inject_drop_count() == drops_before + 1, "override for a non-matching cycle is counted as a drop");

    // Any cycle (mask 0): accepts a 0x90 for any cycle.
    n = build_rule(msg, 0x201, 0, FR_INJ_CYCLE_ANY_MASK, 0, 2);
    fr_inject_consume(msg, n);
    n = build_override(msg, 0x201, 3, pay, 2);
    CHECK(fr_inject_consume(msg, n) == n, "any-cycle rule accepts cycle 3");
    CHECK(fr_inject_match(0x201, 3, &off, &len, &data), "any-cycle rule fires on its armed cycle 3");
    n = build_override(msg, 0x201, 50, pay, 2);
    CHECK(fr_inject_consume(msg, n) == n, "any-cycle rule also accepts cycle 50");
    CHECK(fr_inject_match(0x201, 50, &off, &len, &data), "any-cycle rule re-arms to cycle 50");
}

static void test_clear_inject_rules(void) {
    fr_inject_reset();
    enable_injector();
    uint8_t msg[8];
    const uint8_t clr[] = { PIFLEX_OUT_CLEAR_INJECT_RULES };
    CHECK(fr_inject_consume(clr, 1) == 1, "0x93 CLEAR_INJECT_RULES consumes 1 byte");
    CHECK(fr_inject_rule_count() == 0, "clear removes every rule");
    const uint8_t pay[] = { 0xAA, 0xBB };
    int n = build_override(msg, 0x001, 0, pay, 2);
    (void)n;
    uint8_t off, len; const uint8_t *data;
    CHECK(!fr_inject_match(0x001, 0, &off, &len, &data), "nothing matches after a clear");
}

static void test_source_slot_ops(void) {
    fr_inject_reset();
    const uint8_t clr[] = { PIFLEX_OUT_CLEAR_SOURCE_SCHED };
    CHECK(fr_inject_consume(clr, 1) == 1, "0x95 CLEAR_SOURCE_SCHEDULE consumes 1 byte");
    CHECK(fr_source_cfg_slot_count() == 0, "clear empties the schedule");

    const uint8_t p4[] = { 0x10, 0x20, 0x30, 0x40 };
    uint8_t msg[300];
    int n = build_src_slot(msg, 0x010, FR_SRC_CH_FR1 | FR_SRC_CH_FR2,
                           FR_SRC_FLAG_CYCLE_COUNTER, p4, 4);
    CHECK(fr_inject_consume(msg, n) == n, "0x94 SET_SOURCE_SLOT consumes 6+N bytes");
    CHECK(fr_source_cfg_slot_count() == 1, "one slot after a set");
    const fr_source_slot_t *s = fr_source_cfg_slot(0);
    CHECK(s && s->slot_id == 0x010 && s->payload_len == 4, "slot id + length stored");
    CHECK(s && s->channels == (FR_SRC_CH_FR1 | FR_SRC_CH_FR2), "channel mask stored");
    CHECK(s && (s->flags & FR_SRC_FLAG_CYCLE_COUNTER), "flags stored");
    CHECK(s && memcmp(s->payload, p4, 4) == 0, "payload bytes stored");

    // Replace the same slot id (new length) - no growth.
    const uint8_t p6[] = { 1, 2, 3, 4, 5, 6 };
    n = build_src_slot(msg, 0x010, FR_SRC_CH_FR1, 0, p6, 6);
    CHECK(fr_inject_consume(msg, n) == n, "replacement slot consumed");
    CHECK(fr_source_cfg_slot_count() == 1, "replacing a slot id does not grow the table");
    s = fr_source_cfg_slot(0);
    CHECK(s && s->payload_len == 6, "replacement length applied");

    // Odd payload length is rejected (not stored).
    const uint8_t p3[] = { 9, 9, 9 };
    n = build_src_slot(msg, 0x011, FR_SRC_CH_FR1, 0, p3, 3);
    CHECK(fr_inject_consume(msg, n) == n, "odd-length slot op is still consumed (framed)");
    CHECK(fr_source_cfg_slot_count() == 1, "odd-length slot is dropped, not stored");

    // A second valid slot grows the table.
    n = build_src_slot(msg, 0x012, FR_SRC_CH_FR2, 0, p4, 4);
    fr_inject_consume(msg, n);
    CHECK(fr_source_cfg_slot_count() == 2, "a second slot id grows the table");
}

static void test_source_params(void) {
    fr_inject_reset();
    uint8_t msg[16];
    int n = build_params(msg, 1000, 250, 0);
    CHECK(fr_inject_consume(msg, n) == 10, "0x96 SET_SOURCE_PARAMS consumes 10 bytes");
    CHECK(fr_source_cfg_period_us() == 1000, "period applied");
    CHECK(fr_source_cfg_cycle_gap_us() == 250, "cycle gap applied");
    CHECK(!fr_source_cfg_enabled(), "enable=0 stops the generator");
    n = build_params(msg, 300, 0, 1);
    fr_inject_consume(msg, n);
    CHECK(fr_source_cfg_period_us() == 300 && fr_source_cfg_enabled(), "params re-applied, re-enabled");
}

// A 0x94 delivered one byte short asks for more, then completes - the framer
// sizes it by payload_len, like 0x90.
static void test_fragmented_source_slot(void) {
    fr_inject_reset();
    const uint8_t clr[] = { PIFLEX_OUT_CLEAR_SOURCE_SCHED };
    fr_inject_consume(clr, 1);
    const uint8_t p4[] = { 1, 2, 3, 4 };
    uint8_t msg[32];
    int n = build_src_slot(msg, 0x0AA, FR_SRC_CH_FR1, 0, p4, 4);
    CHECK(fr_inject_consume(msg, n - 1) == 0, "partial 0x94 asks for more bytes");
    CHECK(fr_inject_consume(msg, n) == n, "complete 0x94 is then consumed");
    CHECK(fr_source_cfg_slot_count() == 1, "fragmented slot stored once complete");
}

static void test_status_layout(void) {
    CHECK(sizeof(piflex_inject_status_t) == PIFLEX_STATUS_LEN, "status struct is exactly 16 bytes");
    CHECK(offsetof(piflex_inject_status_t, total_inject_matches) == 8, "matches counter at byte 8");
    CHECK(offsetof(piflex_inject_status_t, total_inject_drops) == 12, "drops counter at byte 12");

    fr_inject_reset();
    enable_injector();

    // Arm a default rule and fire it once -> match counter = 1.
    const uint8_t pay[] = { 0x5A, 0x5B };
    uint8_t msg[64];
    int n = build_override(msg, 0x002, 4, pay, 2);
    fr_inject_consume(msg, n);
    uint8_t off, len; const uint8_t *data;
    CHECK(fr_inject_match(0x002, 4, &off, &len, &data), "armed default rule fires");

    // An override for an unlisted slot -> drop counter = 1.
    n = build_override(msg, 0x7AB, 0, pay, 2);
    fr_inject_consume(msg, n);

    uint8_t st[PIFLEX_STATUS_LEN];
    const int sn = fr_inject_build_status(st);
    CHECK(sn == PIFLEX_STATUS_LEN, "status reply is 16 bytes");
    CHECK(st[0] == PIFLEX_STATUS_MAGIC0 && st[1] == PIFLEX_STATUS_MAGIC1, "status magic 'PF'");
    CHECK(st[2] == PIFLEX_STATUS_VERSION, "status version = 1");
    CHECK(st[3] == 1, "injector_enabled reported");
    CHECK(st[4] == (uint8_t)fr_inject_rule_count(), "inject_rule_count reported");
    CHECK(st[5] == (uint8_t)fr_source_cfg_slot_count(), "source_slot_count reported");
    CHECK(st[6] == 0, "source_running = 0 in a non-FR_SOURCE host build");
    CHECK(st[7] == 0, "reserved byte is zero");

    const uint32_t m = (uint32_t)st[8] | ((uint32_t)st[9] << 8) |
                       ((uint32_t)st[10] << 16) | ((uint32_t)st[11] << 24);
    const uint32_t d = (uint32_t)st[12] | ((uint32_t)st[13] << 8) |
                       ((uint32_t)st[14] << 16) | ((uint32_t)st[15] << 24);
    CHECK(m == fr_inject_match_count() && m == 1, "total_inject_matches LE = 1");
    CHECK(d == fr_inject_drop_count() && d == 1, "total_inject_drops LE = 1");
}

int main(void) {
    test_default_seed();
    test_set_rule_matches_like_compiletime();
    test_cycle_mask_semantics();
    test_clear_inject_rules();
    test_source_slot_ops();
    test_source_params();
    test_fragmented_source_slot();
    test_status_layout();

    printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
    return failures ? 1 : 0;
}
