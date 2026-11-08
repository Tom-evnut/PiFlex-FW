// PiFlex-FW - FlexRay injector control plane + bulk-OUT host-config framer.
// SDK-free so it can be unit-tested on a host (see test/injector_test.c and
// test/hostcfg_test.c). Clean-room: the E2E CRC-8 is the AUTOSAR E2E Profile CRC
// (poly 0x1D), and the bulk-OUT framing is our own SavvyCAN interop contract
// (protocol.h / connections/picoflexray.cpp), not derived from any third-party
// firmware.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "flexray_injector.h"
#include "flexray_injector_rules.h"
#include "flexray_source_cfg.h"
#include "protocol.h"
#include <string.h>

// --- AUTOSAR E2E CRC-8 -----------------------------------------------------

uint8_t fr_e2e_crc8(const uint8_t *data, int len, uint8_t init) {
    uint8_t crc = init;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x1Du) : (uint8_t)(crc << 1);
    }
    return crc;
}

// --- Injector state --------------------------------------------------------

// A RAM rule plus its one pending override. active is set once a valid 0x90 has
// been stored for the rule; cycle_base is the cycle the override fires on;
// data[0..len) is the carved replace window. Keeping the override next to the rule
// means clearing/replacing a rule drops its override with no index bookkeeping.
typedef struct {
    fr_inject_rule_t rule;
    bool             ov_active;
    uint8_t          ov_cycle;                 // the cycle the armed override fires on
    uint8_t          ov_len;
    uint8_t          ov_data[FR_INJ_MAX_REPLACE];
} fr_inject_entry_t;

static fr_inject_entry_t s_rules[FR_INJ_MAX_RULES];
static int               s_rule_count = 0;
static bool              s_enabled    = false;  // SAFETY: injector OFF at power-up

static uint32_t s_match_count = 0;              // frames selected for splice
static uint32_t s_drop_count  = 0;              // op-0x90 slices that matched no rule
static uint32_t s_rule_drops  = 0;              // 0x92 adds rejected (full / bad len)

// Seed the RAM rule table with the boot defaults (flexray_injector_rules.h).
static void seed_default_rules(void) {
    memset(s_rules, 0, sizeof s_rules);
    s_rule_count = 0;
    for (int i = 0; i < FR_INJ_DEFAULT_RULE_COUNT && s_rule_count < FR_INJ_MAX_RULES; i++) {
        s_rules[s_rule_count].rule = FR_INJECT_DEFAULT_RULES[i];
        s_rule_count++;
    }
}

void fr_inject_reset(void) {
    s_enabled     = false;
    s_match_count = 0;
    s_drop_count  = 0;
    s_rule_drops  = 0;
    seed_default_rules();
    fr_source_cfg_reset();   // restore the generator schedule + params to defaults
}

void fr_inject_set_enabled(bool enabled) { s_enabled = enabled; }
bool fr_inject_enabled(void)             { return s_enabled; }
int  fr_inject_rule_count(void)          { return s_rule_count; }
uint32_t fr_inject_match_count(void)     { return s_match_count; }
uint32_t fr_inject_drop_count(void)      { return s_drop_count; }

// A rule's cycle test: does this cycle satisfy the rule's cycle_base/cycle_mask?
// mask 0 -> any cycle. (Phase-4 cycle_match=ANY is mask 0; a full 0x3F mask pins
// one cycle; a partial mask selects a subset.)
static bool rule_cycle_ok(const fr_inject_rule_t *r, uint8_t cycle) {
    return ((cycle & r->cycle_mask) == (r->cycle_base & r->cycle_mask));
}

// Find the rule entry whose slot matches and whose cycle test accepts `cycle`, or
// -1. Used to locate the rule an op-0x90 override targets.
static int find_rule(uint16_t slot_id, uint8_t cycle) {
    for (int i = 0; i < s_rule_count; i++) {
        if (s_rules[i].rule.slot_id != slot_id) continue;
        if (!rule_cycle_ok(&s_rules[i].rule, cycle)) continue;
        return i;
    }
    return -1;
}

// Find a rule by its exact key (slot + cycle_base + cycle_mask), or -1.
static int find_rule_key(uint16_t slot_id, uint8_t cycle_base, uint8_t cycle_mask) {
    for (int i = 0; i < s_rule_count; i++) {
        if (s_rules[i].rule.slot_id == slot_id &&
            s_rules[i].rule.cycle_base == cycle_base &&
            s_rules[i].rule.cycle_mask == cycle_mask)
            return i;
    }
    return -1;
}

bool fr_inject_add_rule(uint16_t slot_id, uint8_t cycle_base, uint8_t cycle_mask,
                        uint8_t replace_offset, uint8_t replace_len) {
    if (replace_len == 0 || replace_len > FR_INJ_MAX_REPLACE) { s_rule_drops++; return false; }
    slot_id &= 0x7FFu;

    int i = find_rule_key(slot_id, cycle_base, cycle_mask);
    if (i < 0) {
        if (s_rule_count >= FR_INJ_MAX_RULES) { s_rule_drops++; return false; }  // full -> drop
        i = s_rule_count++;
        memset(&s_rules[i], 0, sizeof s_rules[i]);
    }
    s_rules[i].rule.slot_id        = slot_id;
    s_rules[i].rule.cycle_base     = cycle_base;
    s_rules[i].rule.cycle_mask     = cycle_mask;
    s_rules[i].rule.replace_offset = replace_offset;
    s_rules[i].rule.replace_len    = replace_len;
    // A changed window invalidates any pending override carved for the old window.
    s_rules[i].ov_active = false;
    s_rules[i].ov_len    = 0;
    return true;
}

void fr_inject_clear_rules(void) {
    memset(s_rules, 0, sizeof s_rules);
    s_rule_count = 0;
}

// --- Bulk-OUT op handling --------------------------------------------------

// Handle a complete op-0x90 override of total length msg_len (op byte included).
// Verifies the E2E CRC-8 over the submitted payload and that it covers the rule's
// replace window, then stores the window. A CRC mismatch or too-short payload is
// silently dropped; a slice that matches no rule is dropped AND counted so the
// host sees it via total_inject_drops.
static void handle_override(const uint8_t *buf, int msg_len) {
    // Layout: [0x90][id lo][id hi][cycle_base][sliceLen lo][sliceLen hi][slice]
    const uint16_t slot_id    = (uint16_t)(buf[1] | (buf[2] << 8));
    const uint8_t  cycle_base = buf[3];
    const int      slice_len  = buf[4] | (buf[5] << 8);
    const uint8_t *slice      = &buf[6];
    (void)msg_len;   // already validated complete by the caller

    if (slice_len < 2) return;                 // need CRC byte + >=1 payload byte

    const uint8_t  rx_crc  = slice[0];
    const uint8_t *payload = &slice[1];
    const int      pay_len = slice_len - 1;

    // E2E CRC-8 over the replacement payload only (init 0xF1), reject a mismatch.
    if (fr_e2e_crc8(payload, pay_len, FR_INJ_E2E_INIT) != rx_crc) return;

    const int ri = find_rule(slot_id, cycle_base);
    if (ri < 0) { s_drop_count++; return; }    // no rule for this slot/cycle -> drop + count
    fr_inject_entry_t *e = &s_rules[ri];

    // The submitted payload must reach the whole replace window, and the window
    // must fit our per-rule store (a rule should never declare more, but guard).
    if (pay_len < e->rule.replace_offset + e->rule.replace_len) return;
    if (e->rule.replace_len > FR_INJ_MAX_REPLACE)               return;

    // Carve the window out of the payload and arm the override on this cycle.
    memcpy(e->ov_data, &payload[e->rule.replace_offset], e->rule.replace_len);
    e->ov_len    = e->rule.replace_len;
    e->ov_cycle  = cycle_base;
    e->ov_active = true;
}

int fr_inject_consume(const uint8_t *buf, int len) {
    if (len < 1) return 0;                      // need at least the op byte

    switch (buf[0]) {
        case PIFLEX_OUT_ENABLE: {               // [0x91][enabled]
            if (len < 2) return 0;
            fr_inject_set_enabled(buf[1] != 0);
            return 2;
        }
        case PIFLEX_OUT_OVERRIDE: {             // [0x90][id][cycle][len][slice]
            if (len < 6) return 0;              // header not complete yet
            const int slice_len = buf[4] | (buf[5] << 8);
            const int msg_len   = 6 + slice_len;
            if (len < msg_len) return 0;        // slice not fully arrived yet
            handle_override(buf, msg_len);
            return msg_len;
        }
        case PIFLEX_OUT_SET_INJECT_RULE: {      // [0x92][slot][cyc_base][cyc_mask][off][len]
            if (len < 7) return 0;
            fr_inject_add_rule((uint16_t)(buf[1] | (buf[2] << 8)),
                               buf[3], buf[4], buf[5], buf[6]);
            return 7;
        }
        case PIFLEX_OUT_CLEAR_INJECT_RULES:     // [0x93]
            fr_inject_clear_rules();
            return 1;
        case PIFLEX_OUT_SET_SOURCE_SLOT: {      // [0x94][slot][chan][N][flags][payload x N]
            if (len < 5) return 0;              // need through payload_len to size the op
            const int n       = buf[4];
            const int msg_len = 6 + n;
            if (len < msg_len) return 0;
            fr_source_cfg_apply_slot(buf, msg_len);
            return msg_len;
        }
        case PIFLEX_OUT_CLEAR_SOURCE_SCHED:     // [0x95]
            fr_source_cfg_clear_schedule();
            return 1;
        case PIFLEX_OUT_SET_SOURCE_PARAMS:      // [0x96][period u32][gap u32][enable]
            if (len < 10) return 0;
            fr_source_cfg_apply_params(buf, 10);
            return 10;
        default:
            return -1;                          // unknown op -> caller resyncs
    }
}

bool fr_inject_match(uint16_t slot_id, uint8_t cycle,
                     uint8_t *out_offset, uint8_t *out_len,
                     const uint8_t **out_data) {
    if (!s_enabled) return false;
    for (int i = 0; i < s_rule_count; i++) {
        fr_inject_entry_t *e = &s_rules[i];
        if (!e->ov_active) continue;
        if (e->rule.slot_id != slot_id) continue;
        if (!rule_cycle_ok(&e->rule, cycle)) continue;   // frozen runtime cycle test
        if (e->ov_cycle != cycle) continue;              // armed override fires on its cycle
        if (out_offset) *out_offset = e->rule.replace_offset;
        if (out_len)    *out_len    = e->ov_len;
        if (out_data)   *out_data   = e->ov_data;
        s_match_count++;
        return true;
    }
    return false;
}

int fr_inject_build_status(uint8_t *out) {
    const uint32_t m = s_match_count;
    const uint32_t d = s_drop_count;
    out[0]  = PIFLEX_STATUS_MAGIC0;
    out[1]  = PIFLEX_STATUS_MAGIC1;
    out[2]  = PIFLEX_STATUS_VERSION;
    out[3]  = s_enabled ? 1u : 0u;
    out[4]  = (uint8_t)s_rule_count;
    out[5]  = (uint8_t)fr_source_cfg_slot_count();
    out[6]  = fr_source_cfg_running() ? 1u : 0u;
    const uint32_t sd = fr_source_cfg_slot_drops();   // 0 in a non-source build too
    out[7]  = (uint8_t)(sd > 255u ? 255u : sd);        // source_slot_drops, saturating
    out[8]  = (uint8_t)(m & 0xFF);
    out[9]  = (uint8_t)((m >> 8) & 0xFF);
    out[10] = (uint8_t)((m >> 16) & 0xFF);
    out[11] = (uint8_t)((m >> 24) & 0xFF);
    out[12] = (uint8_t)(d & 0xFF);
    out[13] = (uint8_t)((d >> 8) & 0xFF);
    out[14] = (uint8_t)((d >> 16) & 0xFF);
    out[15] = (uint8_t)((d >> 24) & 0xFF);
    return PIFLEX_STATUS_LEN;
}
