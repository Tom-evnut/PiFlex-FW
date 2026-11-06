// PiFlex-FW - FlexRay injector control plane. SDK-free so it can be unit-tested
// on a host (see test/injector_test.c). Clean-room: the E2E CRC-8 is the AUTOSAR
// E2E Profile CRC (poly 0x1D), and the bulk-OUT framing is our own SavvyCAN
// interop contract (protocol.h / connections/picoflexray.cpp), not derived from
// any third-party firmware.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "flexray_injector.h"
#include "flexray_injector_rules.h"
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

// One pending override per rule (parallel to FR_INJECT_RULES). active is set
// once a valid 0x90 has been stored for the rule; cycle_base is the cycle the
// override fires on; data[0..len) is the carved replace window.
typedef struct {
    bool    active;
    uint8_t cycle_base;
    uint8_t len;
    uint8_t data[FR_INJ_MAX_REPLACE];
} fr_inject_override_t;

static fr_inject_override_t s_over[FR_INJ_RULE_COUNT];
static bool                 s_enabled = false;   // SAFETY: injector OFF at power-up

void fr_inject_reset(void) {
    s_enabled = false;
    memset(s_over, 0, sizeof s_over);
}

void fr_inject_set_enabled(bool enabled) { s_enabled = enabled; }
bool fr_inject_enabled(void)             { return s_enabled; }

// Find the rule index for a slot ID whose cycle_match accepts cycle_base, or -1.
static int find_rule(uint16_t slot_id, uint8_t cycle_base) {
    for (int i = 0; i < FR_INJ_RULE_COUNT; i++) {
        const fr_inject_rule_t *r = &FR_INJECT_RULES[i];
        if (r->slot_id != slot_id) continue;
        if (r->cycle_match != FR_INJ_CYCLE_ANY && r->cycle_match != cycle_base) continue;
        return i;
    }
    return -1;
}

// --- Bulk-OUT op handling --------------------------------------------------

// Handle a complete op-0x90 override of total length msg_len (op byte included).
// Verifies the E2E CRC-8 over the submitted payload and that it covers the
// rule's replace window, then stores the window. A slice for an unlisted slot /
// unaccepted cycle, a CRC mismatch, or a too-short payload is silently dropped
// (there is no host ACK for 0x90 either way).
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
    if (ri < 0) return;                        // no rule for this slot/cycle -> drop
    const fr_inject_rule_t *r = &FR_INJECT_RULES[ri];

    // The submitted payload must reach the whole replace window, and the window
    // must fit our per-rule store (a rule should never declare more, but guard).
    if (pay_len < r->replace_offset + r->replace_len) return;
    if (r->replace_len > FR_INJ_MAX_REPLACE)            return;

    // Carve the window out of the payload and arm the override.
    memcpy(s_over[ri].data, &payload[r->replace_offset], r->replace_len);
    s_over[ri].len        = r->replace_len;
    s_over[ri].cycle_base = cycle_base;
    s_over[ri].active     = true;
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
        default:
            return -1;                          // unknown op -> caller resyncs
    }
}

bool fr_inject_match(uint16_t slot_id, uint8_t cycle,
                     uint8_t *out_offset, uint8_t *out_len,
                     const uint8_t **out_data) {
    if (!s_enabled) return false;
    for (int i = 0; i < FR_INJ_RULE_COUNT; i++) {
        if (!s_over[i].active) continue;
        const fr_inject_rule_t *r = &FR_INJECT_RULES[i];
        if (r->slot_id != slot_id) continue;
        if (s_over[i].cycle_base != cycle) continue;
        if (out_offset) *out_offset = r->replace_offset;
        if (out_len)    *out_len    = s_over[i].len;
        if (out_data)   *out_data   = s_over[i].data;
        return true;
    }
    return false;
}
