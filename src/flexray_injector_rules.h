// PiFlex-FW - FlexRay injector rule table (phase 4).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The static, compiled-in list of slots the MITM bridge is allowed to override,
// and - for each - which bytes of the payload may be replaced. SavvyCAN submits
// replacement bytes over the bulk-OUT injector protocol (op 0x90, see
// protocol.h); the firmware looks the slot up here, verifies the submitted
// slice, and - when the bridge later forwards a frame whose slot ID and cycle
// match - splices the stored bytes into the payload and recomputes the 24-bit
// frame CRC before it reaches the bus (see flexray_injector.c / the TX path).
//
// A slot not listed here can never be overridden: op 0x90 for an unlisted slot
// is silently dropped. This is the safety allow-list - injection is confined to
// the windows named below, and is additionally gated by the global injector
// enable (op 0x91), which defaults OFF at power-up.
//
// RULE FIELDS
//   slot_id        11-bit FlexRay frame/slot ID to match (0..0x7FF).
//   cycle_match    which FlexRay cycle(s) this rule accepts an override for:
//                  a value 0..63 restricts the rule to that one cycle; the
//                  sentinel FR_INJ_CYCLE_ANY (0xFF) accepts any cycle. The
//                  override op (0x90) still carries the specific cycle the host
//                  wants overridden (cycle_base), and the bridge only injects on
//                  a forwarded frame whose cycle equals that stored cycle_base -
//                  so to override several cycles the host submits one 0x90 per
//                  cycle. cycle_match just bounds which cycles a rule will take.
//   replace_offset first payload byte (0-based) the override overwrites.
//   replace_len    number of payload bytes overwritten. The replace window is
//                  payload[replace_offset .. replace_offset+replace_len). The
//                  host must submit a payload of at least replace_offset+
//                  replace_len bytes (SavvyCAN sends the whole slot payload);
//                  only this window is actually spliced onto the bus. A frame
//                  whose payload is shorter than the window at forward time is
//                  left verbatim (the window would not fit).
#ifndef PIFLEX_FLEXRAY_INJECTOR_RULES_H
#define PIFLEX_FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>

// cycle_match sentinel: a rule that accepts an override for any FlexRay cycle.
#define FR_INJ_CYCLE_ANY   0xFFu

// Largest replace window (bytes) the firmware stores per rule. Bounds the RAM
// held for pending overrides and the length of an accepted 0x90 window; a rule
// must not declare replace_len greater than this. 64 covers every realistic
// override (the SavvyCAN ACC example is 14) while keeping the override store
// small (FR_INJ_MAX_RULES * this).
#define FR_INJ_MAX_REPLACE 64u

typedef struct {
    uint16_t slot_id;        // 11-bit slot/frame ID
    uint8_t  cycle_match;    // 0..63 exact, or FR_INJ_CYCLE_ANY
    uint8_t  replace_offset; // first overwritten payload byte
    uint8_t  replace_len;    // number of overwritten payload bytes (<= FR_INJ_MAX_REPLACE)
} fr_inject_rule_t;

// --- The rule table --------------------------------------------------------
//
// SAMPLE / BENCH RULES (clearly marked): the three slots the bench signal
// generator emits on FR2 - 0x001 (payload 11 11 ..), 0x002 (22 22 ..), 0x004
// (44 44 ..), across cycles 0..63 - so the injection bench can exercise the
// full path end to end. Each overrides the first two payload bytes on any
// cycle; the bench operator sends a SavvyCAN FlexRay frame for the slot with a
// >=2-byte payload and watches the generator's far-side FR1 listener see the
// first two bytes change (e.g. 11 11 -> AA BB) while the rest stays 11.
//
// Replace with the real target slots/windows for a given vehicle before field
// use; this table is the only place injection targets are defined.
static const fr_inject_rule_t FR_INJECT_RULES[] = {
    // slot_id, cycle_match,        replace_offset, replace_len
    { 0x001,   FR_INJ_CYCLE_ANY,    0,              2 },   // SAMPLE (bench gen slot 0x001)
    { 0x002,   FR_INJ_CYCLE_ANY,    0,              2 },   // SAMPLE (bench gen slot 0x002)
    { 0x004,   FR_INJ_CYCLE_ANY,    0,              2 },   // SAMPLE (bench gen slot 0x004)
};

#define FR_INJ_RULE_COUNT ((int)(sizeof(FR_INJECT_RULES) / sizeof(FR_INJECT_RULES[0])))

#endif // PIFLEX_FLEXRAY_INJECTOR_RULES_H
