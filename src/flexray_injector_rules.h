// PiFlex-FW - FlexRay injector rule table defaults (phase 4, RAM-backed phase 6).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The injection allow-list: the slots the MITM bridge is allowed to override, and
// - for each - which payload bytes may be replaced. SavvyCAN submits replacement
// bytes over the bulk-OUT injector protocol (op 0x90, see protocol.h); the
// firmware looks the slot up in the live rule table, verifies the submitted
// slice, and - when the bridge later forwards a frame whose slot ID and cycle
// match - splices the stored bytes into the payload and recomputes the 24-bit
// frame CRC before it reaches the bus (see flexray_injector.c / the TX path).
//
// Since phase 6 the rule table is RAM-resident and host-configurable at runtime
// (op 0x92 SET_INJECT_RULE / 0x93 CLEAR_INJECT_RULES), so changing injection
// targets no longer needs a reflash. The table below is the BOOT DEFAULT, copied
// into RAM by fr_inject_reset(); until the host reconfigures, behaviour is exactly
// the phase-4 sample behaviour. A slot with no rule can never be overridden: op
// 0x90 for an unlisted slot is dropped and counted (total_inject_drops). Injection
// is additionally gated by the global enable (op 0x91), which defaults OFF.
//
// RULE FIELDS
//   slot_id        11-bit FlexRay frame/slot ID to match (0..0x7FF).
//   cycle_base,    the cycle test. A frame is eligible when
//   cycle_mask       (frame.cycle & cycle_mask) == (cycle_base & cycle_mask).
//                  cycle_mask == 0 accepts ANY cycle (the phase-4 cycle_match=ANY
//                  default); a full mask (0x3F) with a cycle_base pins one cycle;
//                  a partial mask selects a subset (e.g. mask 0x01, base 0 = even
//                  cycles). The override op (0x90) still carries the specific
//                  cycle it fires on (cycle_base), and a forwarded frame is
//                  spliced only when its cycle equals that armed override cycle -
//                  so this generalises WHICH cycles a rule accepts, while the
//                  per-splice cycle targeting is unchanged from phase 4.
//   replace_offset first payload byte (0-based) the override overwrites.
//   replace_len    number of payload bytes overwritten (<= FR_INJ_MAX_REPLACE).
//                  The replace window is payload[replace_offset ..
//                  replace_offset+replace_len). The host submits a payload of at
//                  least replace_offset+replace_len bytes; only this window is
//                  spliced onto the bus. A frame whose payload is shorter than the
//                  window at forward time is left verbatim.
#ifndef PIFLEX_FLEXRAY_INJECTOR_RULES_H
#define PIFLEX_FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>

// cycle_mask value meaning "any cycle" (accept an override for every cycle).
#define FR_INJ_CYCLE_ANY_MASK  0x00u

// Largest replace window (bytes) the firmware stores per rule. Bounds the RAM
// held for pending overrides and the length of an accepted 0x90 window; a rule
// must not declare replace_len greater than this. 64 covers every realistic
// override (the SavvyCAN ACC example is 14) while keeping the override store
// small (FR_INJ_MAX_RULES * this).
#define FR_INJ_MAX_REPLACE 64u

// Fixed capacity of the RAM rule table. A SET_INJECT_RULE (0x92) that would
// exceed this is dropped (and counted internally) rather than crashing. 16 is
// ample for a bench / single-vehicle target set while bounding the RAM store
// (FR_INJ_MAX_RULES * (rule + pending override), a few KB).
#define FR_INJ_MAX_RULES   16

typedef struct {
    uint16_t slot_id;        // 11-bit slot/frame ID
    uint8_t  cycle_base;     // cycle match value (see cycle_mask)
    uint8_t  cycle_mask;     // 0 = any cycle; else (cycle & mask) == (base & mask)
    uint8_t  replace_offset; // first overwritten payload byte
    uint8_t  replace_len;    // number of overwritten payload bytes (<= FR_INJ_MAX_REPLACE)
} fr_inject_rule_t;

// --- Boot default rules ----------------------------------------------------
//
// SAMPLE / BENCH RULES (clearly marked): the three slots the bench signal
// generator emits on FR2 - 0x001 (payload 11 11 ..), 0x002 (22 22 ..), 0x004
// (44 44 ..) - so the injection bench can exercise the full path end to end. Each
// accepts an override for ANY cycle (cycle_mask 0) and overrides the first two
// payload bytes; the bench operator sends a SavvyCAN FlexRay frame for the slot
// (>=2-byte payload) and watches the first two bytes change on the targeted cycle.
//
// These are only the power-up defaults now; the host replaces them at runtime with
// op 0x92 for a real vehicle's target slots/windows.
static const fr_inject_rule_t FR_INJECT_DEFAULT_RULES[] = {
    // slot_id, cycle_base, cycle_mask,          replace_offset, replace_len
    { 0x001,   0,          FR_INJ_CYCLE_ANY_MASK, 0,             2 },   // SAMPLE (bench gen slot 0x001)
    { 0x002,   0,          FR_INJ_CYCLE_ANY_MASK, 0,             2 },   // SAMPLE (bench gen slot 0x002)
    { 0x004,   0,          FR_INJ_CYCLE_ANY_MASK, 0,             2 },   // SAMPLE (bench gen slot 0x004)
};

#define FR_INJ_DEFAULT_RULE_COUNT \
    ((int)(sizeof(FR_INJECT_DEFAULT_RULES) / sizeof(FR_INJECT_DEFAULT_RULES[0])))

#endif // PIFLEX_FLEXRAY_INJECTOR_RULES_H
