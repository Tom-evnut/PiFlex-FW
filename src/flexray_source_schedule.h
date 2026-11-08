// PiFlex-FW - FlexRay source / generator schedule (phase 5, RAM-backed phase 6).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The list of slots the board transmits in source mode (-DFR_SOURCE, see
// src/flexray_source_tx.c). Unlike the sniffer (phases 1-2), the MITM bridge
// (phase 3) or the injector (phase 4), source mode makes the board the ACTIVE
// transmitter: it walks this schedule once per FlexRay cycle and emits each slot's
// frame onto the configured channel(s), so a single Pi_Flex board can stand in for
// the bench signal generator.
//
// Since phase 6 the schedule is RAM-resident and host-configurable at runtime (op
// 0x94 SET_SOURCE_SLOT / 0x95 CLEAR_SOURCE_SCHEDULE, and 0x96 SET_SOURCE_PARAMS
// for the timing + run/stop), seeded at boot with the phase-5 sample slots so the
// standalone generator behaves exactly as before until the host reconfigures. The
// live RAM table and timing live in flexray_source_cfg.c; this header is the slot
// data type and the boot defaults.
//
// This is NOT a full FlexRay cluster: there is no coldstart / sync / NIT and no
// bus guardian. The board is assumed to be the sole source on the bench bus, so a
// free-running static-segment emitter with a 0..63 cycle counter is the goal.
//
// SLOT FIELDS
//   slot_id      11-bit FlexRay frame/slot ID to emit (0..0x7FF). Placed in the
//                header exactly as the sniffer/decoder reads it.
//   payload_len  payload bytes, even, 2..FR_MAX_PAYLOAD. The header length field
//                carries payload_len/2 words.
//   channels     bitmask of the channel(s) to emit this slot on:
//                FR_SRC_CH_FR1 (bus A, CRC init 0xFEDCBA) and/or
//                FR_SRC_CH_FR2 (bus B, CRC init 0xABCDEF). The frame CRC is
//                computed per channel, so a slot emitted on both buses carries a
//                different (correct) 24-bit CRC on each.
//   flags        FR_SRC_FLAG_CYCLE_COUNTER: overwrite the LAST payload byte with
//                the current cycle (0..63), so a listener watching only the
//                payload still sees a value advance and wrap. The header cycle
//                field advances regardless; this is an extra, in-payload tell.
//   payload      the payload bytes emitted (payload_len of them). The host sets
//                these per slot over op 0x94; the boot defaults fill a constant
//                "signature" byte per slot (the bench 11../22../44.. patterns).
#ifndef PIFLEX_FLEXRAY_SOURCE_SCHEDULE_H
#define PIFLEX_FLEXRAY_SOURCE_SCHEDULE_H

#include <stdint.h>
#include "flexray_decode.h"   // FR_MAX_PAYLOAD

// channels bitmask bits.
#define FR_SRC_CH_FR1  0x01u   // emit on FR1 / bus A (frame-CRC init 0xFEDCBA)
#define FR_SRC_CH_FR2  0x02u   // emit on FR2 / bus B (frame-CRC init 0xABCDEF)

// flags bits.
#define FR_SRC_FLAG_CYCLE_COUNTER  0x01u  // last payload byte = cycle (0..63)

// Fixed capacity of the RAM schedule. A SET_SOURCE_SLOT (0x94) that would exceed
// this (or carry an invalid payload_len) is dropped rather than crashing.
#define FR_SRC_MAX_SLOTS  16

typedef struct {
    uint16_t slot_id;                  // 11-bit slot/frame ID
    uint8_t  payload_len;              // payload bytes (even, 2..FR_MAX_PAYLOAD)
    uint8_t  channels;                 // FR_SRC_CH_FR1 | FR_SRC_CH_FR2
    uint8_t  flags;                    // FR_SRC_FLAG_CYCLE_COUNTER
    uint8_t  payload[FR_MAX_PAYLOAD];  // the payload bytes to emit
} fr_source_slot_t;

// --- Boot default schedule -------------------------------------------------
//
// SAMPLE / BENCH SCHEDULE (clearly marked): mirrors the three slots the bench
// signal generator emits - 0x001 (payload 11 11 ..), 0x002 (22 22 ..), 0x004
// (44 44 ..) - so the generator the other sessions already use is reproduced by
// the board and the existing far-side listener / SavvyCAN decode them unchanged.
// Each slot is emitted on BOTH channels with its correct per-channel CRC and
// carries the cycle in its last payload byte. These are only the power-up
// defaults now; the host replaces them at runtime with op 0x94.
static const fr_source_slot_t FR_SOURCE_DEFAULT_SCHEDULE[] = {
    // slot_id, payload_len, channels,                    flags,                      payload
    { 0x001, 8, FR_SRC_CH_FR1 | FR_SRC_CH_FR2, FR_SRC_FLAG_CYCLE_COUNTER, {0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11} }, // SAMPLE 0x001
    { 0x002, 8, FR_SRC_CH_FR1 | FR_SRC_CH_FR2, FR_SRC_FLAG_CYCLE_COUNTER, {0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22} }, // SAMPLE 0x002
    { 0x004, 8, FR_SRC_CH_FR1 | FR_SRC_CH_FR2, FR_SRC_FLAG_CYCLE_COUNTER, {0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44} }, // SAMPLE 0x004
};

#define FR_SRC_DEFAULT_SLOT_COUNT \
    ((int)(sizeof(FR_SOURCE_DEFAULT_SCHEDULE) / sizeof(FR_SOURCE_DEFAULT_SCHEDULE[0])))

#endif // PIFLEX_FLEXRAY_SOURCE_SCHEDULE_H
