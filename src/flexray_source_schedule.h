// PiFlex-FW - FlexRay source / generator schedule table (phase 5).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The static, compiled-in list of slots the board transmits in source mode
// (-DFR_SOURCE, see src/flexray_source_tx.c). Unlike the sniffer (phases 1-2),
// the MITM bridge (phase 3) or the injector (phase 4), source mode makes the
// board the ACTIVE transmitter: it walks this table once per FlexRay cycle and
// emits each slot's frame onto the configured channel(s), so a single Pi_Flex
// board can stand in for the bench signal generator.
//
// This is NOT a full FlexRay cluster: there is no coldstart / sync / NIT and no
// bus guardian. The board is assumed to be the sole source on the bench bus, so
// a free-running static-segment emitter with a 0..63 cycle counter is the goal.
// The timing (slot cadence, inter-frame gap) lives in config.h as named
// #defines so it is easy to trim on the bench; this header is data only.
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
//   fill         the byte every payload position is filled with (a constant,
//                readable "signature" per slot, like the bench generator's
//                11../22../44.. patterns).
//   flags        FR_SRC_FLAG_CYCLE_COUNTER: overwrite the LAST payload byte with
//                the current cycle (0..63), so a listener watching only the
//                payload still sees a value advance and wrap. The header cycle
//                field advances regardless; this is an extra, in-payload tell.
#ifndef PIFLEX_FLEXRAY_SOURCE_SCHEDULE_H
#define PIFLEX_FLEXRAY_SOURCE_SCHEDULE_H

#include <stdint.h>

// channels bitmask bits.
#define FR_SRC_CH_FR1  0x01u   // emit on FR1 / bus A (frame-CRC init 0xFEDCBA)
#define FR_SRC_CH_FR2  0x02u   // emit on FR2 / bus B (frame-CRC init 0xABCDEF)

// flags bits.
#define FR_SRC_FLAG_CYCLE_COUNTER  0x01u  // last payload byte = cycle (0..63)

typedef struct {
    uint16_t slot_id;      // 11-bit slot/frame ID
    uint8_t  payload_len;  // payload bytes (even, 2..FR_MAX_PAYLOAD)
    uint8_t  channels;     // FR_SRC_CH_FR1 | FR_SRC_CH_FR2
    uint8_t  fill;         // payload fill byte
    uint8_t  flags;        // FR_SRC_FLAG_CYCLE_COUNTER
} fr_source_slot_t;

// --- The schedule ----------------------------------------------------------
//
// SAMPLE / BENCH SCHEDULE (clearly marked): mirrors the three slots the bench
// signal generator emits - 0x001 (payload 11 11 ..), 0x002 (22 22 ..), 0x004
// (44 44 ..) - so the generator the other sessions already use is reproduced by
// the board and the existing far-side listener / SavvyCAN decode them unchanged.
// Each slot is emitted on BOTH channels (FR1 and FR2) with its correct
// per-channel CRC, and carries the cycle in its last payload byte so changing
// data is visible even without reading the header cycle field.
//
// Replace with a vehicle's real static-segment schedule before field use; this
// table is the only place the generated slots are defined.
static const fr_source_slot_t FR_SOURCE_SCHEDULE[] = {
    // slot_id, payload_len, channels,                    fill, flags
    { 0x001,    8,           FR_SRC_CH_FR1 | FR_SRC_CH_FR2, 0x11, FR_SRC_FLAG_CYCLE_COUNTER }, // SAMPLE (bench slot 0x001)
    { 0x002,    8,           FR_SRC_CH_FR1 | FR_SRC_CH_FR2, 0x22, FR_SRC_FLAG_CYCLE_COUNTER }, // SAMPLE (bench slot 0x002)
    { 0x004,    8,           FR_SRC_CH_FR1 | FR_SRC_CH_FR2, 0x44, FR_SRC_FLAG_CYCLE_COUNTER }, // SAMPLE (bench slot 0x004)
};

#define FR_SOURCE_SLOT_COUNT ((int)(sizeof(FR_SOURCE_SCHEDULE) / sizeof(FR_SOURCE_SCHEDULE[0])))

#endif // PIFLEX_FLEXRAY_SOURCE_SCHEDULE_H
