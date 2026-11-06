// PiFlex-FW - build a scheduled (generated) FlexRay frame (host-testable, no
// Pico SDK). SPDX-License-Identifier: GPL-3.0-or-later
//
// The maths of source mode: given a schedule slot (flexray_source_schedule.h), a
// channel and the current FlexRay cycle, produce the on-wire byte sequence of a
// complete, valid frame - a 5-byte header (slot ID, payload length, the 11-bit
// header CRC, and the cycle in its header field) followed by the generated
// payload and a freshly computed 24-bit frame CRC for that channel. The frame
// is built by reusing the phase-4 injector encoder (fr_inject_build_frame) as
// the transmit core, so the generator and the injector emit byte-identical
// framing. Kept free of any hardware/SDK include so it can be unit-tested on a
// host (see test/source_test.c); the TX emit path (flexray_source_tx.c /
// flexray_inject.pio) clocks these bytes onto the channel(s).
#ifndef PIFLEX_FLEXRAY_SOURCE_H
#define PIFLEX_FLEXRAY_SOURCE_H

#include <stdint.h>
#include "flexray_source_schedule.h"   // fr_source_slot_t

// Generate the payload for slot at cycle into buf (slot->payload_len bytes):
// every byte = slot->fill, then - if FR_SRC_FLAG_CYCLE_COUNTER - the last byte
// is overwritten with the cycle (0..63). buf must hold slot->payload_len bytes.
void fr_source_payload(uint8_t *buf, const fr_source_slot_t *slot, uint8_t cycle);

// Build the on-wire bytes for slot on channel (PIFLEX_SOURCE_FR1 / FR2) at cycle:
//   [5-byte header][generated payload][3-byte frame CRC, big-endian]
// The header carries slot_id, payload_len/2 words, the recomputed 11-bit header
// CRC and the 6-bit cycle; the 24-bit frame CRC uses the channel's init so each
// bus carries its own correct CRC. Returns the total byte count
// (5 + payload_len + 3), or 0 if the slot's payload_len is invalid (odd or
// greater than FR_MAX_PAYLOAD). out must hold FR_INJECT_FRAME_MAX bytes.
int fr_source_build_frame(uint8_t *out, const fr_source_slot_t *slot,
                          uint8_t cycle, uint8_t channel);

#endif // PIFLEX_FLEXRAY_SOURCE_H
