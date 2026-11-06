// PiFlex-FW - build an injected FlexRay frame (host-testable, no Pico SDK).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The maths of injection: given a decoded frame and a replace window, produce
// the on-wire byte sequence of the modified frame - the 5-byte header verbatim
// (payload-only edits do not change the 11-bit header CRC), the payload with the
// window spliced in, and a freshly recomputed 24-bit frame CRC so the injected
// frame is valid on the bus. Kept free of any hardware/SDK include so the splice
// and CRC recompute can be unit-tested on a host (see test/inject_build_test.c);
// the TX emit path (flexray_inject_tx / flexray_inject.pio) turns these bytes
// into FlexRay line coding on the destination channel.
#ifndef PIFLEX_FLEXRAY_INJECT_BUILD_H
#define PIFLEX_FLEXRAY_INJECT_BUILD_H

#include <stdint.h>
#include "flexray_decode.h"   // fr_frame_t, FR_MAX_PAYLOAD

// Largest built frame: 5-byte header + max payload + 3-byte frame CRC.
#define FR_INJECT_FRAME_MAX  (5 + FR_MAX_PAYLOAD + 3)

// Build the injected frame bytes for f into out (>= FR_INJECT_FRAME_MAX):
//   [5-byte header][payload, window spliced][3-byte frame CRC, big-endian]
// window/len/offset come from the matched injection rule (fr_inject_match). The
// window is spliced only if it fits within f's payload; the 24-bit frame CRC is
// recomputed over the modified header+payload using the channel init under which
// the original frame validated (so a single-source bench rig that emits one
// channel's CRC on the other bus is preserved). Returns the total byte count
// (5 + payload_len + 3), or 0 if the window does not fit the payload.
int fr_inject_build_frame(uint8_t *out, const fr_frame_t *f,
                          const uint8_t *window, uint8_t offset, uint8_t len);

#endif // PIFLEX_FLEXRAY_INJECT_BUILD_H
