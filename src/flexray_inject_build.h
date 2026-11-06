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

// TSS length (low bits) the transmitter emits. The receiver needs >= 3; the F11
// car uses 9, so a 9-bit TSS matches real traffic and the phase-3 mirror.
#define FR_INJECT_TSS_BITS   9

// On-wire bits of the largest frame: TSS + FSS(1) + 10 bits/byte (BSS "10" + 8
// data) + FES(2). Plus the leading count word, packed 32 bits/word, gives the
// word buffer the transmitter streams via DMA.
#define FR_INJECT_BITS_MAX   (FR_INJECT_TSS_BITS + 1 + 10 * FR_INJECT_FRAME_MAX + 2)
#define FR_INJECT_WORDS_MAX  (1 + (FR_INJECT_BITS_MAX + 31) / 32)

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

// Serialise the FlexRay line coding for a built frame (len bytes from
// fr_inject_build_frame) into PIO transmitter words. The whole framing lives
// here, in host-testable C, so the PIO is only a uniform bit shifter:
//   out[0]    = total_bits - 1  (the emitter's bit-loop counter)
//   out[1..]  = the on-wire bit stream, packed MSB-first (the first bit emitted
//               is bit 31 of out[1]); trailing bits of the last word are zero
//               and never emitted (the counter stops exactly at total_bits).
// Bit order: TSS (FR_INJECT_TSS_BITS lows), FSS (1 high), then per byte BSS "10"
// (high, low) + 8 data bits MSB-first, then FES "01" (low, high). Returns the
// number of words written (1 + ceil(total_bits/32)); out must hold
// FR_INJECT_WORDS_MAX words. Returns 0 if len is out of range.
int fr_inject_serialize(uint32_t *out, const uint8_t *frame, int len);

#endif // PIFLEX_FLEXRAY_INJECT_BUILD_H
