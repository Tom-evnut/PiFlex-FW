// PiFlex-FW - FlexRay frame decode core (host-testable, no Pico SDK).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The byte-level framing and CRC logic, kept free of any hardware/SDK include
// so it can be unit-tested on a host (see test/decode_test.c). The firmware's
// hardware glue (flexray_rx.c) feeds PIO words into fr_decode_feed().
#ifndef PIFLEX_FLEXRAY_DECODE_H
#define PIFLEX_FLEXRAY_DECODE_H

#include <stdint.h>
#include <stdbool.h>

#define FR_MAX_PAYLOAD  254         // 127 two-byte words
#define FR_START_FLAG   0x100u      // bit 8 of a PIO word: first byte of a frame

// A decoded FlexRay frame.
typedef struct {
    uint16_t slot_id;      // 11-bit frame / slot ID
    uint8_t  cycle;        // 6-bit cycle count
    uint8_t  indicators;   // reserved / payload-preamble / null / sync / startup
    uint16_t header_crc;   // 11-bit (as received)
    uint32_t frame_crc;    // 24-bit (as received)
    uint8_t  channel;      // PIFLEX_SOURCE_FR1 (bus A) or PIFLEX_SOURCE_FR2 (bus B)
    bool     header_crc_ok;
    bool     frame_crc_ok;
    uint8_t  payload_len;  // bytes (always even)
    uint8_t  payload[FR_MAX_PAYLOAD];
} fr_frame_t;

// Per-channel reassembly state.
typedef struct {
    uint8_t buf[5 + FR_MAX_PAYLOAD + 3];
    int     have;      // bytes collected so far
    int     need;      // total bytes expected once the header is parsed (0 = not yet)
    bool    in_frame;  // inside a frame (start seen, not yet complete)
} fr_decode_t;

void fr_decode_reset(fr_decode_t *st);

// Feed one PIO word (bit 8 = frame start, bits 7..0 = data byte, MSB first).
// Returns true and fills *out when a complete frame has been reassembled.
bool fr_decode_feed(fr_decode_t *st, uint32_t word, uint8_t channel, fr_frame_t *out);

// FlexRay CRCs (exposed for reuse and tests).
//   header: 11-bit, poly 0x385, init 0x1A, over {sync, startup, id[11], len[7]}
//   frame : 24-bit, poly 0x5D6DCB, init 0xFEDCBA (FR1) / 0xABCDEF (FR2)
uint16_t flexray_header_crc(uint8_t sync, uint8_t startup, uint16_t frame_id, uint8_t len_words);
uint32_t flexray_frame_crc(const uint8_t *data, int len, uint8_t channel);

#endif // PIFLEX_FLEXRAY_DECODE_H
