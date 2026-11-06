// PiFlex-FW - build an injected FlexRay frame. SDK-free so the splice and the
// 24-bit frame-CRC recompute can be unit-tested on a host (see
// test/inject_build_test.c). Clean-room: reuses the FlexRay CRC from
// flexray_decode.c; not derived from any third-party firmware.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "flexray_inject_build.h"
#include "flexray_decode.h"
#include "protocol.h"
#include <string.h>

int fr_inject_build_frame(uint8_t *out, const fr_frame_t *f,
                          const uint8_t *window, uint8_t offset, uint8_t len) {
    // The window must fit entirely within the payload, or there is nothing valid
    // to inject - the caller then forwards the frame verbatim instead.
    if ((int)offset + (int)len > (int)f->payload_len) return 0;

    // 5-byte header, reconstructed from the decoded fields. A payload-only edit
    // does not touch any header field, so the 11-bit header CRC is unchanged and
    // the header bytes are reproduced exactly as received.
    const int words = f->payload_len / 2;
    out[0] = (uint8_t)((f->indicators << 3) | ((f->slot_id >> 8) & 0x07));
    out[1] = (uint8_t)(f->slot_id & 0xFF);
    out[2] = (uint8_t)((words << 1) | ((f->header_crc >> 10) & 0x01));
    out[3] = (uint8_t)((f->header_crc >> 2) & 0xFF);
    out[4] = (uint8_t)(((f->header_crc & 0x03) << 6) | (f->cycle & 0x3F));

    // Payload, then splice the replacement window over it.
    memcpy(&out[5], f->payload, f->payload_len);
    const int hp = 5 + f->payload_len;   // header + payload byte count

    // Choose the channel init under which the ORIGINAL frame's CRC validated, so
    // the rebuilt frame keeps the same channel convention the far side expects.
    // A proper two-bus install validates under its own channel; a single-source
    // bench rig that emits channel-A CRCs on bus B validates under the other
    // channel - this picks whichever matched before we touch the payload.
    uint8_t ch = f->channel;
    if (flexray_frame_crc(out, hp, ch) != f->frame_crc) {
        const uint8_t other = (ch == PIFLEX_SOURCE_FR2) ? PIFLEX_SOURCE_FR1 : PIFLEX_SOURCE_FR2;
        if (flexray_frame_crc(out, hp, other) == f->frame_crc) ch = other;
    }

    if (len) memcpy(&out[5 + offset], window, len);   // len 0 (verbatim rebuild): window may be NULL

    // Recompute the 24-bit frame CRC over the modified header+payload, MSB-first,
    // and append it big-endian (same on-wire order the decoder reads, before FES).
    const uint32_t crc = flexray_frame_crc(out, hp, ch);
    out[hp + 0] = (uint8_t)((crc >> 16) & 0xFF);
    out[hp + 1] = (uint8_t)((crc >> 8) & 0xFF);
    out[hp + 2] = (uint8_t)(crc & 0xFF);
    return hp + 3;
}
