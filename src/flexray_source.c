// PiFlex-FW - build a scheduled (generated) FlexRay frame. SDK-free so the
// payload generation and CRC maths can be unit-tested on a host (see
// test/source_test.c). Clean-room: reuses this repo's own FlexRay CRCs
// (flexray_decode.c) and injector encoder (flexray_inject_build.c); not derived
// from any third-party firmware.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "flexray_source.h"
#include "flexray_inject_build.h"   // fr_inject_build_frame, FR_INJECT_FRAME_MAX
#include "flexray_decode.h"         // fr_frame_t, flexray_*_crc, FR_MAX_PAYLOAD
#include "protocol.h"
#include <string.h>

void fr_source_payload(uint8_t *buf, const fr_source_slot_t *slot, uint8_t cycle) {
    memcpy(buf, slot->payload, slot->payload_len);
    if ((slot->flags & FR_SRC_FLAG_CYCLE_COUNTER) && slot->payload_len > 0)
        buf[slot->payload_len - 1] = (uint8_t)(cycle & 0x3F);
}

int fr_source_build_frame(uint8_t *out, const fr_source_slot_t *slot,
                          uint8_t cycle, uint8_t channel) {
    // A FlexRay payload is a whole number of two-byte words, so the length must
    // be even and within the decoder's maximum.
    if (slot->payload_len > FR_MAX_PAYLOAD || (slot->payload_len & 1u)) return 0;

    // Assemble a self-consistent decoded frame, then hand it to the injector's
    // verbatim rebuild (window len 0) to serialise the on-wire bytes - the same
    // encoder the phase-4 injector uses, so generator and injector framing match.
    fr_frame_t f;
    memset(&f, 0, sizeof f);
    f.channel     = channel;
    f.indicators  = 0;                         // plain data frame: no sync/startup
    f.slot_id     = (uint16_t)(slot->slot_id & 0x7FFu);
    f.cycle       = (uint8_t)(cycle & 0x3Fu);
    f.payload_len = slot->payload_len;
    fr_source_payload(f.payload, slot, f.cycle);

    const uint8_t words = (uint8_t)(slot->payload_len / 2);
    f.header_crc = flexray_header_crc(0, 0, f.slot_id, words);

    // Compute the frame CRC under THIS channel's init and store it, so the
    // injector encoder keeps this channel (it picks the init under which the
    // supplied frame_crc already validates) and the emitted CRC is the channel's.
    uint8_t hdr[5 + FR_MAX_PAYLOAD];
    hdr[0] = (uint8_t)((f.indicators << 3) | ((f.slot_id >> 8) & 0x07));
    hdr[1] = (uint8_t)(f.slot_id & 0xFF);
    hdr[2] = (uint8_t)((words << 1) | ((f.header_crc >> 10) & 0x01));
    hdr[3] = (uint8_t)((f.header_crc >> 2) & 0xFF);
    hdr[4] = (uint8_t)(((f.header_crc & 0x03) << 6) | (f.cycle & 0x3F));
    memcpy(&hdr[5], f.payload, f.payload_len);
    f.frame_crc = flexray_frame_crc(hdr, 5 + f.payload_len, channel);

    return fr_inject_build_frame(out, &f, NULL, 0, 0);
}
