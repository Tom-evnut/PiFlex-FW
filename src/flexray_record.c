// PiFlex-FW - serialise a decoded FlexRay frame into a host (SavvyCAN) record.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Layout must match SavvyCAN's PicoFlexRayParser (connections/picoflexrayparser.*
// on the fork-ci branch) and the summary in protocol.h:
//   fmt 1: [u16 len]                 [u8 source][5B hdr][payload][3B CRC]
//   fmt 2: [u16 0x8000|len][u8 ver=2][u8 flags][u8 source][u32 dev us LE][hdr][payload][CRC]
//   fmt 3: [u16 0x8000|len][u8 ver=3][u8 flags][u8 source][u32 dev us LE][u16 fine][u32 seq][hdr][payload][CRC]
#include "flexray_record.h"
#include "protocol.h"

// Pack the 5-byte FlexRay header exactly as the host un-packs it.
static void pack_header(uint8_t *h, const fr_frame_t *f) {
    h[0] = (uint8_t)((f->indicators << 3) | ((f->slot_id >> 8) & 0x07));
    h[1] = (uint8_t)(f->slot_id & 0xFF);
    h[2] = (uint8_t)(((f->payload_len / 2) << 1) | ((f->header_crc >> 10) & 0x01));
    h[3] = (uint8_t)((f->header_crc >> 2) & 0xFF);
    h[4] = (uint8_t)(((f->header_crc & 0x03) << 6) | (f->cycle & 0x3F));
}

static void put_u32le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

int flexray_record_build(uint8_t *buf, const fr_frame_t *f, uint8_t format,
                         uint32_t dev_us, uint32_t seq) {
    uint8_t *body = &buf[2];

    // Bytes before the 5-byte FlexRay header, and the extended record version.
    int prefix;
    if (format == PIFLEX_FMT_TSSEQ)        prefix = 13;  // ver, flags, source, u32 us, u16 fine, u32 seq
    else if (format == PIFLEX_FMT_DEVTIME) prefix = 7;   // ver, flags, source, u32 us
    else { format = PIFLEX_FMT_STOCK;      prefix = 1; } // source only

    if (format == PIFLEX_FMT_STOCK) {
        body[0] = f->channel;                            // source: 1=FR1, 2=FR2
    } else {
        uint8_t flags = 0;
        if (f->frame_crc_ok)  flags |= PIFLEX_FLAG_FRAME_CRC_OK;
        if (f->header_crc_ok) flags |= PIFLEX_FLAG_HEADER_CRC_OK;
        // PIFLEX_FLAG_TS_HW is left clear: the timestamp is read in software when
        // the frame is dequeued, not latched in hardware at the end of the frame.
        body[0] = format;                                // version (2 or 3)
        body[1] = flags;
        body[2] = f->channel;                            // source
        put_u32le(&body[3], dev_us);
        if (format == PIFLEX_FMT_TSSEQ) {
            body[7] = 0;                                 // fine clock (10 ns steps): not sourced yet; host ignores it
            body[8] = 0;
            put_u32le(&body[9], seq);                    // sequence number: a gap means the USB FIFO dropped frames
        }
    }

    uint8_t *hdr = body + prefix;
    pack_header(hdr, f);
    for (int i = 0; i < f->payload_len; i++) hdr[5 + i] = f->payload[i];

    uint8_t *crc = &hdr[5 + f->payload_len];             // 24-bit frame CRC, big-endian
    crc[0] = (uint8_t)((f->frame_crc >> 16) & 0xFF);
    crc[1] = (uint8_t)((f->frame_crc >> 8) & 0xFF);
    crc[2] = (uint8_t)(f->frame_crc & 0xFF);

    const uint16_t body_len = (uint16_t)(prefix + 5 + f->payload_len + 3);
    const uint16_t len_field = (format == PIFLEX_FMT_STOCK)
                                   ? body_len
                                   : (uint16_t)(body_len | PIFLEX_LEN_EXT_FLAG);
    buf[0] = (uint8_t)(len_field & 0xFF);
    buf[1] = (uint8_t)((len_field >> 8) & 0xFF);

    return 2 + body_len;
}
