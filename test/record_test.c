// PiFlex-FW - host test for the SDK-free record serialiser (flexray_record.c).
//
// Builds a known frame, serialises it in each record format, and parses the
// bytes back using the same field offsets as SavvyCAN's PicoFlexRayParser
// (connections/picoflexrayparser.*), checking every field round-trips. This
// catches a layout mistake on the host, before the firmware reaches the bench.
//
// Build & run (host compiler, no Pico SDK needed):
//   gcc -I ../src record_test.c ../src/flexray_record.c ../src/flexray_decode.c -o record_test && ./record_test
#include "flexray_record.h"
#include "flexray_decode.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

// Host-side parse mirroring picoflexrayparser.cpp, for one record in buf.
static void check_record(const uint8_t *buf, int n, const fr_frame_t *f, uint8_t fmt,
                         uint32_t dev_us, uint32_t seq) {
    const int lenField = buf[0] | (buf[1] << 8);
    const int extended = lenField & 0x8000;
    const int bodyLen  = lenField & 0x7FFF;
    const uint8_t *body = &buf[2];

    CHECK(n == 2 + bodyLen, "total length matches length field");
    CHECK((fmt == PIFLEX_FMT_STOCK) ? !extended : (extended != 0), "extended flag set for v2/v3 only");

    int prefix;
    if (!extended) {
        prefix = 1;
        CHECK(body[0] == f->channel, "fmt1 source");
    } else {
        const uint8_t version = body[0];
        CHECK(version == fmt, "version byte");
        prefix = (version == 3) ? 13 : 7;

        const uint8_t flags = body[1];
        CHECK(((flags & PIFLEX_FLAG_FRAME_CRC_OK) != 0) == f->frame_crc_ok, "frame CRC flag");
        CHECK(((flags & PIFLEX_FLAG_HEADER_CRC_OK) != 0) == f->header_crc_ok, "header CRC flag");
        CHECK((flags & PIFLEX_FLAG_TS_HW) == 0, "TS_HW flag clear (software timestamp)");
        CHECK(body[2] == f->channel, "source");

        const uint32_t gotUs = (uint32_t)body[3] | ((uint32_t)body[4] << 8)
                             | ((uint32_t)body[5] << 16) | ((uint32_t)body[6] << 24);
        CHECK(gotUs == dev_us, "device microseconds (LE)");

        if (version == 3) {
            const uint16_t fine = (uint16_t)(body[7] | (body[8] << 8));
            CHECK(fine == 0, "fine clock zero");
            const uint32_t gotSeq = (uint32_t)body[9] | ((uint32_t)body[10] << 8)
                                  | ((uint32_t)body[11] << 16) | ((uint32_t)body[12] << 24);
            CHECK(gotSeq == seq, "sequence number (LE)");
        }
    }

    const uint8_t *h = body + prefix;
    const uint8_t indicators = h[0] >> 3;
    const uint16_t slotId    = (uint16_t)(((h[0] & 0x07) << 8) | h[1]);
    const int payloadLen     = ((h[2] >> 1) & 0x7F) * 2;
    const uint16_t headerCrc = (uint16_t)(((h[2] & 0x01) << 10) | (h[3] << 2) | (h[4] >> 6));
    const uint8_t cycle      = h[4] & 0x3F;

    CHECK(indicators == f->indicators, "indicators");
    CHECK(slotId == f->slot_id, "slot id");
    CHECK(payloadLen == f->payload_len, "payload length");
    CHECK(headerCrc == f->header_crc, "header crc");
    CHECK(cycle == f->cycle, "cycle");
    CHECK(memcmp(h + 5, f->payload, f->payload_len) == 0, "payload bytes");

    CHECK(prefix + 5 + payloadLen + 3 == bodyLen, "body length = prefix + header + payload + crc");

    const uint8_t *crc = h + 5 + payloadLen;
    const uint32_t frameCrc = ((uint32_t)crc[0] << 16) | ((uint32_t)crc[1] << 8) | crc[2];
    CHECK(frameCrc == f->frame_crc, "frame crc (big-endian)");
}

// Fill *f with a self-consistent frame (correct header/frame CRCs).
static void make_frame(fr_frame_t *f, uint8_t channel) {
    static const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02};
    memset(f, 0, sizeof *f);
    f->channel     = channel;
    f->indicators  = 0;
    f->slot_id     = 0x123;
    f->cycle       = 9;
    f->payload_len = (uint8_t)sizeof payload;
    memcpy(f->payload, payload, sizeof payload);
    f->header_crc  = flexray_header_crc(0, 0, f->slot_id, f->payload_len / 2);

    uint8_t hdr[5 + sizeof payload];
    hdr[0] = (uint8_t)((f->indicators << 3) | ((f->slot_id >> 8) & 0x07));
    hdr[1] = (uint8_t)(f->slot_id & 0xFF);
    hdr[2] = (uint8_t)(((f->payload_len / 2) << 1) | ((f->header_crc >> 10) & 0x01));
    hdr[3] = (uint8_t)((f->header_crc >> 2) & 0xFF);
    hdr[4] = (uint8_t)(((f->header_crc & 0x03) << 6) | (f->cycle & 0x3F));
    memcpy(&hdr[5], payload, sizeof payload);
    f->frame_crc   = flexray_frame_crc(hdr, 5 + (int)sizeof payload, channel);
    f->header_crc_ok = true;
    f->frame_crc_ok  = true;
}

int main(void) {
    uint8_t rec[PIFLEX_RECORD_MAX];
    const uint32_t dev_us = 0x11223344u;
    const uint32_t seq    = 0x0A0B0C0Du;

    fr_frame_t f;
    make_frame(&f, PIFLEX_SOURCE_FR1);

    // Each format round-trips field-for-field.
    check_record(rec, flexray_record_build(rec, &f, PIFLEX_FMT_STOCK,   dev_us, seq), &f, PIFLEX_FMT_STOCK,   dev_us, seq);
    check_record(rec, flexray_record_build(rec, &f, PIFLEX_FMT_DEVTIME, dev_us, seq), &f, PIFLEX_FMT_DEVTIME, dev_us, seq);
    check_record(rec, flexray_record_build(rec, &f, PIFLEX_FMT_TSSEQ,   dev_us, seq), &f, PIFLEX_FMT_TSSEQ,   dev_us, seq);

    // A CRC-failed frame clears the frame-CRC flag but keeps the header-CRC flag (format 2/3).
    {
        fr_frame_t bad = f;
        bad.frame_crc_ok = false;
        check_record(rec, flexray_record_build(rec, &bad, PIFLEX_FMT_DEVTIME, dev_us, seq), &bad, PIFLEX_FMT_DEVTIME, dev_us, seq);
    }

    // FR2 source and a zero-payload (null) frame round-trip in format 3.
    {
        fr_frame_t g;
        make_frame(&g, PIFLEX_SOURCE_FR2);
        g.payload_len = 0;
        g.header_crc  = flexray_header_crc(0, 0, g.slot_id, 0);
        uint8_t hdr[5];
        hdr[0] = (uint8_t)((g.indicators << 3) | ((g.slot_id >> 8) & 0x07));
        hdr[1] = (uint8_t)(g.slot_id & 0xFF);
        hdr[2] = (uint8_t)((0 << 1) | ((g.header_crc >> 10) & 0x01));
        hdr[3] = (uint8_t)((g.header_crc >> 2) & 0xFF);
        hdr[4] = (uint8_t)(((g.header_crc & 0x03) << 6) | (g.cycle & 0x3F));
        g.frame_crc = flexray_frame_crc(hdr, 5, PIFLEX_SOURCE_FR2);
        check_record(rec, flexray_record_build(rec, &g, PIFLEX_FMT_TSSEQ, dev_us, seq), &g, PIFLEX_FMT_TSSEQ, dev_us, seq);
    }

    // An unknown format is treated as format 1 (no extended flag).
    {
        const int n = flexray_record_build(rec, &f, 99, dev_us, seq);
        CHECK((rec[1] & 0x80) == 0, "unknown format falls back to format 1");
        check_record(rec, n, &f, PIFLEX_FMT_STOCK, dev_us, seq);
    }

    printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
    return failures ? 1 : 0;
}
