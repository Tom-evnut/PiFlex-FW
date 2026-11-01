// PiFlex-FW - entry point (phase 1: FlexRay sniffer skeleton).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Brings up USB (Panda-compatible) and the FlexRay RX samplers, then streams
// captured frames to SavvyCAN as format-1 records. Device timestamps (format
// 2) and the fine clock + sequence number (format 3) come in later phases.
#include "pico/stdlib.h"
#include "tusb.h"
#include "config.h"
#include "protocol.h"
#include "flexray_rx.h"
#include <string.h>

static uint8_t s_record_format = PIFLEX_FMT_STOCK;

// Pack a decoded frame into a format-1 record and queue it on the bulk IN pipe.
// Layout must match SavvyCAN's PicoFlexRayParser (see protocol.h).
static void send_frame_fmt1(const fr_frame_t *f) {
    uint8_t rec[2 + 1 + 5 + FR_MAX_PAYLOAD + 3];
    const uint16_t body_len = (uint16_t)(1 + 5 + f->payload_len + 3);

    rec[0] = (uint8_t)(body_len & 0xFF);
    rec[1] = (uint8_t)((body_len >> 8) & 0xFF);
    rec[2] = f->channel;                                   // source: 1=FR1, 2=FR2

    // 5-byte FlexRay header, bit-packed exactly as the host un-packs it.
    rec[3] = (uint8_t)((f->indicators << 3) | ((f->slot_id >> 8) & 0x07));
    rec[4] = (uint8_t)(f->slot_id & 0xFF);
    rec[5] = (uint8_t)(((f->payload_len / 2) << 1) | ((f->header_crc >> 10) & 0x01));
    rec[6] = (uint8_t)((f->header_crc >> 2) & 0xFF);
    rec[7] = (uint8_t)(((f->header_crc & 0x03) << 6) | (f->cycle & 0x3F));

    memcpy(&rec[8], f->payload, f->payload_len);

    uint8_t *crc = &rec[8 + f->payload_len];               // 24-bit frame CRC, big-endian
    crc[0] = (uint8_t)((f->frame_crc >> 16) & 0xFF);
    crc[1] = (uint8_t)((f->frame_crc >> 8) & 0xFF);
    crc[2] = (uint8_t)(f->frame_crc & 0xFF);

    const uint32_t total = (uint32_t)(2 + body_len);
    if (tud_vendor_write_available() >= total) {
        tud_vendor_write(rec, total);
        tud_vendor_write_flush();
    }
    // else: drop; a later phase tracks this as a USB-FIFO loss (format 3 seq).
}

// Host control requests on the vendor interface.
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *req) {
    if (stage != CONTROL_STAGE_SETUP) return true;

    switch (req->bRequest) {
        case PIFLEX_SET_RECORD_FORMAT:
            s_record_format = (uint8_t)req->wValue;        // TODO honour 2/3 once implemented
            return tud_control_status(rhport, req);

        case PIFLEX_GET_TS_INFO: {
            // TODO: reply with the timestamped-capture identity when format 3 lands.
            static const uint8_t info[4] = {0, 0, 0, 0};
            return tud_control_xfer(rhport, req, (void *)info, sizeof(info));
        }

        case PIFLEX_RESET_COMMS:
            return tud_control_status(rhport, req);

        default:
            return false;                                  // stall unknown requests
    }
}

int main(void) {
    stdio_init_all();

    // Pi_Flex board bring-up (mirrors the proven dynm KNOWN_GOOD sequence).
    //
    // We are a pure sniffer and never transmit, so hold both TXEN lines INACTIVE.
    // TXEN is active-low, so inactive = HIGH keeps each NCV7383 in receive mode and
    // driving its RXD output; left floating the transceiver can sit in TX mode and
    // RXD stays idle (captured nothing on the bench). KNOWN_GOOD pulls these up; we
    // drive them high directly since we never need the PIO to assert them.
    gpio_init(FR1_TXEN_PIN); gpio_set_dir(FR1_TXEN_PIN, GPIO_OUT); gpio_put(FR1_TXEN_PIN, 1);
    gpio_init(FR2_TXEN_PIN); gpio_set_dir(FR2_TXEN_PIN, GPIO_OUT); gpio_put(FR2_TXEN_PIN, 1);

    // Pull up the RXD inputs so the line is defined between frames (matches KNOWN_GOOD).
    gpio_pull_up(FR1_RXD_PIN);
    gpio_pull_up(FR2_RXD_PIN);

    // Take both NCV7383 transceivers out of standby into normal mode. BGE must be
    // HIGH for the transceiver to receive on this board (matches the proven KNOWN_GOOD
    // setup: STBN=1, BGE=1); BGE low left RXD inactive and captured nothing on the bench.
    gpio_init(FR_STBN_PIN); gpio_set_dir(FR_STBN_PIN, GPIO_OUT); gpio_put(FR_STBN_PIN, 1);
    gpio_init(FR_BGE_PIN);  gpio_set_dir(FR_BGE_PIN,  GPIO_OUT); gpio_put(FR_BGE_PIN,  1);

    tusb_init();
    flexray_rx_init();

    fr_frame_t frame;
    for (;;) {
        tud_task();
        if (tud_vendor_mounted() && flexray_rx_poll(&frame)) {
            (void)s_record_format;   // phase 1 always emits format 1
            send_frame_fmt1(&frame);
        }
    }
}
