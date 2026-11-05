// PiFlex-FW - entry point (phase 1: FlexRay sniffer skeleton).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Brings up USB (Panda-compatible) and the FlexRay RX samplers, then streams
// captured frames to SavvyCAN. The host selects the record format: 1 (stock),
// 2 (device microsecond timestamp + CRC flags), or 3 (format 2 + a per-frame
// sequence number that exposes USB FIFO loss). See protocol.h / flexray_record.c.
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "tusb.h"
#include "config.h"
#include "protocol.h"
#include "flexray_rx.h"
#include "flexray_tx.h"
#include "flexray_record.h"

static uint8_t s_record_format = PIFLEX_FMT_STOCK;

// Counts frames the capture core produces; sent as the format-3 sequence number
// so the host can detect frames lost to a full USB FIFO (a gap in the sequence).
static uint32_t s_frame_seq = 0;

// Set by the ENTER_BOOTLOADER control request; the main loop does the reset once
// the control-status ACK has been sent, so the host sees the request succeed.
static volatile bool s_enter_bootloader = false;

// Pack a decoded frame in the selected record format and queue it on the bulk
// IN pipe. Layout lives in flexray_record.c, matched to SavvyCAN's parser.
static void send_frame(const fr_frame_t *f) {
    // Format 1 carries no CRC flags, and the host treats every format-1 frame as
    // having passed both CRCs, so only forward good frames in that mode. Formats
    // 2/3 carry the CRC status in their flags byte, so forward every frame — a
    // CRC error is information the host wants to see.
    if (s_record_format == PIFLEX_FMT_STOCK && !(f->header_crc_ok && f->frame_crc_ok))
        return;

    uint8_t rec[PIFLEX_RECORD_MAX];
    const uint32_t dev_us = time_us_32();   // device microsecond clock; wraps ~71.6 min, the host unwraps it
    const uint32_t seq    = s_frame_seq++;  // advances per captured frame, even when a record is dropped
    const int total = flexray_record_build(rec, f, s_record_format, dev_us, seq);

    if (tud_vendor_write_available() >= (uint32_t)total) {
        tud_vendor_write(rec, (uint32_t)total);
        tud_vendor_write_flush();
    }
    // else: drop. In format 3 the resulting gap in the sequence number tells the host.
}

// Host control requests on the vendor interface.
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *req) {
    if (stage != CONTROL_STAGE_SETUP) return true;

    switch (req->bRequest) {
        case PIFLEX_SET_RECORD_FORMAT: {
            const uint8_t fmt = (uint8_t)req->wValue;
            if (fmt == PIFLEX_FMT_STOCK || fmt == PIFLEX_FMT_DEVTIME || fmt == PIFLEX_FMT_TSSEQ)
                s_record_format = fmt;                     // ignore an unknown value, keep the current format
            return tud_control_status(rhport, req);
        }

        case PIFLEX_GET_TS_INFO: {
            // A non-empty reply tells the host we do timestamped capture, so it
            // selects format 3 (device us + per-frame sequence). The host only
            // checks that the reply is non-empty; its contents are unused.
            static const uint8_t info[4] = {'P', 'F', PIFLEX_FMT_TSSEQ, 0};
            return tud_control_xfer(rhport, req, (void *)info, sizeof(info));
        }

        case PIFLEX_RESET_COMMS:
            return tud_control_status(rhport, req);

        case PIFLEX_ENTER_BOOTLOADER:
            // Ack now; the main loop reboots into the bootloader after the ACK
            // has gone out. Saves a physical BOOTSEL press on every reflash.
            s_enter_bootloader = true;
            return tud_control_status(rhport, req);

        default:
            return false;                                  // stall unknown requests
    }
}

int main(void) {
    stdio_init_all();

    // Pi_Flex board bring-up (mirrors the proven dynm KNOWN_GOOD sequence).
    //
    // The TXD/TXEN lines belong to the TX bridge now: flexray_tx_init() (below)
    // takes over those four GPIOs, leaving each TXEN released (inactive HIGH, so
    // the NCV7383 stays in receive mode and drives its RXD) until a frame arms
    // it. TXEN must not be left floating - the transceiver can then sit in TX
    // mode and RXD stays idle (captured nothing on the bench) - so the bridge
    // drives both TXEN high before handing them to PIO.

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
    // Start the MITM bit-mirror bridge. RX (pio0) keeps capturing while the
    // bridge (pio1) repeats each channel onto the other and drives TXEN.
    flexray_tx_init();

    fr_frame_t frame;
    for (;;) {
        tud_task();
        if (s_enter_bootloader) {
            sleep_ms(10);          // let the control-status ACK reach the host
            reset_usb_boot(0, 0);  // reboot into the RP2 USB mass-storage bootloader
        }
        if (tud_vendor_mounted() && flexray_rx_poll(&frame)) {
            send_frame(&frame);
        }
    }
}
