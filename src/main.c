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
#include "flexray_injector.h"
#include "flexray_inject_tx.h"
#include "flexray_source_tx.h"
#include "flexray_mode.h"
#include "flexray_mode_hw.h"

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

// Drain the bulk-OUT host-config pipe (EP 0x03) and dispatch each complete op to
// the control plane. TinyUSB presents the OUT endpoint as a byte FIFO with no
// message boundaries, so bytes are accumulated here and framed by fr_inject_consume
// (op byte + each op's deterministic length: 0x90-0x96, see protocol.h). An unknown
// leading byte resyncs by dropping the buffer; a buffer that fills without a
// complete message (a malformed/garbage stream) is also dropped, so the pipe always
// recovers. Sized for the largest op (0x90: 6-byte header + CRC + max payload).
static void poll_injector_out(void) {
    static uint8_t buf[6 + 1 + FR_MAX_PAYLOAD];   // largest 0x90: header + slice
    static int have = 0;

    while (tud_vendor_available()) {
        if (have >= (int)sizeof buf) have = 0;    // no complete op in a full buffer -> resync
        const uint32_t n = tud_vendor_read(buf + have, sizeof buf - (uint32_t)have);
        if (n == 0) break;
        have += (int)n;

        for (;;) {
            const int used = fr_inject_consume(buf, have);
            if (used == 0) break;                 // need more bytes
            if (used < 0) { have = 0; break; }    // unknown op -> drop and resync
            memmove(buf, buf + used, (size_t)(have - used));
            have -= used;
        }
    }
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

        case PIFLEX_GET_INJECT_STATUS: {
            // 16-byte host-config status: magic/version, injector enable + rule
            // count, generator slot count + run-state, and the match/drop counters
            // (see protocol.h). Lets the host confirm its pushed config took effect
            // and watch injection actually fire (and non-matching overrides drop).
            static uint8_t status[PIFLEX_STATUS_LEN];
            fr_inject_build_status(status);
            return tud_control_xfer(rhport, req, status, sizeof(status));
        }

        case PIFLEX_SET_MODE:
            // Switch the data path at runtime. wValue = mode 0..3; an out-of-range
            // value is ignored (fr_mode_set returns false, no transition). The swap
            // releases both TXEN before reprogramming pio1, so it is bus-safe; this
            // runs in tud_task() (main-loop context), not an ISR. ACK either way.
            fr_mode_set((unsigned)req->wValue);
            return tud_control_status(rhport, req);

        case PIFLEX_GET_MODE: {
            static uint8_t mode;
            mode = (uint8_t)fr_mode_active();
            return tud_control_xfer(rhport, req, &mode, sizeof(mode));
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
    // This firmware selects the TX data path (sniff / bridge / inject / source) at
    // runtime (flexray_mode_hw_init below, then control request 0xA4). At boot the
    // mode manager's safe_bus step drives both TXEN released (inactive HIGH, so
    // each NCV7383 stays in receive and drives its RXD) before any TX program is
    // loaded; TXEN is never left floating. The default mode is SNIFF, so the board
    // drives nothing until the host selects a mode.

    // Pull up the RXD inputs so the line is defined between frames (matches KNOWN_GOOD).
    gpio_pull_up(FR1_RXD_PIN);
    gpio_pull_up(FR2_RXD_PIN);

    // Take both NCV7383 transceivers out of standby into normal mode. BGE must be
    // HIGH for the transceiver to receive on this board (matches the proven KNOWN_GOOD
    // setup: STBN=1, BGE=1); BGE low left RXD inactive and captured nothing on the bench.
    gpio_init(FR_STBN_PIN); gpio_set_dir(FR_STBN_PIN, GPIO_OUT); gpio_put(FR_STBN_PIN, 1);
    gpio_init(FR_BGE_PIN);  gpio_set_dir(FR_BGE_PIN,  GPIO_OUT); gpio_put(FR_BGE_PIN,  1);

    tusb_init();
    fr_inject_reset();   // SAFETY: injector OFF until the host enables it (op 0x91)
    flexray_rx_init();   // RX capture (pio0) runs in every mode

    // Bring up the runtime mode manager at the default (compile-time) mode. The
    // host switches modes later with 0xA4; RX keeps capturing throughout. Only one
    // TX mode drives pio1 at a time; the manager releases TXEN before every swap.
    flexray_mode_hw_init((fr_mode_t)FR_DEFAULT_MODE);

    fr_frame_t frame;
    for (;;) {
        tud_task();
        if (s_enter_bootloader) {
            sleep_ms(10);          // let the control-status ACK reach the host
            reset_usb_boot(0, 0);  // reboot into the RP2 USB mass-storage bootloader
        }

        const fr_mode_t mode = fr_mode_active();

        // SOURCE drives the generator whether or not a host is attached: the board
        // is a standalone bench signal source. Captured frames (including the
        // board's own echoed emission) still only stream to the host while mounted.
        if (mode == PIFLEX_MODE_SOURCE)
            flexray_source_tick();

        if (tud_vendor_mounted()) {
            poll_injector_out();                 // drain EP 0x03 host-config ops (all modes)
            if (flexray_rx_poll(&frame)) {
                send_frame(&frame);              // stream the captured frame to the host
                if (mode == PIFLEX_MODE_INJECT)
                    flexray_inject_forward(&frame);  // regenerate FR2->FR1, splicing a match
            }
        }
    }
}
