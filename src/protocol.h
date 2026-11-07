// PiFlex-FW - host (SavvyCAN) interoperability contract.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// These values define how the device talks to SavvyCAN's PicoFlexRay connection
// (connections/picoflexray*.cpp on the SavvyCAN fork-ci branch). They are an
// interoperability contract matched to our own host code, not copied from any
// third-party firmware.
#ifndef PIFLEX_PROTOCOL_H
#define PIFLEX_PROTOCOL_H

#include <stdint.h>

// USB identity: enumerate Panda-compatible so SavvyCAN binds unchanged.
#define PIFLEX_USB_VID        0x3801u
#define PIFLEX_USB_PID        0xDDCCu
#define PIFLEX_SERIAL_PREFIX  "picoflex"
#define PIFLEX_EP_IN          0x81   // bulk IN: FlexRay frame stream to host (EP1-IN)
#define PIFLEX_EP_OUT         0x03   // bulk OUT: injection ops from host (EP3-OUT).
                                     // Must be 0x03: SavvyCAN's picoflexray.cpp
                                     // sends the injector ops (0x90/0x91) and the
                                     // connect-time injector-disable to EP_BULK_OUT
                                     // = 0x03, matching existing dynm devices. An
                                     // OUT at 0x01 makes every connect log a
                                     // LIBUSB_ERROR_NOT_FOUND write failure.

// Vendor control requests (bRequest on the control endpoint).
#define PIFLEX_SET_RECORD_FORMAT 0xA0  // wValue = 1 / 2 / 3
#define PIFLEX_SET_CAN_RECORDS   0xA1  // enable interleaved CAN records (2+2 board)
#define PIFLEX_GET_TS_INFO       0xA2  // reply identifies timestamped-capture firmware
#define PIFLEX_GET_INJECT_STATUS 0xA3  // IN: 16-byte host-config status (see below)
#define PIFLEX_RESET_COMMS       0xC0  // flush / resync the stream
#define PIFLEX_ENTER_BOOTLOADER  0xD1  // reboot into the RP2 USB bootloader for reflash

// Bulk-OUT injector ops (first byte of the OUT payload). All multi-byte fields
// are little-endian. Each op has a deterministic length so the fr_inject_consume
// byte-stream framer (flexray_injector.c) can carve it out of the EP 0x03 FIFO.
#define PIFLEX_OUT_OVERRIDE      0x90  // submit a replacement slice for a rule
#define PIFLEX_OUT_ENABLE        0x91  // [0x91][enabled] global injector on/off

// Host-configurable injection rules + generator schedule (phase 6). The rule
// table (injection allow-list) and the source schedule are RAM-resident, seeded
// at boot with the phase-4/5 sample defaults, and reconfigurable at runtime by
// these ops - no reflash. See flexray_injector.c / flexray_source_cfg.c.
//
//   0x92 SET_INJECT_RULE    7 bytes:
//        [0x92][slot lo][slot hi][cycle_base][cycle_mask][win_off][win_len]
//        Add or replace (keyed by slot+cycle_base+cycle_mask) a RAM injection
//        rule. A forwarded frame is eligible to be spliced when
//          frame.slot == rule.slot AND
//          (frame.cycle & rule.cycle_mask) == (rule.cycle_base & rule.cycle_mask)
//        cycle_mask == 0 means "any cycle" (the rule accepts an override for any
//        cycle, generalising the phase-4 cycle_match=ANY). The override (op 0x90)
//        still carries the specific cycle it fires on; win_off/win_len is the
//        payload byte window an override replaces (same meaning as phase 4).
//   0x93 CLEAR_INJECT_RULES  1 byte:  [0x93]   remove all RAM injection rules.
//   0x94 SET_SOURCE_SLOT     6+N bytes:
//        [0x94][slot lo][slot hi][chan_mask][payload_len=N][flags][payload x N]
//        Add or replace (keyed by slot id) a generator schedule slot. chan_mask
//        bit0=FR1, bit1=FR2. flags bit0 = last payload byte tracks the cycle
//        counter (FR_SRC_FLAG_CYCLE_COUNTER). Framed by reading payload_len.
//   0x95 CLEAR_SOURCE_SCHEDULE 1 byte: [0x95]  remove all RAM schedule slots.
//   0x96 SET_SOURCE_PARAMS   10 bytes:
//        [0x96][period_us u32][cycle_gap_us u32][enable]
//        Set generator frame period, inter-cycle gap, and run/stop.
//
// 0x92/0x93 affect injection, which only splices in an -DFR_INJECT build;
// 0x94/0x95/0x96 affect the generator, which only runs in an -DFR_SOURCE build.
// The control plane (framing + RAM tables) compiles into every build, so the
// ops are accepted and stored uniformly; they just have no bus effect in a build
// whose data path is absent.
#define PIFLEX_OUT_SET_INJECT_RULE    0x92
#define PIFLEX_OUT_CLEAR_INJECT_RULES 0x93
#define PIFLEX_OUT_SET_SOURCE_SLOT    0x94
#define PIFLEX_OUT_CLEAR_SOURCE_SCHED 0x95
#define PIFLEX_OUT_SET_SOURCE_PARAMS  0x96

// GET_INJECT_STATUS (control-IN 0xA3) reply: exactly 16 bytes, little-endian.
//   [0]  'P' (0x50)                 [1]  'F' (0x46)
//   [2]  version = 1                [3]  injector_enabled (0/1)
//   [4]  inject_rule_count          [5]  source_slot_count
//   [6]  source_running (0/1)       [7]  source_slot_drops (u8, saturating 255)
//   [8..11]  u32 total_inject_matches   frames spliced since boot / last clear
//   [12..15] u32 total_inject_drops     op-0x90 slices that matched no rule
// source_slot_drops = op-0x94 SET_SOURCE_SLOT ops rejected (empty / odd /
// over-long payload, or a full table) since boot or the last schedule clear
// (0x95), so the host can see a rejected "apply schedule" instead of a silent drop.
#define PIFLEX_STATUS_LEN      16
#define PIFLEX_STATUS_MAGIC0   0x50  // 'P'
#define PIFLEX_STATUS_MAGIC1   0x46  // 'F'
#define PIFLEX_STATUS_VERSION  1

// Documentation mirror of the 16-byte reply. The firmware serialises the bytes
// explicitly (host-defined little-endian, build-independent); this struct lets
// the host and the host tests pin the layout - sizeof must be PIFLEX_STATUS_LEN.
typedef struct __attribute__((packed)) {
    uint8_t  magic0;                // 'P'
    uint8_t  magic1;                // 'F'
    uint8_t  version;               // PIFLEX_STATUS_VERSION
    uint8_t  injector_enabled;      // 0/1
    uint8_t  inject_rule_count;     // RAM injection rules in force
    uint8_t  source_slot_count;     // RAM generator schedule slots
    uint8_t  source_running;        // 0/1 (generator actually emitting)
    uint8_t  source_slot_drops;     // op-0x94 slots rejected (saturating 255)
    uint32_t total_inject_matches;  // LE
    uint32_t total_inject_drops;    // LE
} piflex_inject_status_t;

// Record formats on the bulk IN stream. Body layout (little-endian u16 length
// first; bit 15 of the length flags an extended v2/v3 record):
//   fmt 1: [u16 len][u8 source][5B header][payload][3B frame CRC, big-endian]
//   fmt 2: [u16 0x8000|len][u8 ver=2][u8 flags][u8 source][u32 device us LE][hdr][payload][CRC]
//   fmt 3: fmt 2 + [u16 fine clock @100 MHz / 10 ns][u32 sequence number]
enum piflex_record_format {
    PIFLEX_FMT_STOCK   = 1,
    PIFLEX_FMT_DEVTIME = 2,
    PIFLEX_FMT_TSSEQ   = 3,
};

// Record flag bits (formats 2 and 3).
#define PIFLEX_FLAG_FRAME_CRC_OK  0x01
#define PIFLEX_FLAG_HEADER_CRC_OK 0x02
#define PIFLEX_FLAG_TS_HW         0x04
#define PIFLEX_LEN_EXT_FLAG       0x8000u  // length bit 15: extended (v2/v3) record

// Source byte: capture channel as decimal digits (1 = FR1 / bus 0, 2 = FR2 / bus 1).
#define PIFLEX_SOURCE_FR1  1
#define PIFLEX_SOURCE_FR2  2

#endif // PIFLEX_PROTOCOL_H
