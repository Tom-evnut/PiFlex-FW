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
#define PIFLEX_EP_IN          0x81   // bulk IN: FlexRay frame stream to host
#define PIFLEX_EP_OUT         0x01   // bulk OUT: injection ops from host

// Vendor control requests (bRequest on the control endpoint).
#define PIFLEX_SET_RECORD_FORMAT 0xA0  // wValue = 1 / 2 / 3
#define PIFLEX_SET_CAN_RECORDS   0xA1  // enable interleaved CAN records (2+2 board)
#define PIFLEX_GET_TS_INFO       0xA2  // reply identifies timestamped-capture firmware
#define PIFLEX_RESET_COMMS       0xC0  // flush / resync the stream

// Bulk-OUT injector ops (first byte of the OUT payload).
#define PIFLEX_OUT_OVERRIDE      0x90  // submit a replacement slice for a rule
#define PIFLEX_OUT_ENABLE        0x91  // [0x91][enabled] global injector on/off

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
