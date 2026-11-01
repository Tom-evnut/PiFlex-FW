// PiFlex-FW - serialise a decoded FlexRay frame into a host (SavvyCAN) record.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// SDK-free so the exact byte layout can be unit-tested on a host against the
// contract in protocol.h (see test/record_test.c). The firmware (main.c) reads
// the device clock / sequence number and does the USB write; this just packs.
#ifndef PIFLEX_FLEXRAY_RECORD_H
#define PIFLEX_FLEXRAY_RECORD_H

#include <stdint.h>
#include "flexray_decode.h"   // fr_frame_t, FR_MAX_PAYLOAD

// Largest possible record: u16 length + format-3 prefix (13) + 5-byte header +
// max payload + 3-byte frame CRC.
#define PIFLEX_RECORD_MAX  (2 + 13 + 5 + FR_MAX_PAYLOAD + 3)

// Serialise f into buf (>= PIFLEX_RECORD_MAX bytes) as a record of the given
// format (PIFLEX_FMT_STOCK / _DEVTIME / _TSSEQ). dev_us is the device microsecond
// clock and seq the per-frame sequence number; both are ignored by format 1 and
// seq is ignored by format 2. Returns the total record length (including the
// 2-byte length prefix). An unknown format is treated as format 1.
int flexray_record_build(uint8_t *buf, const fr_frame_t *f, uint8_t format,
                         uint32_t dev_us, uint32_t seq);

#endif // PIFLEX_FLEXRAY_RECORD_H
