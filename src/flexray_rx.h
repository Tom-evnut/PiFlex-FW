// PiFlex-FW - FlexRay channel capture (SKELETON)
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef PIFLEX_FLEXRAY_RX_H
#define PIFLEX_FLEXRAY_RX_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// A decoded FlexRay frame, before it is packed into a host record.
typedef struct {
    uint16_t slot_id;      // 11-bit frame / slot ID
    uint8_t  cycle;        // 6-bit cycle count
    uint8_t  indicators;   // reserved / payload-preamble / null / sync / startup
    uint16_t header_crc;   // 11-bit
    uint32_t frame_crc;    // 24-bit
    uint8_t  channel;      // PIFLEX_SOURCE_FR1 (bus A) or PIFLEX_SOURCE_FR2 (bus B)
    bool     header_crc_ok;
    bool     frame_crc_ok;
    uint8_t  payload_len;  // bytes (always even)
    uint8_t  payload[FR_MAX_PAYLOAD];
} fr_frame_t;

// Claim PIO resources and start both channels' samplers.
void flexray_rx_init(void);

// Pump decoded frames out of the sample FIFOs. Fills *out and returns true when
// a complete frame is ready, else false.
// TODO: implement TSS lock, byte destuffing, header/payload read and CRC checks.
bool flexray_rx_poll(fr_frame_t *out);

#endif // PIFLEX_FLEXRAY_RX_H
