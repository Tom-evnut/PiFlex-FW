// PiFlex-FW - board configuration (Raspberry Pi Pico 2 / RP2350, non-W).
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef PIFLEX_CONFIG_H
#define PIFLEX_CONFIG_H

#include "pico/stdlib.h"

// --- FlexRay front end -----------------------------------------------------
// Two onsemi NCV7383 transceivers on Damien's Pi_Flex board. Each has a digital
// side (RXD/TXD/TXEN) wired to the Pico and a bus side (BP/BM). Channel 1 is
// bus side A, channel 2 is bus side B - together they form the MITM bridge.
//
// TODO: confirm these GPIO numbers against Pi_Flex_V1.kicad_sch and the bench
// board before first flash. The schematic net names are RXD1/TXD1/TXEN1,
// RXD2/TXD2/TXEN2, STBN and BGE; the numbers below are placeholders.
#define FR1_RXD_PIN     2    // NCV7383 #1 RXD  -> Pico  (input)   [net RXD1]   TODO confirm
#define FR1_TXD_PIN     3    // NCV7383 #1 TXD  <- Pico  (output)  [net TXD1]   TODO confirm
#define FR1_TXEN_PIN    4    // NCV7383 #1 TXEN <- Pico  (output)  [net TXEN1]  TODO confirm
#define FR2_RXD_PIN     6    // NCV7383 #2 RXD  -> Pico  (input)   [net RXD2]   TODO confirm
#define FR2_TXD_PIN     7    // NCV7383 #2 TXD  <- Pico  (output)  [net TXD2]   TODO confirm
#define FR2_TXEN_PIN    8    // NCV7383 #2 TXEN <- Pico  (output)  [net TXEN2]  TODO confirm
#define FR_STBN_PIN     10   // /STB standby (active low, drive high to run)    [net STBN]  TODO confirm
#define FR_BGE_PIN      11   // bus-guardian enable                            [net BGE]   TODO confirm

#define STATUS_LED_PIN  PICO_DEFAULT_LED_PIN  // direct GPIO on the non-W Pico 2

// --- FlexRay timing --------------------------------------------------------
#define FR_BITRATE_HZ   10000000u  // 10 Mbit/s
#define FR_OVERSAMPLE   8u         // PIO samples per bit (see flexray_rx.pio)

// FR_MAX_PAYLOAD lives in flexray_decode.h (shared with the host-testable core).

#endif // PIFLEX_CONFIG_H
