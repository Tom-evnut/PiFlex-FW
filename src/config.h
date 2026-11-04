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
// GPIO map for the Pi_Flex V1 board (Pico 2). Confirmed against the proven
// dynm/Damien KNOWN_GOOD firmware's flexray_pins.h, which runs correctly on this
// exact bench board (RX decodes on both channels, TX verified on FR1 GP28/GP27).
// FR1 = "car side", FR2 = "DDE side".
#define FR1_RXD_PIN     26   // NCV7383 #1 RXD  -> Pico  (input)   [net RXD1]
#define FR1_TXD_PIN     28   // NCV7383 #1 TXD  <- Pico  (output)  [net TXD1]
#define FR1_TXEN_PIN    27   // NCV7383 #1 TXEN <- Pico  (output)  [net TXEN1]
#define FR2_RXD_PIN     6    // NCV7383 #2 RXD  -> Pico  (input)   [net RXD2]
#define FR2_TXD_PIN     4    // NCV7383 #2 TXD  <- Pico  (output)  [net TXD2]
#define FR2_TXEN_PIN    5    // NCV7383 #2 TXEN <- Pico  (output)  [net TXEN2]
#define FR_STBN_PIN     3    // /STB standby (active low, drive high to run)    [net STBN]
#define FR_BGE_PIN      2    // bus-guardian enable                            [net BGE]

#define STATUS_LED_PIN  PICO_DEFAULT_LED_PIN  // direct GPIO on the non-W Pico 2

// --- FlexRay timing --------------------------------------------------------
#define FR_BITRATE_HZ   10000000u  // 10 Mbit/s
#define FR_OVERSAMPLE   8u         // PIO samples per bit (see flexray_rx.pio)

// --- FlexRay MITM bridge (TX) ----------------------------------------------
// The two TX state machines mirror each channel's RXD onto the other channel's
// TXD, asserting TXEN (active-low) for each frame. See flexray_tx.pio.
//
// Sustained-high line length (in bit times) that marks bus idle and end-of-
// frame. Must exceed the 9-bit worst-case in-frame high run (8 data '1's plus
// the following BSS high bit) so a run inside a frame never looks like idle.
// Bigger = more noise margin but a longer post-frame TXEN hold; smaller risks a
// false frame-end on '1'-heavy data. <<TUNE>> on the bench.
#define FR_TX_IDLE_BITS 11u

// FR_MAX_PAYLOAD lives in flexray_decode.h (shared with the host-testable core).

#endif // PIFLEX_CONFIG_H
