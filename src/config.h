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
// Sustained-high line length (in bit times) that marks bus idle / end-of-frame.
// Must exceed the 9-bit worst-case in-frame high run (8 data '1's + the next BSS
// high) so a run inside a frame never looks like idle. C turns this into the
// replay-pass count the PIO counts (see flexray_tx_program_init). <<TUNE>>.
#define FR_TX_IDLE_BITS 11u

// Note: the anti-oscillation watchdog (whole-frame TXEN cap) is built inside the
// PIO program from a shifted constant - see flexray_tx.pio - so it needs no value
// here. It only has to exceed the longest real frame (~2640 bits).

// FR_BRIDGE_ONE_WAY (define via -DFR_BRIDGE_ONE_WAY=1 or the CMake option):
// build the bridge one-way, forwarding FR2 -> FR1 only with no reverse path, so
// the forward-path fidelity can be bench-verified in isolation (no echo loop).

// --- FlexRay source / generator (phase 5, -DFR_SOURCE) ---------------------
// Source mode (see flexray_source_tx.c) is a free-running static-segment
// emitter: it walks FR_SOURCE_SCHEDULE once per cycle, pacing one slot frame
// every FR_SRC_FRAME_PERIOD_US and advancing the 0..63 cycle counter when the
// schedule wraps. These are deliberately generous bench defaults (well under the
// ~17 us a short frame occupies on the 10 Mbit/s wire) so emission never
// saturates; trim them on the bench to set the desired frame rate. <<TUNE>>.
//
// FR_SRC_FRAME_PERIOD_US : spacing between successive emitted slot frames. With
//   N slots this makes one cycle take N * FR_SRC_FRAME_PERIOD_US, and a full
//   0..63 sweep 64 * N * FR_SRC_FRAME_PERIOD_US.
// FR_SRC_CYCLE_GAP_US    : extra idle inserted when the schedule wraps to the
//   next cycle (0 = none; a steady cadence). Lets the bench widen the cycle
//   boundary without changing the per-slot spacing.
#define FR_SRC_FRAME_PERIOD_US  500u
#define FR_SRC_CYCLE_GAP_US     0u

// FR_MAX_PAYLOAD lives in flexray_decode.h (shared with the host-testable core).

#endif // PIFLEX_CONFIG_H
