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

// --- System clock ----------------------------------------------------------
// Run clk_sys at an EXACT multiple of the FlexRay sample/emit rates so every
// FlexRay PIO clock divider is INTEGER. This is a timing-fidelity requirement,
// not just a speed knob:
//   - RX decoder  (flexray_rx.pio):     div = clk_sys / (FR_BITRATE_HZ * FR_OVERSAMPLE)
//   - source/inject emitter (flexray_inject.pio): div = clk_sys / (FR_BITRATE_HZ * 2)
// At the SDK default 150 MHz BOTH are fractional (150/80 = 1.875 and 150/20 =
// 7.5). A fractional PIO divider dithers each SM tick by up to one system clock,
// so RX sample points and emitted edges jitter ~+-7 ns. On a chain where a PiFlex
// is BOTH the source and the receiver (the two-board bench rig), that dither
// compounds with the bit-mirror's copy-loop edge quantization (flexray_tx.pio,
// ~1/3 bit) and eats the receiver's bit-centre margin -> single bits next to
// transitions mis-sample (~15% of bridged frames corrupt; worst mid-frame /
// high-transition bytes, clean at frame ends). 160 MHz is the lowest clock that
// makes BOTH dividers integer (RX 160/80 = 2.0, emit 160/20 = 8.0) and it also
// shrinks the mirror's copy-loop quantization (~33 ns -> ~25 ns). Set in main()
// before any PIO init. This is a mild overclock above the 150 MHz default: the
// bench must confirm stability on this board (main() nudges vreg for margin).
// 192 MHz (RX 2.4 -> fractional again) is NOT valid; use 160/240/320 (x80).
#define FR_SYS_CLK_KHZ  160000u

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
// emitter: it walks the RAM schedule once per cycle, pacing one slot frame every
// "period_us" and advancing the 0..63 cycle counter when the schedule wraps.
//
// Since phase 6 the schedule, the frame period, the inter-cycle gap, and the
// run/stop state are RAM-resident and host-configurable at runtime (SavvyCAN ops
// 0x94/0x95/0x96, see protocol.h / flexray_source_cfg.c). The boot defaults live
// SDK-free in flexray_source_cfg.h (FR_SRC_DEFAULT_PERIOD_US = 500 us,
// FR_SRC_DEFAULT_CYCLE_GAP_US = 0, generator enabled) - deliberately generous
// bench values (well under the ~17 us a short frame occupies on the 10 Mbit/s
// wire) so emission never saturates; the host trims them live over 0x96 instead
// of a reflash.

// FR_MAX_PAYLOAD lives in flexray_decode.h (shared with the host-testable core).

// --- Runtime data-path mode (phase 7) --------------------------------------
// One firmware compiles every TX path in and selects SNIFF / BRIDGE / INJECT /
// SOURCE at runtime over control requests 0xA4/0xA5 (see protocol.h /
// flexray_mode.c). FR_DEFAULT_MODE is only the power-up mode before the host
// selects one; it defaults to SNIFF (0) so the board drives nothing until told
// to. Override at build time with e.g. -DFR_DEFAULT_MODE=1 to boot into BRIDGE.
// Values: 0 = SNIFF, 1 = BRIDGE, 2 = INJECT, 3 = SOURCE (enum piflex_mode).
#ifndef FR_DEFAULT_MODE
#define FR_DEFAULT_MODE  0  /* PIFLEX_MODE_SNIFF */
#endif

#endif // PIFLEX_CONFIG_H
