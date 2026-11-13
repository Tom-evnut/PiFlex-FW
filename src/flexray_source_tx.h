// PiFlex-FW - FlexRay source / generator TX path (hardware glue, phase 5).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Source mode (-DFR_SOURCE) makes the board the active transmitter: it walks the
// static schedule (flexray_source_schedule.h) and emits each slot's frame on the
// configured channel(s), turning a single Pi_Flex board into the bench FlexRay
// signal source. Built in place of the phase-3 MITM bridge; the two FlexRay TX
// state machines (one per channel, on pio1) are the sole drivers of FR1/FR2
// TXD/TXEN, so there is no PIO contention. RX capture (pio0) keeps running, so
// SavvyCAN can watch what the board emits via the normal record stream (the
// NCV7383 does not blank its own RXD during TX, so each channel's transmission
// echoes back into its receiver - a bench characteristic to confirm).
#ifndef PIFLEX_FLEXRAY_SOURCE_TX_H
#define PIFLEX_FLEXRAY_SOURCE_TX_H

// Claim two PIO SMs (FR1 + FR2) and their DMA channels on the inject PIO and
// start the generator transmitters, driving both TXEN released (idle) until the
// first frame. Call instead of flexray_tx_init() in the source build.
void flexray_source_init(void);

// Tear the generator down for a runtime mode switch: stop + free both channels'
// DMA and SMs and the shared pio1 program (TXEN released by the mode manager
// first). After this the pio1 instruction memory, SMs and DMA are free.
void flexray_source_deinit(void);

// Advance the free-running schedule. Call once per main-loop iteration: when the
// next slot's time is due (FR_SRC_FRAME_PERIOD_US apart) it builds that slot's
// frame for each configured channel, clocks it out via DMA, then steps to the
// next slot - wrapping to the next cycle (0..63) at the end of the schedule.
// Non-blocking: a channel whose previous frame is still transmitting is skipped.
void flexray_source_tick(void);

#endif // PIFLEX_FLEXRAY_SOURCE_TX_H
