// PiFlex-FW - FlexRay injector TX path (hardware glue, phase 4 Stage B).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Emits a rebuilt (optionally spliced) FlexRay frame on the destination channel
// through the flexray_inject.pio transmitter. This is the "modify-and-re-emit"
// path the injector takes for a slot it overrides, in place of the verbatim
// bit-mirror. Only compiled/started in the injector build (-DFR_INJECT), where
// the FR2 -> FR1 mirror is NOT started, so the inject SM is the sole driver of
// FR1 TXD/TXEN and there is no PIO contention on the bus.
//
// TIMING / APPROACH (see flexray_inject_tx.c for the full budget): the pure
// bit-mirror emits each bit ~30 ns after it arrives and cannot alter a byte in
// flight. Injection therefore decodes the frame (the RX core already does this),
// splices the override + recomputes the 24-bit CRC, and re-emits the frame here.
// That is store-and-forward for the matched slot: ~1 frame of latency. The ideal
// zero-added-latency cut-through (mirror verbatim up to the replace window, then
// substitute bytes + emit a fresh CRC) needs a mixed-mode TX PIO and is the
// documented bench follow-up.
#ifndef PIFLEX_FLEXRAY_INJECT_TX_H
#define PIFLEX_FLEXRAY_INJECT_TX_H

#include "flexray_decode.h"   // fr_frame_t

// Claim a PIO SM and start the injected-frame transmitter on the destination
// channel (FR1 TXD/TXEN), and release the listened channel's (FR2) TXEN so it
// stays in receive. Call instead of flexray_tx_init() in the injector build.
void flexray_inject_tx_init(void);

// Handle one frame captured on the listened (source) channel. If it is on the
// source channel, rebuild it - splicing the override window when injection is
// enabled and a rule+override match (fr_inject_match), else verbatim - and emit
// it on the destination channel with a freshly recomputed frame CRC. Frames on
// the destination channel are ignored. Returns true if a frame was emitted.
bool flexray_inject_forward(const fr_frame_t *f);

#endif // PIFLEX_FLEXRAY_INJECT_TX_H
