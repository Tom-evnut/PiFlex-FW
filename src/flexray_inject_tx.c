// PiFlex-FW - FlexRay injector TX path (hardware glue, phase 4 Stage B).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// ===========================================================================
// TIMING BUDGET - why injection is store-and-forward here, and what it costs
// ===========================================================================
// FlexRay on this bench runs at 10 Mbit/s = 100 ns/bit. A frame on the wire is
//   TSS (>=3 low bits; the car uses 9) + FSS (1) + per byte [BSS "10" + 8 data]
//   + FES, i.e. 10 bit-cells per byte. Header = 5 bytes = 50 cells (~5 us);
//   payload up to 254 bytes; frame CRC = 3 bytes (~3 us).
//
// The phase-3 bridge is a pure PIO bit-mirror: it copies RXD->TXD with ~30 ns
// latency and asserts TXEN at the idle->TSS edge, BEFORE any byte is known. It
// physically cannot change a byte in flight, and by the time software has even
// reassembled the slot ID (header bytes 0-1) and cycle (header byte 4) via the
// RX decoder + DMA ring, the mirror has already driven those bits onto the bus.
//
// So to alter a payload byte and still emit a VALID frame (the 24-bit CRC spans
// the whole header+payload and sits at the end), the firmware must:
//   1. receive and decode the frame (the RX core already produces fr_frame_t),
//   2. splice the override window and recompute the frame CRC (flexray_inject_build),
//   3. re-emit the whole frame through flexray_inject.pio.
// Step 1 means the frame is fully in hand only AFTER its last payload byte, so
// this is STORE-AND-FORWARD: the injected frame is delayed by ~one frame time
// (plus the re-emit). On a continuously scheduled TDMA bus that pushes the
// injected frame past its own slot window, so a strict FlexRay controller would
// see it in the next slot. On the bench rig (generator on FR2, far-side listener
// on FR1, SavvyCAN) the listener still receives a well-formed, CRC-valid frame
// with the overridden bytes - which is what Stage B is verifying.
//
// To avoid PIO contention this build does NOT start the FR2->FR1 bit-mirror; the
// inject SM is the sole driver of FR1 TXD/TXEN. Every FR2 frame is regenerated
// here (verbatim when no override matches, spliced when one does), so unmatched
// slots are reproduced byte-for-byte with their CRC recomputed to the original
// value - not bit-verbatim like the mirror, but byte-identical on decode.
//
// IDEAL (bench follow-up): a zero-added-latency CUT-THROUGH. Recognition of the
// match completes at the header/payload boundary (slot ID + cycle are in the
// header), and the override bytes are pre-stored, so a mixed-mode TX PIO could
// mirror verbatim up to the replace window, substitute the stored bytes, then
// emit a freshly recomputed CRC in place of the trailing CRC bytes - keeping the
// frame in its own slot. The CRC is ready one byte-time after the last payload
// byte (the CPU has ~1500 cycles/byte at 150 MHz), exactly when it must go out.
// That needs a new PIO path and bit-level alignment trimming on the bench; it is
// deliberately left as the documented next step rather than shipped unverified.
// ===========================================================================
#include "flexray_inject_tx.h"
#include "flexray_inject_build.h"
#include "flexray_injector.h"
#include "config.h"
#include "protocol.h"
#include "flexray_inject.pio.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

static PIO  s_pio = pio1;
static uint s_sm, s_offset;
static int  s_dma = -1;

// One frame as TX FIFO words: word 0 = byte count N, words 1..N = each data byte
// left-justified (byte << 24). A whole frame far exceeds the 8-deep PIO FIFO, so
// DMA streams it into the FIFO paced by the SM's TX DREQ. Sized for the largest
// frame (header + max payload + CRC) plus the count word.
static uint32_t s_txwords[1 + FR_INJECT_FRAME_MAX];

void flexray_inject_tx_init(void) {
    // Listened (source) channel FR2 stays in receive: release its TXEN (high).
    gpio_init(FR2_TXEN_PIN);
    gpio_put(FR2_TXEN_PIN, 1);
    gpio_set_dir(FR2_TXEN_PIN, GPIO_OUT);

    // Start the injected-frame transmitter on the destination channel FR1.
    s_offset = pio_add_program(s_pio, &flexray_inject_program);
    s_sm     = pio_claim_unused_sm(s_pio, true);
    flexray_inject_program_init(s_pio, s_sm, s_offset, FR1_TXD_PIN, FR1_TXEN_PIN);

    // DMA channel that feeds the SM's TX FIFO, paced by its DREQ.
    s_dma = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config((uint)s_dma);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(s_pio, s_sm, true /* TX */));
    dma_channel_configure((uint)s_dma, &cfg,
                          &s_pio->txf[s_sm],   // write: SM TX FIFO
                          s_txwords,           // read: the word buffer
                          0, false);           // count set per frame; don't start yet
}

// Stream a rebuilt frame to the transmitter via DMA. Drops the frame if the
// previous one is still transmitting (the DMA channel is busy), so a slow bus
// never wedges the capture loop - store-and-forward injection already accepts
// that an occasional frame is not reproduced.
static void send_bytes(const uint8_t *bytes, int len) {
    if (len < 1 || s_dma < 0) return;
    if (dma_channel_is_busy((uint)s_dma)) return;   // previous frame still going -> drop

    s_txwords[0] = (uint32_t)len;
    for (int i = 0; i < len; i++)
        s_txwords[1 + i] = (uint32_t)bytes[i] << 24;

    dma_channel_set_read_addr((uint)s_dma, s_txwords, false);
    dma_channel_set_trans_count((uint)s_dma, (uint32_t)(1 + len), true /* trigger */);
}

bool flexray_inject_forward(const fr_frame_t *f) {
    // Only forward frames from the listened (source) channel onto the destination.
    if (f->channel != PIFLEX_SOURCE_FR2) return false;

    // Default: verbatim rebuild (no window). If injection is enabled and an
    // override matches this slot + cycle, splice its window instead.
    uint8_t off = 0, len = 0;
    const uint8_t *data = NULL;
    fr_inject_match(f->slot_id, f->cycle, &off, &len, &data);

    uint8_t frame[FR_INJECT_FRAME_MAX];
    const int n = fr_inject_build_frame(frame, f, data, off, len);
    if (n <= 0) return false;   // window did not fit -> nothing emitted this frame

    send_bytes(frame, n);
    return true;
}
