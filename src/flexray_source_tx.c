// PiFlex-FW - FlexRay source / generator TX path (hardware glue, phase 5).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// ===========================================================================
// SOURCE MODE - the board as an active FlexRay signal generator
// ===========================================================================
// Phases 1-4 only ever react to traffic (sniff, bridge, splice). Source mode
// makes the board the TRANSMITTER: it owns FR1 and FR2 TXD/TXEN and emits its
// own static schedule, so one Pi_Flex can replace the bench signal generator.
//
// This is NOT a full FlexRay cluster - there is no coldstart, sync, NIT or bus
// guardian. The board is assumed to be the sole source on the bench bus, so a
// free-running static-segment emitter with a 0..63 cycle counter is enough: it
// walks FR_SOURCE_SCHEDULE once per cycle, encodes each slot's frame with the
// phase-4 encoder (fr_source_build_frame -> fr_inject_build_frame, so framing
// is byte-identical to the injector), and clocks it onto the configured
// channel(s) through the uniform-shifter inject PIO (flexray_inject.pio).
//
// Two SMs run on pio1 (one per channel) so a slot can be emitted on FR1 and FR2
// together, each with its own DMA feeding its TX FIFO and its own serialised
// word buffer. The frame CRC is computed per channel (init 0xFEDCBA for FR1 /
// bus A, 0xABCDEF for FR2 / bus B), so each bus carries its own valid CRC.
//
// RX capture (pio0) is left running: the generator and the two RX decoders use
// different PIO blocks and different pins, so SavvyCAN still sees a record
// stream. On this board the NCV7383 cannot blank its own RXD during TX, so each
// channel's transmission echoes back into its own receiver - the board observes
// what it emits. That self-capture is the intended way to watch the generator
// over USB; whether both channels' echoes are clean is a bench confirmation.
//
// Pacing is time-driven from the main loop (flexray_source_tick): one slot every
// FR_SRC_FRAME_PERIOD_US, non-blocking - a channel whose previous frame is still
// in flight is skipped for this slot rather than stalling the loop (which must
// keep servicing USB and RX). Timing constants live in config.h.
// ===========================================================================
#include "flexray_source_tx.h"
#include "flexray_source.h"
#include "flexray_source_schedule.h"
#include "flexray_source_cfg.h"     // live RAM schedule + timing/run-state
#include "flexray_inject_build.h"   // fr_inject_serialize, FR_INJECT_*_MAX
#include "config.h"
#include "protocol.h"
#include "flexray_inject.pio.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

static PIO  s_pio = pio1;
static uint s_prog_off;

// One transmitter per channel. Each has its own SM, DMA channel and serialised
// word buffer (word 0 = total_bits-1, words 1.. = the MSB-first bit stream from
// fr_inject_serialize), so both channels can have a frame in flight at once.
typedef struct {
    uint     sm;
    int      dma;
    uint8_t  source;   // PIFLEX_SOURCE_FR1 / FR2 (CRC init + channel-mask select)
    uint8_t  ch_mask;  // FR_SRC_CH_FR1 / FR_SRC_CH_FR2
    uint32_t words[FR_INJECT_WORDS_MAX];
} fr_src_chan_t;

static fr_src_chan_t s_ch[2];

// Free-running schedule cursor.
static int      s_slot_idx;   // index into FR_SOURCE_SCHEDULE
static uint8_t  s_cycle;      // FlexRay cycle 0..63 (wraps)
static uint32_t s_next_us;    // device-clock time the next slot frame is due

static void chan_init(fr_src_chan_t *c, uint8_t source, uint8_t mask,
                      uint txd_pin, uint txen_pin) {
    c->source  = source;
    c->ch_mask = mask;
    c->sm      = pio_claim_unused_sm(s_pio, true);
    flexray_inject_program_init(s_pio, c->sm, s_prog_off, txd_pin, txen_pin);

    c->dma = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config((uint)c->dma);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(s_pio, c->sm, true /* TX */));
    dma_channel_configure((uint)c->dma, &cfg,
                          &s_pio->txf[c->sm],  // write: SM TX FIFO
                          c->words,            // read: this channel's word buffer
                          0, false);           // count set per frame; don't start yet
}

void flexray_source_init(void) {
    // One copy of the uniform-shifter program on pio1 drives both SMs.
    s_prog_off = pio_add_program(s_pio, &flexray_inject_program);

    chan_init(&s_ch[0], PIFLEX_SOURCE_FR1, FR_SRC_CH_FR1, FR1_TXD_PIN, FR1_TXEN_PIN);
    chan_init(&s_ch[1], PIFLEX_SOURCE_FR2, FR_SRC_CH_FR2, FR2_TXD_PIN, FR2_TXEN_PIN);

    s_slot_idx = 0;
    s_cycle    = 0;
    s_next_us  = time_us_32();
}

void flexray_source_deinit(void) {
    // Stop and free both channels' DMA + SM, then the shared pio1 program. The
    // mode manager has already released both TXEN lines, so neither channel drives
    // the bus before teardown.
    for (int i = 0; i < 2; i++) {
        if (s_ch[i].dma >= 0) {
            dma_channel_abort((uint)s_ch[i].dma);
            dma_channel_unclaim((uint)s_ch[i].dma);
            s_ch[i].dma = -1;
        }
        pio_sm_set_enabled(s_pio, s_ch[i].sm, false);
        pio_sm_unclaim(s_pio, s_ch[i].sm);
    }
    pio_remove_program(s_pio, &flexray_inject_program, s_prog_off);
}

// Build and clock out one slot's frame on one channel. Dropped (returns without
// emitting) if that channel's previous frame is still transmitting, so a slow
// bus never wedges the loop - a free-running source can skip an occasional frame.
static void emit_slot(fr_src_chan_t *c, const fr_source_slot_t *slot, uint8_t cycle) {
    if (c->dma < 0) return;
    if (dma_channel_is_busy((uint)c->dma)) return;   // previous frame still going -> skip

    uint8_t bytes[FR_INJECT_FRAME_MAX];
    const int n = fr_source_build_frame(bytes, slot, cycle, c->source);
    if (n <= 0) return;

    const int words = fr_inject_serialize(c->words, bytes, n);
    if (words <= 0) return;

    dma_channel_set_read_addr((uint)c->dma, c->words, false);
    dma_channel_set_trans_count((uint)c->dma, (uint32_t)words, true /* trigger */);
}

void flexray_source_tick(void) {
    // Timing + run-state are host-configurable at runtime (op 0x96). An empty
    // schedule (host cleared every slot) or a stopped generator emits nothing.
    const uint32_t period_us = fr_source_cfg_period_us();
    const int      count     = fr_source_cfg_slot_count();
    if (!fr_source_cfg_enabled() || count <= 0) return;

    const uint32_t now = time_us_32();
    if ((int32_t)(now - s_next_us) < 0) return;      // next slot not due yet
    s_next_us += period_us;
    // If the loop fell far behind (long USB/RX service), resync the cadence to now
    // rather than bursting a backlog of frames.
    if ((int32_t)(now - s_next_us) > (int32_t)period_us)
        s_next_us = now + period_us;

    // The RAM schedule can shrink under us (host sent 0x94/0x95); keep the cursor
    // in range before indexing.
    if (s_slot_idx >= count) s_slot_idx = 0;
    const fr_source_slot_t *slot = fr_source_cfg_slot(s_slot_idx);
    if (!slot) return;

    // Emit this slot on every channel it is scheduled for. The two SMs are
    // independent, so FR1 and FR2 frames are launched back-to-back (effectively
    // together), each with its own per-channel CRC.
    for (int i = 0; i < 2; i++)
        if (slot->channels & s_ch[i].ch_mask)
            emit_slot(&s_ch[i], slot, s_cycle);

    // Advance the schedule; wrap to the next 0..63 cycle at the end of the table.
    if (++s_slot_idx >= count) {
        s_slot_idx = 0;
        s_cycle = (uint8_t)((s_cycle + 1) & 0x3F);
        s_next_us += fr_source_cfg_cycle_gap_us();   // optional cycle-boundary idle
    }
}
