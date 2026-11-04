// PiFlex-FW - FlexRay channel capture (hardware glue).
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Starts the two PIO decoders (one per NCV7383 channel) and drains their FIFOs
// into the SDK-free decode core (flexray_decode.c). The PIO program recovers
// byte framing; the core does the frame assembly and CRC checks.
//
// Each PIO RX FIFO is only four words deep, so a FlexRay burst can overrun it in
// a few microseconds while the main loop is busy in tud_task()/send_frame().
// To stop dropping capture under USB load, a per-channel DMA channel is paced by
// the PIO RX DREQ and streams the words into a RAM ring buffer; flexray_rx_poll()
// then drains the ring (not the FIFO) and feeds the decode core exactly as before.
// See the ring-sizing and overflow notes near the constants below.
#include "flexray_rx.h"
#include "flexray_decode.h"
#include "flexray_rx.pio.h"
#include "config.h"
#include "protocol.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// --- Ring sizing -----------------------------------------------------------
// The PIO emits one 32-bit word per received FlexRay byte. A byte on the wire is
// 2 BSS bits + 8 data bits = 10 bit-times; at 10 Mbit/s that is 1.0 us/word, so
// the worst-case sustained capture rate is ~1 word per microsecond per channel.
//
// The Pico's USB is full-speed, serviced roughly once per 1 ms USB frame, and a
// single missed service (or a long send_frame()) stalls the main loop for about
// that long. A 2048-word ring covers ~2.0 ms of back-to-back worst-case capture
// per channel - two full missed USB frames of slack, and far more in practice
// because FlexRay is not 100% busy (idle gaps between slots and cycles). At 8 KB
// per channel (16 KB for both) this is cheap against the RP2350's 520 KB SRAM;
// bump FR_RX_RING_WORDS_LOG2 if a future TX load needs a deeper cushion.
//
// Must stay a power of two: the DMA write-ring wrap works on a 2^n-aligned buffer
// and we mask the read index with (FR_RX_RING_WORDS - 1).
#define FR_RX_RING_WORDS_LOG2   11u
#define FR_RX_RING_WORDS        (1u << FR_RX_RING_WORDS_LOG2)          // 2048 words
#define FR_RX_RING_MASK         (FR_RX_RING_WORDS - 1u)
#define FR_RX_RING_BYTES_LOG2   (FR_RX_RING_WORDS_LOG2 + 2u)           // *4 bytes/word
_Static_assert((FR_RX_RING_WORDS & FR_RX_RING_MASK) == 0u, "ring must be a power of two");

static PIO  s_pio = pio0;
static uint s_offset;

// Per-channel capture state. Index 0 = FR1, index 1 = FR2.
typedef struct {
    uint            sm;        // PIO state machine driving this channel
    uint            dma;       // DMA channel streaming the RX FIFO into the ring
    uint8_t         source;    // PIFLEX_SOURCE_FR1 / FR2 tag for the decoder
    fr_decode_t     dec;       // reassembly state (unchanged core)
    volatile uint32_t laps;    // ring wraparounds, bumped in the DMA IRQ (see below)
    uint32_t        rd;        // absolute words consumed (wraps mod 2^32, like the producer)
    volatile uint32_t overflow;// times the DMA lapped the reader and we dropped capture
    // 2^n-aligned so the DMA write-ring wrap stays inside the buffer.
    uint32_t        ring[FR_RX_RING_WORDS] __attribute__((aligned(1u << FR_RX_RING_BYTES_LOG2)));
} fr_rx_chan_t;

static fr_rx_chan_t s_chan[2];

// DMA completion IRQ: each channel self-triggers after FR_RX_RING_WORDS transfers
// (one full lap of the ring), which fires this handler. Counting laps here turns
// the hardware write pointer - which only ever reports a position inside the ring
// - into a true absolute "words produced" count (laps * RING_WORDS + position), so
// a reader that falls a full lap behind is detectable rather than silently lapped.
static void fr_rx_dma_irq(void) {
    for (int i = 0; i < 2; i++) {
        uint ch = s_chan[i].dma;
        if (dma_channel_get_irq0_status(ch)) {
            dma_channel_acknowledge_irq0(ch);
            s_chan[i].laps++;
        }
    }
}

static void fr_rx_chan_start_dma(fr_rx_chan_t *c) {
    c->dma = (uint)dma_claim_unused_channel(true);

    dma_channel_config cfg = dma_channel_get_default_config(c->dma);
    channel_config_set_read_increment(&cfg, false);                   // fixed PIO FIFO
    channel_config_set_write_increment(&cfg, true);                   // sweep the ring
    channel_config_set_dreq(&cfg, pio_get_dreq(s_pio, c->sm, false)); // paced by RX FIFO
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_ring(&cfg, true /* wrap writes */, FR_RX_RING_BYTES_LOG2);

    dma_channel_set_irq0_enabled(c->dma, true);

    // Self-triggering endless capture: after a full lap the channel re-arms itself
    // (write address has wrapped back to the ring base) and raises the IRQ. No CPU
    // intervention, so capture keeps running even while the main loop is stalled.
    dma_channel_configure(c->dma, &cfg,
                          c->ring,                 // write: ring base
                          &s_pio->rxf[c->sm],      // read:  this SM's RX FIFO
                          dma_encode_transfer_count_with_self_trigger(FR_RX_RING_WORDS),
                          true /* start now */);
}

void flexray_rx_init(void) {
    s_offset = pio_add_program(s_pio, &flexray_rx_program);

    s_chan[0].sm = pio_claim_unused_sm(s_pio, true);
    s_chan[1].sm = pio_claim_unused_sm(s_pio, true);
    s_chan[0].source = PIFLEX_SOURCE_FR1;
    s_chan[1].source = PIFLEX_SOURCE_FR2;

    flexray_rx_program_init(s_pio, s_chan[0].sm, s_offset, FR1_RXD_PIN, FR_BITRATE_HZ, FR_OVERSAMPLE);
    flexray_rx_program_init(s_pio, s_chan[1].sm, s_offset, FR2_RXD_PIN, FR_BITRATE_HZ, FR_OVERSAMPLE);

    for (int i = 0; i < 2; i++) {
        fr_decode_reset(&s_chan[i].dec);
        s_chan[i].laps = 0;
        s_chan[i].rd = 0;
        s_chan[i].overflow = 0;
    }

    // Route both capture channels' completion IRQ to our lap counter before arming.
    irq_set_exclusive_handler(DMA_IRQ_0, fr_rx_dma_irq);
    irq_set_enabled(DMA_IRQ_0, true);

    fr_rx_chan_start_dma(&s_chan[0]);
    fr_rx_chan_start_dma(&s_chan[1]);
}

// Absolute count of words the DMA has written for this channel. laps and the
// hardware write pointer are read together with a re-check so a lap landing
// mid-read can't splice a stale lap count onto a freshly wrapped position.
static inline uint32_t fr_rx_produced(const fr_rx_chan_t *c) {
    uint32_t laps, idx, laps2;
    do {
        laps = c->laps;
        idx  = (uint32_t)(((uintptr_t)dma_channel_hw_addr(c->dma)->write_addr
                           - (uintptr_t)c->ring) >> 2);
        laps2 = c->laps;
    } while (laps != laps2);
    return (laps << FR_RX_RING_WORDS_LOG2) + idx;
}

// Drain one channel's ring into its decoder. Returns true (leaving rd on the next
// unconsumed word) as soon as a frame completes, so the caller gets one frame per
// call exactly as the old FIFO path did; remaining words wait for the next call.
static bool fr_rx_drain(fr_rx_chan_t *c, fr_frame_t *out) {
    uint32_t produced = fr_rx_produced(c);

    // Overflow: the writer is more than a whole ring ahead of the reader, so the
    // oldest unread words have been overwritten. Rather than feed spliced garbage
    // (which would corrupt a frame mid-flight), count it, skip to the live head and
    // resync the decoder on the next frame-start. The dropped span shows up to the
    // host as a FlexRay frame gap / sequence-number jump - capture loss stays
    // observable instead of masquerading as clean data.
    if ((uint32_t)(produced - c->rd) > FR_RX_RING_WORDS) {
        c->overflow++;
        c->rd = produced;
        fr_decode_reset(&c->dec);
        return false;
    }

    while (c->rd != produced) {
        uint32_t word = c->ring[c->rd & FR_RX_RING_MASK];
        c->rd++;
        if (fr_decode_feed(&c->dec, word, c->source, out)) return true;
    }
    return false;
}

bool flexray_rx_poll(fr_frame_t *out) {
    if (fr_rx_drain(&s_chan[0], out)) return true;
    if (fr_rx_drain(&s_chan[1], out)) return true;
    return false;
}

// Capture-side ring overflow count for a channel (PIFLEX_SOURCE_FR1 / FR2), for
// diagnostics/telemetry. Additive to the API; the core poll path is unchanged.
uint32_t flexray_rx_overflow_count(uint8_t channel) {
    if (channel == s_chan[0].source) return s_chan[0].overflow;
    if (channel == s_chan[1].source) return s_chan[1].overflow;
    return 0;
}
