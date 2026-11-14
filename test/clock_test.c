// PiFlex-FW - host test for the FlexRay PIO clock-divider integrality invariant.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The three FlexRay PIO programs derive their SM clock from clk_sys with a
// floating divider:
//   RX decoder (flexray_rx.pio):              clk_sys / (FR_BITRATE_HZ * FR_OVERSAMPLE)
//   source/inject emitter (flexray_inject.pio): clk_sys / (FR_BITRATE_HZ * 2)
// A FRACTIONAL divider makes the PIO fractional clock divider dither each SM tick
// by up to one system clock, which jitters the RX sample points and the emitted
// edges. Compounded with the bit-mirror's copy-loop edge quantization that
// dither eats the receiver's bit-centre margin and mis-samples bits next to
// transitions (the ~15% bridged-frame corruption seen on the two-board rig). So
// the system clock MUST be an exact multiple of both FlexRay PIO rates, i.e.
// every divider integer. This test pins that invariant for the configured clock
// (FR_SYS_CLK_KHZ) and documents that the old 150 MHz default violated it.
//
// SDK-free: the values below MIRROR src/config.h (which pulls in the Pico SDK and
// so can't be included on the host). Keep them in sync with config.h.
//
// Build & run (host compiler, no Pico SDK needed):
//   gcc -I ../src clock_test.c -o clock_test && ./clock_test
#include <stdio.h>
#include <stdint.h>

// --- mirror of src/config.h -------------------------------------------------
#define FR_BITRATE_HZ   10000000u   // 10 Mbit/s
#define FR_OVERSAMPLE   8u          // RX PIO samples per bit (flexray_rx.pio)
#define FR_EMIT_CYCLES  2u          // emitter PIO cycles per bit (flexray_inject.pio)
#define FR_SYS_CLK_KHZ  160000u     // configured system clock (config.h)
// ---------------------------------------------------------------------------

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } } while (0)

// True iff clk_sys (Hz) divides evenly into an integer PIO divider for a rate of
// (FR_BITRATE_HZ * mult) ticks/s -- i.e. clk_sys is an exact multiple of the rate.
static int divider_is_integer(uint64_t clk_hz, uint64_t mult) {
    uint64_t rate = (uint64_t)FR_BITRATE_HZ * mult;
    return (clk_hz % rate) == 0;
}

int main(void) {
    const uint64_t clk = (uint64_t)FR_SYS_CLK_KHZ * 1000ull;

    // The configured clock must give an INTEGER divider for both FlexRay paths.
    CHECK(divider_is_integer(clk, FR_OVERSAMPLE),
          "RX decoder divider must be integer at FR_SYS_CLK_KHZ");
    CHECK(divider_is_integer(clk, FR_EMIT_CYCLES),
          "source/inject emitter divider must be integer at FR_SYS_CLK_KHZ");

    // Spell out the actual divider values so a wrong clock choice is obvious.
    CHECK(clk / ((uint64_t)FR_BITRATE_HZ * FR_OVERSAMPLE) == 2
          && clk % ((uint64_t)FR_BITRATE_HZ * FR_OVERSAMPLE) == 0,
          "RX divider should be exactly 2.0 at 160 MHz");
    CHECK(clk / ((uint64_t)FR_BITRATE_HZ * FR_EMIT_CYCLES) == 8
          && clk % ((uint64_t)FR_BITRATE_HZ * FR_EMIT_CYCLES) == 0,
          "emitter divider should be exactly 8.0 at 160 MHz");

    // Regression guard: the previous default (150 MHz) was fractional on BOTH
    // paths -- the root cause this change fixes. If someone reverts the clock to
    // 150 MHz this documents why that re-breaks bridge fidelity.
    const uint64_t old_clk = 150000000ull;
    CHECK(!divider_is_integer(old_clk, FR_OVERSAMPLE),
          "sanity: 150 MHz RX divider is fractional (1.875) - the bug");
    CHECK(!divider_is_integer(old_clk, FR_EMIT_CYCLES),
          "sanity: 150 MHz emitter divider is fractional (7.5) - the bug");

    // Any valid clock must be a multiple of lcm(80 MHz, 20 MHz) = 80 MHz.
    CHECK(clk % 80000000ull == 0,
          "FR_SYS_CLK_KHZ must be a multiple of 80 MHz (both dividers integer)");

    if (failures == 0) {
        printf("clock_test: all checks passed (clk=%lu Hz: RX div=%lu, emit div=%lu)\n",
               (unsigned long)clk,
               (unsigned long)(clk / ((uint64_t)FR_BITRATE_HZ * FR_OVERSAMPLE)),
               (unsigned long)(clk / ((uint64_t)FR_BITRATE_HZ * FR_EMIT_CYCLES)));
        return 0;
    }
    printf("clock_test: %d FAILURES\n", failures);
    return 1;
}
