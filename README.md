# PiFlex-FW

Open, GPLv3-licensed firmware for Damien Maguire's **Pi_Flex** board — a
Raspberry Pi Pico 2 (RP2350) + onsemi NCV7383 FlexRay front end — that turns it
into a **SavvyCAN-compatible FlexRay (and later CAN) sniffer, injector and
source**.

It is a clean-room replacement for `dynm/pico-flexray`, which ships with **no
license** and therefore cannot be reused. PiFlex-FW is written from the FlexRay
line coding, the NCV7383 datasheet, and the USB contract of our own SavvyCAN
host code — no third-party firmware source is copied.

## Hardware

- **MCU:** Raspberry Pi Pico 2 (RP2350), **non-W** — build `PICO_BOARD pico2`.
- **FlexRay:** 2× onsemi NCV7383 transceivers (channel 1 = bus side A, channel 2
  = bus side B; together a MITM bridge). No on-board termination or relay.
- **CAN:** none on Pi_Flex V1; the 2× CAN + 2× FlexRay variant is planned and
  will use software CAN (`can2040`) on a spare PIO block — no CAN controller IC.
- Board design: <https://github.com/damienmaguire/BMW-F11-EV-Conversion/tree/main/Design/FlexRay>

> ⚠️ **Pin map is provisional.** The GPIO numbers in [`src/config.h`](src/config.h)
> are placeholders tagged `TODO confirm`. Verify them against
> `Pi_Flex_V1.kicad_sch` and the physical board before the first flash.

## Host side

Pairs with the `PicoFlexRay` connection in [SavvyCAN](https://github.com/collin80/SavvyCAN)
(on the `fork-ci` fork). The device enumerates Panda-compatible (VID `0x3801`,
PID `0xDDCC`, serial `picoflex…`) so SavvyCAN binds unchanged. The USB contract
lives in [`src/protocol.h`](src/protocol.h).

## Build

```sh
export PICO_SDK_PATH=/path/to/pico-sdk   # SDK 2.x (RP2350 support)
cmake -B build -S .
cmake --build build
```

Flash `build/piflex_fw.uf2` via BOOTSEL.

## Status (phase 1)

| Phase | What | State |
|-------|------|-------|
| 1 | FlexRay sniffer → format-1 stream over USB | **decoder drafted** — PIO BSS-sync deserializer + C framing + header/frame CRC written; frame CRC validated against standard vectors, **PIO timing untuned on hardware** (`<<TUNE>>` marks in `src/flexray_rx.pio`) |
| 2 | Device timestamps (format 2) + fine clock / sequence (format 3) | planned |
| 3 | MITM pass-through (both transceivers, per-frame TXEN, 9-bit TSS) | planned |
| 4 | Injector (`0x90`/`0x91`, E2E CRC-8) | planned |
| 5 | Source / generator mode | planned |
| 6 | CAN + the 2× CAN / 2× FlexRay board (`can2040`) | planned |

The core clean-room decode — TSS/FSS/BSS line recovery in
[`src/flexray_rx.pio`](src/flexray_rx.pio), and byte framing + 11-bit header CRC
and 24-bit frame CRC in [`src/flexray_rx.c`](src/flexray_rx.c) — is written. The
frame CRC is verified against the standard CRC-24/FlexRay check vectors. What
remains is **tuning the PIO sample timing and idle threshold on the bench** with
a logic analyser (the `<<TUNE>>` markers), and confirming the header-CRC init
(0x1A) and the GPIO pin map against the hardware.

## License

GPL-3.0-or-later. See [`LICENSE`](LICENSE). (The firmware links `can2040`, which
is GPLv3, once CAN support lands.)
