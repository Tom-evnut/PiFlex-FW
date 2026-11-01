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
| 1 | FlexRay sniffer → format-1 stream over USB | **scaffolded** — USB + record encoder wired; PIO decoder is a stub (`src/flexray_rx.c`) |
| 2 | Device timestamps (format 2) + fine clock / sequence (format 3) | planned |
| 3 | MITM pass-through (both transceivers, per-frame TXEN, 9-bit TSS) | planned |
| 4 | Injector (`0x90`/`0x91`, E2E CRC-8) | planned |
| 5 | Source / generator mode | planned |
| 6 | CAN + the 2× CAN / 2× FlexRay board (`can2040`) | planned |

The core clean-room work — recovering FlexRay byte framing from the oversampled
line and checking the header/frame CRCs — is the `TODO` block in
[`src/flexray_rx.c`](src/flexray_rx.c).

## License

GPL-3.0-or-later. See [`LICENSE`](LICENSE). (The firmware links `can2040`, which
is GPLv3, once CAN support lands.)
