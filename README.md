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

> ✅ **Confirmed on the bench.** The GPIO map in [`src/config.h`](src/config.h)
> (FR1 RXD/TXD/TXEN = 26/28/27, FR2 = 6/4/5, STBN=3, BGE=2) is verified on a
> physical Pi_Flex V1 board — FlexRay decodes end-to-end on both channels.

## Host side

Pairs with the `PicoFlexRay` connection in [SavvyCAN](https://github.com/collin80/SavvyCAN)
(on the `fork-ci` fork). The device enumerates Panda-compatible (VID `0x3801`,
PID `0xDDCC`, serial `picoflex…`) so SavvyCAN binds unchanged. The USB contract
lives in [`src/protocol.h`](src/protocol.h).

## Build

Needs the Pico SDK, an `arm-none-eabi-gcc` toolchain, CMake >= 3.13, Ninja, and a
host C/C++ compiler (the SDK builds `pioasm`/`picotool` natively). The
arduino-pico core bundles a usable `arm-none-eabi-gcc` if you already have it.

```sh
export PICO_SDK_PATH=/path/to/pico-sdk       # RP2350-capable SDK
./build.sh                                   # also honours PICO_TOOLCHAIN_PATH
```

Flash `build/piflex_fw.uf2` via BOOTSEL. After the first flash, the host can
re-enter the bootloader for the next reflash without touching the board by
sending the `0xD1` vendor control request (`PIFLEX_ENTER_BOOTLOADER`).

Known-good on Windows: Pico SDK (RP2350), `arm-none-eabi-gcc` 10.3 (arduino-pico
`pqt-gcc`), CMake 3.30 + Ninja, MinGW g++ for the host tools.

## Tests

The SDK-free cores have host round-trip tests that need only a host compiler (no
Pico SDK): `decode_test` exercises framing + CRCs, and `record_test` checks the
USB record layout against SavvyCAN's parser offsets for every format.

```sh
cd test
gcc -I ../src decode_test.c ../src/flexray_decode.c -o decode_test && ./decode_test
gcc -I ../src record_test.c ../src/flexray_record.c ../src/flexray_decode.c -o record_test && ./record_test
```

## Status (phase 1)

| Phase | What | State |
|-------|------|-------|
| 1 | FlexRay sniffer → format-1 stream over USB | **working on hardware** — captures FlexRay end-to-end on a Pi_Flex V1 bench board: 3 slots decode byte-exact, cycles 0–63, ~540 fps, 0 resync skips; enumerates as panda; PIO sample timing tuned against a logic capture; frame CRC validated against standard vectors |
| 2 | Device timestamps (format 2) + per-frame sequence (format 3) | **working on hardware** — emits format 2 (device µs + CRC flags) and format 3 (+ a sequence number that exposes USB-FIFO loss) on the host's `SET_RECORD_FORMAT` request. Bench-verified on a Pi_Flex V1 board: format-3 records decode byte-exact, the device clock and sequence number advance per frame, and `frame_crc_ok`/`header_crc_ok` now reach the host both set and clear. The fine clock is sent as zero (the timestamp is read in software at dequeue, not latched in hardware), so the hardware-timestamp flag stays clear |
| 3 | MITM pass-through (both transceivers, per-frame TXEN, 9-bit TSS) | planned |
| 4 | Injector (`0x90`/`0x91`, E2E CRC-8) | planned |
| 5 | Source / generator mode | planned |
| 6 | CAN + the 2× CAN / 2× FlexRay board (`can2040`) | planned |

The core clean-room decode — TSS/FSS/BSS line recovery in
[`src/flexray_rx.pio`](src/flexray_rx.pio), and byte framing + 11-bit header CRC
and 24-bit frame CRC in [`src/flexray_rx.c`](src/flexray_rx.c) — is written and
**proven on hardware**: the PIO sample timing was trimmed against a raw-word
capture, the GPIO pin map is confirmed, and frames decode byte-exact on a bench
board. The frame CRC is also verified against the standard CRC-24/FlexRay check
vectors.

> **Bench note (single-source capture):** the frame CRC is validated per channel
> (init `0xFEDCBA` for channel A / FR1, `0xABCDEF` for channel B / FR2, per the
> FlexRay spec). A signal generator that emits a channel-A CRC on a frame fed to
> FR2 will show `frame_crc_ok = false` even though the payload is intact; this is
> correct spec behaviour, not a decode fault. For such a rig, build with
> `-DFR_FRAME_CRC_ANY_CHANNEL=1` to accept either channel's init (off by default,
> since it weakens wrong-channel detection on a real bus).

## License

GPL-3.0-or-later. See [`LICENSE`](LICENSE). (The firmware links `can2040`, which
is GPLv3, once CAN support lands.)
