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
Pico SDK): `decode_test` exercises framing + CRCs, `record_test` checks the USB
record layout against SavvyCAN's parser offsets for every format, `injector_test`
checks the injector control plane (E2E CRC-8, op `0x90`/`0x91` framing, rule
matching), `inject_build_test` rebuilds an injected frame and decodes it back
to prove the splice + recomputed 24-bit CRC validate, and `source_test` builds
the phase-5 generator's scheduled frames and decodes them back to prove the slot
IDs, the per-channel 24-bit CRC, the 0–63 cycle counter, and the scheduled
payload bytes.

```sh
cd test
gcc -I ../src decode_test.c ../src/flexray_decode.c -o decode_test && ./decode_test
gcc -I ../src record_test.c ../src/flexray_record.c ../src/flexray_decode.c -o record_test && ./record_test
gcc -I ../src injector_test.c ../src/flexray_injector.c -o injector_test && ./injector_test
gcc -I ../src inject_build_test.c ../src/flexray_inject_build.c ../src/flexray_decode.c -o inject_build_test && ./inject_build_test
gcc -I ../src source_test.c ../src/flexray_source.c ../src/flexray_inject_build.c ../src/flexray_decode.c -o source_test && ./source_test
```

## Status (phase 1)

| Phase | What | State |
|-------|------|-------|
| 1 | FlexRay sniffer → format-1 stream over USB | **working on hardware** — captures FlexRay end-to-end on a Pi_Flex V1 bench board: 3 slots decode byte-exact, cycles 0–63, ~540 fps, 0 resync skips; enumerates as panda; PIO sample timing tuned against a logic capture; frame CRC validated against standard vectors |
| 2 | Device timestamps (format 2) + per-frame sequence (format 3) | **working on hardware** — emits format 2 (device µs + CRC flags) and format 3 (+ a sequence number that exposes USB-FIFO loss) on the host's `SET_RECORD_FORMAT` request. Bench-verified on a Pi_Flex V1 board: format-3 records decode byte-exact, the device clock and sequence number advance per frame, and `frame_crc_ok`/`header_crc_ok` now reach the host both set and clear. The fine clock is sent as zero (the timestamp is read in software at dequeue, not latched in hardware), so the hardware-timestamp flag stays clear |
| 3 | MITM pass-through (both transceivers, per-frame TXEN, 9-bit TSS) | **working on hardware** — bit-mirror bridge: each channel's frames are repeated onto the other with a 9-bit TSS and per-frame TXEN, via a per-direction TX replay PIO. Bench-verified on a Pi_Flex V1 board: steady-state bidirectional bridging is byte-exact (independent far-side listener 100% on all three slots), zero dropped frames, RX capture uncompromised while bridging. A symmetric cross-direction gate + per-frame TXEN watchdog suppress the transceiver's own TXD→RXD echo (the NCV7383 can't blank RxD during TX, so this is handled in firmware). Known characteristic: abruptly cutting the source mid-stream leaves a brief (~1–2 s) self-quenching echo of the in-flight frames before the bus settles to idle; a continuously-running bus doesn't trigger it. Build `-DFR_BRIDGE_ONE_WAY=ON` for isolated forward-path testing |
| 4 | Injector (`0x90`/`0x91`, E2E CRC-8) | **working on hardware** — SavvyCAN drives a payload override on the MITM bridge. Control plane on bulk-OUT EP `0x03`: op `0x91` toggles a global injector-enable (OFF at power-up for safety; the host pushes the desired state on connect), op `0x90` submits a replacement slice `[E2E CRC-8 (AUTOSAR, poly 0x1D, init 0xF1)][payload]` for a slot, verified against the CRC and a static allow-list of injectable slots + replace windows (`src/flexray_injector_rules.h`; ships sample rules for the bench generator's slots `0x001`/`0x002`/`0x004`). TX path (`-DFR_INJECT=ON`): a matched frame is decoded, the override window is spliced into the payload, the 24-bit frame CRC is recomputed, and the frame is re-emitted on FR1 via a dedicated FlexRay TX PIO (a CPU bit-serializer drives a uniform shifter PIO) — store-and-forward (~1 frame of latency), as an in-flight bit-mirror cannot change a byte and recompute the trailing CRC in the same slot (zero-latency cut-through is the documented follow-up). Bench-verified on a Pi_Flex V1 board: a cycle-targeted override lands at the exact window on exactly the targeted cycle with a valid recomputed CRC (the far-side listener and PiFlex's own strict FR1 RX both decode the result, all CRC-ok), non-matched slots and injector-off passthrough stay byte-exact, and there is no capture regression or runaway. Measured store-and-forward latency ~78 µs median (62–122 µs). The override store is last-write-wins per rule, so simultaneous multi-cycle override of one slot is a future enhancement. Default and `-DFR_BRIDGE_ONE_WAY=ON` builds keep the phase-3 bit-mirror unchanged |
| 5 | Source / generator mode (`-DFR_SOURCE`) | **implemented, bench-pending** — the board becomes an active FlexRay signal source: instead of the MITM bridge it transmits its own compile-time static schedule (`src/flexray_source_schedule.h`; ships the bench sample slots `0x001`/`0x002`/`0x004`, 8-byte payloads, both channels). A free-running scheduler walks the slot table once per FlexRay cycle with a 0–63 cycle counter (wraps) written to the header cycle field, paces one slot frame every `FR_SRC_FRAME_PERIOD_US` (named timing `#define`s in `config.h`), and clocks each frame out through the phase-4 CPU bit-serializer + uniform-shifter PIO (`flexray_inject_build` / `flexray_inject.pio`). Dual-channel: two TX state machines on pio1 drive FR1 and FR2, each frame carrying its correct per-channel 24-bit CRC (init `0xFEDCBA` on FR1/bus A, `0xABCDEF` on FR2/bus B); the last payload byte also advances with the cycle so changing data is visible without reading the header. RX capture (pio0) keeps running, so SavvyCAN watches the board's own emission via the normal record stream (the NCV7383 does not blank its RXD during TX, so each channel self-captures). Not a full FlexRay cluster — no coldstart/sync/NIT/bus-guardian; it assumes it is the sole source on the bench bus. Host round-trip test `source_test` passes. Default and `-DFR_BRIDGE_ONE_WAY`/`-DFR_INJECT` builds are unchanged (the source TUs are compiled only under `-DFR_SOURCE`). **Not yet verified on hardware.** |
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
