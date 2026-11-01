#!/usr/bin/env bash
# PiFlex-FW build helper (git-bash / MSYS on Windows, or Linux/macOS).
#
# Needs: the Pico SDK, an arm-none-eabi-gcc toolchain, CMake >= 3.13, Ninja, and
# a host C/C++ compiler (the SDK builds pioasm/picotool natively). Override any
# path with the matching environment variable before running.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

: "${PICO_SDK_PATH:=$HOME/pico-sdk}"
export PICO_SDK_PATH

# ARM toolchain: use PICO_TOOLCHAIN_PATH if set, else the newest arduino-pico
# pqt-gcc (which bundles arm-none-eabi-gcc) if that core is installed.
if [ -z "${PICO_TOOLCHAIN_PATH:-}" ]; then
  cand="$(ls -d "$HOME/AppData/Local/Arduino15/packages/rp2040/tools/pqt-gcc"/*/ 2>/dev/null | sort | tail -1 || true)"
  [ -n "$cand" ] && export PICO_TOOLCHAIN_PATH="${cand%/}"
fi
[ -n "${PICO_TOOLCHAIN_PATH:-}" ] && export PATH="$PICO_TOOLCHAIN_PATH/bin:$PATH"

# Portable CMake/Ninja under ~/pico-tools (added to PATH if present).
[ -d "$HOME/pico-tools" ] && \
  export PATH="$HOME/pico-tools:$HOME/pico-tools/cmake-3.30.5-windows-x86_64/bin:$PATH"

build="${PIFLEX_BUILD_DIR:-$root/build}"
cmake -S "$root" -B "$build" -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build "$build"
echo "Built: $build/piflex_fw.elf  ($build/piflex_fw.uf2)"
