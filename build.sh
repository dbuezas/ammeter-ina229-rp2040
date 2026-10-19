#!/usr/bin/env bash
# Build the ammeter firmware and, if the RP2040 is in bootloader mode, flash it.
#
# picotool finds the board both in BOOTSEL mode and while it runs this firmware, so the
# button is not needed.
set -euo pipefail
cd "$(dirname "$0")"

export PICO_SDK_PATH="${PICO_SDK_PATH:-$HOME/pico-sdk}"
[ -d "$PICO_SDK_PATH" ] || { echo "no pico-sdk at $PICO_SDK_PATH"; exit 1; }

# Homebrew's arm-none-eabi-gcc has no newlib, so linking fails. ARM's own toolchain has
# it; use that when it is in ~/arm-gnu-toolchain.
if [ -d "$HOME/arm-gnu-toolchain/bin" ]; then
    export PICO_TOOLCHAIN_PATH="$HOME/arm-gnu-toolchain"
    export PATH="$PICO_TOOLCHAIN_PATH/bin:$PATH"
fi

cmake -S firmware -B build -DPICO_BOARD=waveshare_rp2040_zero >/dev/null
cmake --build build -j"$(sysctl -n hw.ncpu)"

UF2=build/ina229_ammeter.uf2
ls -l "$UF2"

if [ "${1:-}" = "--flash" ]; then
    # picotool, not `cp` to /Volumes/RPI-RP2: the disk disappears as soon as the board
    # reboots, and cp can fail with the image only partly written.
    picotool load "$UF2" -f -x
    echo "flashed; the board reboots and enumerates as a USB serial port"
fi
