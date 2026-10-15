#!/usr/bin/env bash
# Recover a board that is silent (no serial output, no LED) because an on-board
# chip is holding the I2C2 data line low. The startup code (mbed-os
# low_level_init) writes to the MAX14690 PMIC on that bus with no timeout, so it
# hangs before main() and the watchdog never gets armed.
#
# This pulses I2C2's SCL 9 times through the I2CM2 bit-bang register so the
# stuck device finishes its byte and lets go, then resets the MCU.
# Unplugging USB also clears it.
set -euo pipefail
source "$(dirname "$0")/env.sh"

BB=0x40018028  # I2CM2 bb register: bit0 SCL out, bit1 SDA out, bit2 SCL in, bit3 SDA in

state() {
    pyocd commander -t "$BESS_PYOCD_TARGET" -c "halt" -c "read32 $BB 4" -c "go" 2>/dev/null \
        | awk '/^40018028:/{print $2}'
}

before="$(state)"
if [ -z "$before" ]; then
    echo "Can't reach the MAX32630 over SWD (debugger connected, chip not answering)."
    echo "Check the board's own USB power and the 10-pin SWD ribbon, then retry."
    exit 1
fi
echo "I2C2 line state before: 0x$before (0000000f = both lines high, 00000007 = SDA stuck low)"

args=(-c "halt")
for _ in 1 2 3 4 5 6 7 8 9; do
    args+=(-c "write32 $BB 0x2" -c "write32 $BB 0x3")
done
pyocd commander -t "$BESS_PYOCD_TARGET" "${args[@]}" >/dev/null 2>&1

after="$(state)"
echo "I2C2 line state after:  0x$after"
pyocd reset -t "$BESS_PYOCD_TARGET" >/dev/null 2>&1
echo "board reset"
