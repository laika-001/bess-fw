#!/usr/bin/env bash
# Open the board's serial console (115200 baud). Exit screen with Ctrl-A then K.
set -euo pipefail
source "$(dirname "$0")/env.sh"

port="$(bess_port)"
if [ -z "$port" ]; then
    echo "No /dev/cu.usbmodem* port found. Is the board plugged in?"
    exit 1
fi
exec screen "$port" 115200
