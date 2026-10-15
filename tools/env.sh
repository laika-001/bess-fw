# Shared settings for the BESS firmware tools. Sourced by the other scripts.

BESS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
BESS_VENV="${BESS_VENV:-$HOME/.venvs/mbed}"
BESS_TARGET=MAX32630FTHR
BESS_PYOCD_TARGET=max32630
BESS_BIN="$BESS_ROOT/cmake_build/$BESS_TARGET/develop/GCC_ARM/bess-blinky.bin"

# mbed-tools and pyocd live in the venv; cmake and ninja in Homebrew.
# arm-none-eabi-gcc comes from Homebrew's gcc-arm-embedded cask or, when that
# cask's checksum is stale, from ARM's own .pkg in /Applications/ArmGNUToolchain.
export PATH="$BESS_VENV/bin:/opt/homebrew/bin:$PATH"
for d in /Applications/ArmGNUToolchain/*/arm-none-eabi/bin; do
    [ -x "$d/arm-none-eabi-gcc" ] && export PATH="$PATH:$d" && break
done

# The DAPLink serial port's suffix changes between machines and USB ports.
bess_port() {
    ls /dev/cu.usbmodem* 2>/dev/null | head -1
}
