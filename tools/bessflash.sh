#!/usr/bin/env bash
# Build the firmware and flash it to the MAX32630FTHR over DAPLink.
# Usage: tools/bessflash.sh [--build-only]
set -euo pipefail
source "$(dirname "$0")/env.sh"
cd "$BESS_ROOT"

# cmake_build caches absolute paths. If the folder was copied from another
# machine or user, the cache points at the old location and the build fails.
cache="cmake_build/$BESS_TARGET/develop/GCC_ARM/CMakeCache.txt"
if [ -f "$cache" ] && ! grep -q "CMAKE_HOME_DIRECTORY:INTERNAL=$BESS_ROOT\$" "$cache"; then
    echo "cmake_build was made in a different folder; removing it for a clean build"
    rm -rf cmake_build
fi
# It also caches the toolchain's absolute paths (Homebrew cask vs ARM's .pkg
# install in /Applications). If the cached archiver is gone, start clean.
if [ -f "$cache" ]; then
    cached_ar="$(sed -n 's/^CMAKE_AR:FILEPATH=//p' "$cache")"
    if [ -n "$cached_ar" ] && [ ! -x "$cached_ar" ]; then
        echo "cmake_build points at a toolchain that isn't installed ($cached_ar); removing it for a clean build"
        rm -rf cmake_build
    fi
fi

# The memap post-step prints "FAILED: bess-blinky.elf" even when the .bin is
# fine, so success is judged by the "-- built:" line, not the exit status.
# When nothing changed, ninja prints "no work to do" and no "-- built:" line.
log="$(mktemp)"
mbed-tools compile -m "$BESS_TARGET" -t GCC_ARM >"$log" 2>&1 || true
grep -E 'error:|warning: .*main\.cpp' "$log" || true
if ! grep -q -- "-- built: .*bess-blinky.bin" "$log" &&
   ! { grep -q "ninja: no work to do" "$log" && [ -f "$BESS_BIN" ]; }; then
    echo "BUILD FAILED, full log: $log"
    exit 1
fi
rm -f "$log"
echo "built: $BESS_BIN"

[ "${1:-}" = "--build-only" ] && exit 0
pyocd flash -t "$BESS_PYOCD_TARGET" "$BESS_BIN"
