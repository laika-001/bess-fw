#!/usr/bin/env bash
# One-time setup of the BESS firmware toolchain on a new Mac.
# Safe to re-run: every step checks before it changes anything.
set -euo pipefail
source "$(dirname "$0")/env.sh"
cd "$BESS_ROOT"

step() { printf '\n== %s\n' "$*"; }

step "Homebrew"
if ! command -v brew >/dev/null; then
    echo "Homebrew is not installed. Install it from https://brew.sh, then re-run this script."
    exit 1
fi
# gcc-arm-embedded is a .pkg installer, so this may ask for your password.
# If the compiler is already installed (for example from ARM's own .pkg, when
# the cask's checksum is out of date), skip the cask instead of failing on it.
if command -v arm-none-eabi-gcc >/dev/null; then
    echo "arm-none-eabi-gcc already installed at $(command -v arm-none-eabi-gcc); skipping the cask"
    HOMEBREW_BUNDLE_CASK_SKIP=gcc-arm-embedded brew bundle --file tools/Brewfile
else
    brew bundle --file tools/Brewfile
fi

step "Python venv at $BESS_VENV"
if [ ! -x "$BESS_VENV/bin/python3" ]; then
    # Prefer the Command Line Tools Python 3.9, which the bundled wheels are built for.
    py=/usr/bin/python3
    command -v "$py" >/dev/null || py="$(command -v python3)"
    mkdir -p "$(dirname "$BESS_VENV")"
    "$py" -m venv "$BESS_VENV"
fi
"$BESS_VENV/bin/python3" --version

step "Python packages (mbed-tools, pyocd, ...)"
# The wheels in tools/wheels are for Apple silicon + Python 3.9. On anything
# else, fall back to downloading the same pinned versions from PyPI.
if ! "$BESS_VENV/bin/pip" install -q --no-index --find-links tools/wheels -r tools/requirements.txt; then
    echo "offline install failed; downloading the pinned versions from PyPI instead"
    "$BESS_VENV/bin/pip" install -q -r tools/requirements.txt
fi
"$BESS_VENV/bin/mbed-tools" --version
"$BESS_VENV/bin/pyocd" --version

step "mbed-os source"
pin="$(sed 's/.*#//' mbed-os.lib)"
if [ ! -d mbed-os/.git ]; then
    git clone https://github.com/ARMmbed/mbed-os.git mbed-os
fi
if [ "$(git -C mbed-os rev-parse HEAD)" != "$pin" ]; then
    git -C mbed-os fetch -q origin
    git -C mbed-os checkout -q "$pin"
fi
echo "mbed-os at $(git -C mbed-os rev-parse --short HEAD)"

step "Bosch BSEC2 (not part of this repository: Bosch license)"
tools/fetch_bsec.sh

step "Shell commands bessflash / bessmon in ~/.zshrc"
marker="# >>> bess-fw tools >>>"
if grep -qF "$marker" "$HOME/.zshrc" 2>/dev/null; then
    echo "already present"
else
    cat >>"$HOME/.zshrc" <<EOF

$marker
bessflash() { "$BESS_ROOT/tools/bessflash.sh" "\$@"; }
bessmon() { "$BESS_ROOT/tools/monitor.sh"; }
# <<< bess-fw tools <<<
EOF
    echo "added; open a new terminal (or run: source ~/.zshrc) to use them"
fi

step "Toolchain check"
arm-none-eabi-gcc --version | head -1
cmake --version | head -1
echo "ninja $(ninja --version)"

step "Done"
echo "Plug in the board, then run: bessflash   (build + flash)   and   bessmon   (serial console)"
