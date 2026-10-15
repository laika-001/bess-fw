#!/usr/bin/env bash
# Download Bosch BSEC2 into third_party/bsec2/ for building the firmware.
#
# This repository does NOT redistribute BSEC. Bosch's license terms for the BSEC
# binaries only permit using them in products with Bosch BME68x sensors, and
# forbid passing the software on to third parties. So each developer gets BSEC
# from Bosch directly, accepting Bosch's terms:
#   https://www.bosch-sensortec.com/media/boschsensortec/downloads/software/bme688_development_software/2024_12/20241219_clickthrough_license_terms_bsec_bme680_bme688_bme690.pdf
#
# Usage: tools/fetch_bsec.sh [--accept-bosch-license]
# Downloads the exact files the firmware was built and tested with (Bosch-BSEC2-Library
# commit 4f559a6, BSEC v2.6.1.0) and checks their SHA-256 before installing them.
set -euo pipefail
cd "$(dirname "$0")/.."

REPO=https://raw.githubusercontent.com/boschsensortec/Bosch-BSEC2-Library
COMMIT=4f559a6aa450f0ce436e602b42dc384f52128c66
DEST=third_party/bsec2

# destination | path in Bosch's repo | sha256
FILES="
inc/bsec_datatypes.h|src/inc/bsec_datatypes.h|a632c78ca4e38ad7773fe3493ecb2ff2054a0a5eec429c091db9bfe7ed55f927
inc/bsec_interface.h|src/inc/bsec_interface.h|7643d4dbc0958c9403d5477eb73f9453e54cf69d3ba99cab1c877a0cd4570ae1
cortex-m4/libalgobsec.a|src/cortex-m4/libalgobsec.a|6bf158ee5bbd121f1d64c3168aa343e16b27c5e22218f1d67da74b7d503601a3
config/bme688_sel_33v_3s_4d/bsec_selectivity.txt|src/config/bme688/bme688_sel_33v_3s_4d/bsec_selectivity.txt|65b0ea03a890d06445603e015cd085198d00f114568099fe251f0b4d232a4711
LICENSE.md|LICENSE.md|a27fd42924746a4b45808c21dc71f7c0e6006b4ce9f933d2301e154ed4255d76
"

all_present() {
    while IFS='|' read -r dst _ sum; do
        [ -n "$dst" ] || continue
        [ -f "$DEST/$dst" ] && [ "$(shasum -a 256 "$DEST/$dst" | cut -d' ' -f1)" = "$sum" ] || return 1
    done <<< "$FILES"
}

if all_present; then
    echo "BSEC already in $DEST (checksums match)"
    exit 0
fi

if [ "${1:-}" != "--accept-bosch-license" ]; then
    echo "BSEC2 is licensed by Bosch Sensortec and is not part of this repository."
    echo "Read the terms: https://www.bosch-sensortec.com/media/boschsensortec/downloads/software/bme688_development_software/2024_12/20241219_clickthrough_license_terms_bsec_bme680_bme688_bme690.pdf"
    read -r -p "Do you accept Bosch's BSEC license terms? [y/N] " ans
    case "$ans" in y|Y|yes|YES) ;; *) echo "Not downloaded. The firmware needs BSEC to build."; exit 1 ;; esac
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
while IFS='|' read -r dst src sum; do
    [ -n "$dst" ] || continue
    mkdir -p "$tmp/$(dirname "$dst")"
    curl -fsSL --connect-timeout 20 --max-time 120 --retry 4 --retry-all-errors -o "$tmp/$dst" "$REPO/$COMMIT/$src"
    got="$(shasum -a 256 "$tmp/$dst" | cut -d' ' -f1)"
    if [ "$got" != "$sum" ]; then
        echo "checksum mismatch for $src (got $got)"
        exit 1
    fi
done <<< "$FILES"
mkdir -p "$DEST"
cp -R "$tmp"/. "$DEST"/
echo "BSEC v2.6.1.0 installed in $DEST (Bosch-BSEC2-Library ${COMMIT:0:7}, checksums verified)"
