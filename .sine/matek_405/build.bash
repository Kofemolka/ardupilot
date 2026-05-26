#!/usr/bin/env bash
set -euo pipefail

. ../.env/bin/activate
export PATH="$HOME/apps/toolchains/xpack/xpack-arm-none-eabi-gcc-13.3.1-1.1/bin:$PATH"

BOARD_NAME="MatekF405-TE"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$REPO_DIR"

./waf configure --board ${BOARD_NAME} --bootloader
./waf bootloader

./waf configure \
    --board ${BOARD_NAME} \
    --extra-hwdef="$SCRIPT_DIR/sine_extra.dat"
./waf plane

BIN_DIR="${REPO_DIR}/build/${BOARD_NAME}/bin"
ZIP_FILE="${REPO_DIR}/build/${BOARD_NAME}_$(date +%Y%m%d_%H%M%S).zip"

echo ""
echo "Zipping ${BIN_DIR} -> ${ZIP_FILE}"
zip -j "${ZIP_FILE}" "${BIN_DIR}"/*
echo "Done: ${ZIP_FILE}"
