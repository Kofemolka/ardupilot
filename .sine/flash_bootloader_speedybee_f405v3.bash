#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

BOOTLOADER="$REPO_DIR/build/speedybeef4v3/bin/AP_Bootloader.bin"
# fallback path used by some waf versions
[ -f "$BOOTLOADER" ] || BOOTLOADER="$REPO_DIR/build/speedybeef4v3/AP_Bootloader.bin"

if [ ! -f "$BOOTLOADER" ]; then
    echo "Bootloader not found: $BOOTLOADER"
    echo "Run build_bootloader_speedybee_f405v3.bash first."
    exit 1
fi

echo "Put the board in DFU mode (hold BOOT button, then plug USB)."
echo "Press Enter when ready..."
read -r

dfu-util -a 0 -s 0x08000000:force:leave -D "$BOOTLOADER"
