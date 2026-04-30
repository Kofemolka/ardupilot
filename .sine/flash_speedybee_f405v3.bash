#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

FIRMWARE="$REPO_DIR/build/speedybeef4v3/bin/arduplane.apj"

if [ ! -f "$FIRMWARE" ]; then
    echo "Firmware not found: $FIRMWARE"
    echo "Run build_plane_speedybee_f405v3.bash first."
    exit 1
fi

source "$REPO_DIR/.env/bin/activate"

python "$REPO_DIR/Tools/scripts/uploader.py" \
    --port /dev/ttyACM0 \
    "$FIRMWARE"
