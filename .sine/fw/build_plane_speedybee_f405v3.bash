#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$REPO_DIR"

export PATH="$HOME/apps/toolchains/xpack/xpack-arm-none-eabi-gcc-13.3.1-1.1/bin:$PATH"

source .env/bin/activate

./waf configure \
    --board speedybeef4v3 \
    --extra-hwdef="$SCRIPT_DIR/sine_extra.dat"

./waf plane
