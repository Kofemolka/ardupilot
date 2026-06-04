#!/bin/bash

set -e

SRC_FILE="$(realpath $1)"

cd ..

source .env/bin/activate

./waf configure --board sitl --out=build/replay
./waf replay

./build/replay/sitl/tool/Replay ${SRC_FILE}