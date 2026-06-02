#!/bin/bash

set -e

SRC_FILE="$(realpath $1)"

cd ..

source .env/bin/activate

./waf configure --board sitl
./waf replay

./build/sitl/tool/Replay ${SRC_FILE}