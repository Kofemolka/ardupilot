#!/bin/bash

set -e

cd ..

source .env/bin/activate

rm -rf scripts/*
mkdir -p scripts/modules

cp .sine/sdk/modules/*.lua scripts/modules/
cp .sine/sdk/pos_inject.lua scripts/pos_inject.lua
# cp .sine/sdk/broadcast_demo.lua scripts/broadcast_demo.lua
# cp .sine/sdk/sms_demo.lua scripts/sms_demo.lua

PARAM_FILE="$(realpath .sine/sdk/sdk.param)"

Tools/autotest/sim_vehicle.py -v ArduPlane \
    -w \
    -j 8 --debug \
    --add-param-file="$PARAM_FILE" \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    -l 49.799976,24.354701,250,90 \
    --console \
    -A "--serial1=uart:/dev/serial/by-id/usb-sine.engineering_sine.link_0056003f464d500620333836-if01:57600"

# home slave
#   usb-sine.engineering_sine.link_004100534e4d500420343332-if01
# work slave
#   usb-sine.engineering_sine.link_0056003f464d500620333836-if01