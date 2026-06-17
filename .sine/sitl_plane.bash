#!/bin/bash

set -e

cd ..

source .env/bin/activate

Tools/autotest/sim_vehicle.py -v ArduPlane \
    -w \
    -j 8 --debug \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    -l 49.799976,24.354701,250,90 \
    --console \
    -A "--serial1=uart:/dev/serial/by-id/usb-sine.engineering_sine.link_004100534e4d500420343332-if01:57600"
