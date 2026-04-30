#!/bin/bash

set -e

cd ..

source .env/bin/activate

PARAM_FILE="$(realpath .sine/sine_beacons_plane_modem.param)"

Tools/autotest/sim_vehicle.py -v ArduPlane \
    -w \
    -j 8 --debug \
    --add-param-file="$PARAM_FILE" \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    --out=udp:127.0.0.1:14552 \
    -l 49.799976,24.354701,250,90 \
    --console \
    -A "--serial1=uart:/dev/serial/by-id/usb-sine.engineering_sine.link_0056003f464d500620333836-if01:57600" \