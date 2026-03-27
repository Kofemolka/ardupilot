#!/bin/bash

set -e

source .env/bin/activate

Tools/autotest/sim_vehicle.py -v ArduCopter \
    -w \
    -j 8 --debug \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    -A "--serial1=uart:/dev/serial/by-id/usb-sine.engineering_sine.link_0056003f464d500620333836-if03:57600" \
    -l 49.799976,24.354701,250,90 \
    --console