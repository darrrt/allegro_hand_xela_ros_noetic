#! /use/bin/bash
# used for the vscan device connect to allegro
# sudo modprobe can
# sudo modprobe can-raw
# sudo modprobe slcan


sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB*
sudo ip link set up slcan0