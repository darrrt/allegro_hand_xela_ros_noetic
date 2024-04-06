#! /use/bin/bash
# used for the vscan device connect to allegro
# sudo modprobe can
# sudo modprobe can-raw
# sudo modprobe slcan

ls /dev/ttyUSB*
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB*
sudo ip link set up slcan0

# roslaunch allegro_hand_controllers allegro_viz.launch HAND:=right
roslaunch allegro_hand_controllers allegro_hand.launch HAND:=right CONTROLLER:=pd