#!/bin/bash

# Script to setup can on Xavier NX
# Based on: https://elinux.org/Jetson/AGX_Xavier_CAN
# Noel Danielsson 2021-05-14

# Install CAN Modules
sudo modprobe can
sudo modprobe can-raw
sudo modprobe can-dev
sudo modprobe mttcan

# Set CAN
sudo ifconfig can0 down
sudo ip link set can0 up type can bitrate 1000000

# Verify by trying to send or dump CAN commands:
# $ cansend can0 123#abcdabcd
# $ candump can0
# Socket CAN can be cheked by viewing output from:
# $ ifconfig -a