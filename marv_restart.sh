#!/bin/bash

path=$(pwd)
cd $path

echo "### MARV Package Restart ###"

sleep 1

# Send ctrl + c to all windows
echo "Shutting down ROS nodes"
screen -S ros-marv -X at marv_bash stuff $'\003'
screen -S ros-marv -X at marv_driver stuff $'\003'
screen -S ros-marv -X at marv_scenarios stuff $'\003'
screen -S ros-marv -X at sbg_ins stuff $'\003'
screen -S ros-marv -X at ros_bag stuff $'\003'
screen -S ros-marv -X at specific_ros_bag stuff $'\003'

sleep 7

screen -X -S marv-ros -p 0 quit

# Stop marv package service
sudo systemctl stop marv_package

echo "Package shut down, restarting"

sleep 1

# Start marv package service
sudo systemctl start marv_package

echo "MARV Package restarted, reattaching to newly created screen session in 3s..."
sleep 3
screen -r ros-marv