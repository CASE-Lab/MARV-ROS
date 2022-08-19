#!/bin/bash

path=$(pwd)
cd $path

echo "### MARV Package Stop ###"

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

echo "MARV Package stopped"