#!/bin/bash

path=$(pwd)
cd $path

echo "MARV RNS Starting..."

sleep 1

# Source ROS
source /opt/ros/galactic/setup.bash

# Change location to server location
cd ./WEB-UI/dist

# Start node server.js
node server.js