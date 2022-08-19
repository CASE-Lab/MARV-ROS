#!/bin/bash

path=$(pwd)
cd $path

echo "### MARV Package Restart ###"

sleep 1

# Start marv package service
sudo systemctl start marv_package

echo "MARV Package started, reattaching to newly created screen session in 3s..."
sleep 3
screen -r ros-marv