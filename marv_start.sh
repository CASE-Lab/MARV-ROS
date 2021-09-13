#!/bin/bash

# "sudo systemctl disable nvgetty.service" + reboot is needed in order to access /dev/ttyTHS0

# Change this to the marv repo folder location
path="/home/reach-002/Documents"
#path="/home/viktor/Documents"

echo "MARV System Initializing..."

sleep 1

# Initialize CAN
echo "Starting CAN services"
$path/MARV-ROS/start_scripts/canconfig.sh
sleep 1

# setting up named screen windows
screen -AdmS ros-marv -t marv-bash bash
screen -S ros-marv -X screen -t ros_bag bash
screen -S ros-marv -X screen -t specific_ros_bag bash
screen -S ros-marv -X screen -t sbg_ins bash
screen -S ros-marv -X screen -t marv_driver bash
screen -S ros-marv -X screen -t marv_scenarios bash

# Set up extra terminal
screen -S ros-marv -p marv-bash -X stuff "source /opt/ros/eloquent/setup.bash
source $path/MARV-ROS/colcon_ws/install/setup.bash
"

# Start all packages
echo "Starting MARV Driver"
screen -S ros-marv -p marv_driver -X stuff "source /opt/ros/eloquent/setup.bash
source $path/MARV-ROS/colcon_ws/install/setup.bash
^l
ros2 launch marv_driver marv_driver.launch.py
"
sleep 1
echo "Starting MARV Scenarios"
screen -S ros-marv -p marv_scenarios -X stuff "source /opt/ros/eloquent/setup.bash
source $path/MARV-ROS/colcon_ws/install/setup.bash
^l
ros2 launch marv_scenarios marv_scenarios.launch.py
"
sleep 1
echo "Starting SBG INS Driver"
screen -S ros-marv -p sbg_ins -X stuff "source /opt/ros/eloquent/setup.bash
source $path/MARV-ROS/colcon_ws/install/setup.bash
^l
ros2 launch sbg_driver sbg_device_launch.py
"
sleep 1
echo "Starting ROS Bag"
screen -S ros-marv -p ros_bag -X stuff "source /opt/ros/eloquent/setup.bash
source $path/MARV-ROS/colcon_ws/install/setup.bash
^l
./start_scripts/start_ros_bag.sh
"
echo "Starting Specific ROS Bag"
screen -S ros-marv -p specific_ros_bag -X stuff "source /opt/ros/eloquent/setup.bash
source $path/MARV-ROS/colcon_ws/install/setup.bash
^l
./start_scripts/start_specific_ros_bag.sh
"

echo "Reattaching to newly created screen session in 5s..."
sleep 5
screen -r ros-marv
