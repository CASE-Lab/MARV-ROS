# MARV-ROS
ROS software for the MARV system. Provides basic functions. Inside colcon_ws the following packages are provided,
- marv_driver: Basic system functions
- marv_msgs: Custom messages package
- marv_scenarios: Contains all the user implemented scenarios, such as waypoint_navigation.
- sbg_ros2: The SBG INS driver

## Dependencies
Install everything mentioned in "reach setup" on the REACH unit.

## Build
Inside colcon_ws folder
- source ROS2 galactic
- build with "colcon build --symlink-install" inside the colcon_ws folder

## Logging
There are two ROS bags for logging data. The first "ros bag" logs every topic, while the second "specific ros bag" only logs the specified topics, such as navigation data. These specific topics can be changed inside the "start_specific_ros_bah.sh" script.

## Running
- Testing purposes: The marv package can be executed using "./marv_start.sh" command, this will start a screen terminal session.
- Normal usage: Make sure that the service has been set up in the "reach setup" document. Then the package should start automatically during boot. If any manual restart is needed then attach to the screen session with "screen -r", shut down all terminals using ctrl + C, ctrl + D until screen has been shut down. Stop the service and then start it again, see commands below.

## The MARV package service commands
- Enable service with "sudo systemctl enable marv_package", used once to enable the service every during boot.
- Disable service with "sudo systemctl enable marv_package", used once to disable the service every time during boot.
- Manually start service with "sudo systemctl start marv_package".
- Manually stop service with "sudo systemctl stop marv_package".
- Check status of service with "systemctl status marv_package".

## Screen Terminal
Link to guide: https://linuxize.com/post/how-to-use-linux-screen/
Reattach: If a session is ongoing it is possible to reattach by executing "screen -r"
Kill all sessions: If multiple sessions are running or there is a need to shut down everything, then execute "killall screen"

## Extra ROS node
The "scenario_starter" ROS node is supposed to simulate the rest of the MARV system's behaviour. It simulates the 12V Auto state and OCU commands start scenario commands. This node can be used for trying out scenarios "on land" before a real test drive. It does however not simulate any sensor data but it will give an indication if the Python script starts up correctly.
