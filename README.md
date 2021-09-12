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
- source ROS2 eloquent
- build with "colcon build --symlink-install"

## Logging
There are two ROS bags for logging data. The first one tries to log everything, which does not always work. The second "specific ros bag" only logs the specified topics. These should be entered inside the "start_specific_ros_bah.sh" script.

## Run
Before running change the "path" variable inside "marv_start.sh" to the location of the repository.
Then simply execute the script to start a screen teminal session.

## Screen Terminal
Link to guide: https://linuxize.com/post/how-to-use-linux-screen/
Reattach: If a session is ongoing it is possible to reattach by executing "screen -r"
Kill all sessions: If multiple sessions are running or there is a need to shut down everything, then execute "killall screen"
