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
- Normal usage: Make sure that the service has been set up in the "reach setup" document. Then the package should start automatically during boot. If any manual action is needed then the following scripts can be executed, "./marv_restart", "./marv_stop", "./marv_start". If in screen session, deattach by "ctrl + a", then press "d", after that run command.
- The package could also be shut down by attaching to the screen session with "screen -r", shut down all terminals using ctrl + C, ctrl + D until screen has been shut down.

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


# MARV-RNS Addition
The MARV RNS adds a Remote Network Steering scenario for the MARV ROS package. This package allows for remote control and a usb camera feed over wifi, 4G, LTE or 5G. 

The operator controls the MARV using an XBOX controller and a web interface served by the web server. 

## Dependencies
Before you can run the server you need to install Node.js, preferrably version 16 since the server was only developed and tested with this version. Node version manager (NVM) works well for installing a specific Node version on the REACH computer.
Then you need to install all npm dependencies for the server. This can be done by sourcing ROS2 ("source /opt/ros/galactic/setup.bash") and running "npm install" inside the "WEB-UI" folder. If this does not work, try "npm i rclnodejs" and then "npm install".

## Running RNS
### MARV ROS package
Start the MARV ROS package as described above in "Running".

### Web server and ROS node (in one package)
Launch the server by,
- Source ROS2 Galactic ("source /opt/ros/galactic/setup.bash")
- Locate the server folder ("cd Documents/MARV-ROS-WITH-RNS/WEB-UI/dist")
- Start the server using "node server.js". The ROS node is baked into the server.js script and will start simultaneously with the server. Might want to do this differently in the future, for example with the PM2 daemon.

On host computer (REACH) - for video stream:
- Open a web brower (preferrably Chrome based) and go to the IP address localhost:1337/broadcst.html to auto detect the usb camera and start brodcasting the video stream

On client computer:
- Open a web browser (preferrably Chrome based) and go to the IP address of the REACH computer (192.168.1.91 in the CASE lab) and add port 1337 to the end, like this "192.168.1.91:1337"
- Press "Enable Gamepad" and you are ready to steer the MARV

## Gamepad controls
- Throttle ...