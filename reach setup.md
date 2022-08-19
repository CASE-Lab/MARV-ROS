#### REACH SETUP for Ubuntu 20.04 with ROS2 Galactic ####

## Initial setup
Flash memory card with jetson-os (can be done using Nvidia SDK Manager)
Format hard-drive (SSD)
Enable ssh (https://www.cyberciti.biz/faq/ubuntu-linux-install-openssh-server/)
Move installation to SSD Jetsonhacks (https://www.jetsonhacks.com/2020/05/29/jetson-xavier-nx-run-from-ssd/)
Shut down ssh over telemetri, Jetsonhacks (https://www.jetsonhacks.com/2019/10/10/jetson-nano-uart/)

## Install dependencies
Ros2 (https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
Pip3 (https://linuxize.com/post/how-to-install-pip-on-ubuntu-20.04/)
Pyserial (pip3 install pyserial)
Colcon (https://colcon.readthedocs.io/en/released/user/installation.html)
Python-can (pip3 install python-can)
Cantools (pip3 install cantools)
Screen terminal (sudo apt install screen)

## Add repository, change admin privileges and build ros package
Clone the marv-ros repository to the documents folder
Run "sudo visudo" and remove sudo requirement, add a line with "reach-001 ALL=(ALL) NOPASSWD:ALL"
Add user to dialout usb (run "sudo usermod -a -G dialout $USER")
Build marv-ros (colcon build --symlink-install) inside the colcon_ws

## Setup marv package service (enabling autostart of the whole marv package)
Edit marv_package to the correct user, e.g reach-001
Copy marv_package.service with "sudo cp -a marv_package.service /etc/systemd/system"
Enable service with "sudo systemctl enable marv_package"
(Start service directly, without reboot, to see if everything starts up with "sudo systemctl start marv_package")

