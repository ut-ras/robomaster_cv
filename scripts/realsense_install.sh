#!/bin/bash

sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

sudo apt-get install apt-transport-https

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

sudo apt-get update
sudo apt-get upgrade

sudo apt install ros-humble-realsense2-*

sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Eloquent and earlier
rosdep update # "sudo rosdep update --include-eol-distros" for Eloquent and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

sudo apt-get install ros-humble-realsense2-camera


#Tests
colcon build
relasense-viewer
ros2 run realsense2_camera realsense2_camera_node