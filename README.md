# UT RoboMaster CV

## Pre-requisites
You will need **Ubuntu 22.04**. If you do not want to dual-boot (or switch to Ubuntu entirely), WSL2 would be a good option.

To install Ubuntu 22.04 on WSL2, run the following in a Windows Terminal:
```cmd
wsl --install -d Ubuntu-22.04
```

To access WSL2 through VSCode, download the "WSL" extension.

## Setup
1. Install ROS2 Humble by following [these instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
2. Add the `setup.bash` file to your shell startup script by running the following command in a terminal:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```
3. Install `colcon` using the following command in a terminal:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```
4. Install the RealSense SDK following steps 2 and 3 [outlined here](https://github.com/IntelRealSense/realsense-ros).
   1. Make sure to get the "optional" developer and debug packages

5. Clone this repository to a location of your choice using
   ```bash
   git clone --recurse-submodules https://github.com/ut-ras/robomaster_cv.git
   ```
6. Build the ROS2 workspace using
   ```bash
   colcon build
   ```
7. Install the ros RealSense camera package
   ```bash
      sudo apt-get install ros-foxy-realsense2-camera
   ```
8. Start the RealSense camera node using
   ```bash
   ros2 run realsense2_camera realsense2_camera_node
   ```
