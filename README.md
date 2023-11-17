# UT RoboMaster CV

## Pre-requisites

If you would like to install the toolchain locally, you will need **Ubuntu 22.04**. If you do not want to dual-boot (or switch to Ubuntu entirely), WSL2 would be a good option. Additionally, there is a Dockerfile that can be used to run the toolchain, but it might not be able to flash to the board.

To install Ubuntu 22.04 on WSL2, run the following in a Windows Terminal:
```cmd
wsl --install -d Ubuntu-22.04
```

To access WSL2 through VSCode, download the "WSL" extension.

## Setup
1. Clone this repository to a location of your choice*(using the WSL Terminal) with the following command:
   ```bash
   git clone --recurse-submodules https://github.com/ut-ras/robomaster_cv.git
   ```
2. Install ROS2 Humble by following [these instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
3. Add the `setup.bash` file to your shell startup script by running the following command in a terminal:

   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```
4. Install `colcon` using the following command in a terminal:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```
5. Install the RealSense SDK following steps 2 and 3 [outlined here](https://github.com/IntelRealSense/realsense-ros).
   1. Make sure to get the "optional" developer and debug packages
7. Build the ROS2 workspace using
   ```bash
   colcon build
   ```
8. Install the ros RealSense camera package
   ```bash
   sudo apt-get install ros-humble-realsense2-camera
   ```
9. Start the RealSense camera node using
   ```bash
   ros2 run realsense2_camera realsense2_camera_node
   ```

## Docker Setup

First, ensure that you have Docker installed and running. If not, you can download it (here)[https://www.docker.com/]. You may need to also install Docker Compose if you are on linux. Instructions for installing the compose plugin should already be included with the default Docker install.

Then, run the startup script in a terminal window.

`./start.sh`

After it finishes building, you should see a new terminal pop up that gives access to the Docker container. This terminal forwards the local directory to the directory `/robomaster_cv`. From here, you can build and run the system as normal using `colcon build`. When you need to exit, simply run `exit` or press `Ctrl+D`.

### Mac Docker X11 Setup

1. Install XQuartz

2. Start XQuartz by opening the application (Cmd + Space -> Search for XQuartz)

3. Open XQuartz settings and navigate to the security tab

4. Check "Allow connections from network clients"

5. Quit XQuartz (make sure it is completely closed and gone from your taskbar)

6. Reopen XQuartz

7. In a local terminal, run `xhost +localhost`

8. Run `./start.sh` to open the Docker container

9. On every window you want to use X11 on, run `export DISPLAY=docker.for.mac.host.internal:0`


#### References

https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088