#!/bin/bash
#
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source /usr/local/bin/scripts/setup.sh" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash
source /usr/local/bin/scripts/setup.sh

sudo udevadm control --reload-rules

# Configure git
git config --global --add safe.directory /robomaster_cv

sudo apt-get update
rosdep update

# Restart udev daemon
sudo service udev restart

$@