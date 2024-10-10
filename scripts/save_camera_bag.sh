#!/bin/bash
NOW=$( date '+%Y%m%d_%H%M%S' )
ros2 bag record -e /robot --start-paused -o "$NOW-rosbag"