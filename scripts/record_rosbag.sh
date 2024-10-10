#!/bin/bash
NOW=$( date '+%Y_%m_%d-%H_%M_%S' )
ros2 bag record -e /robot --start-paused -o "/robomaster_cv/rosbags/rosbag2_$NOW"