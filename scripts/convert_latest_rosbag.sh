#!/bin/bash
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
names=( $ROOT/../rosbags/* )
name=${names[-1]}
base_name=$(basename ${name})
python3 $ROOT/rosbag2mp4.py -t /robot/rs2/color/image_raw/compressed -o "$name/$base_name.mp4" ${name}