#!/bin/bash

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
exec $ROOT/record_rosbag.sh
exec $ROOT/convert_latest_rosbag.sh