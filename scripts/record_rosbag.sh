#!/bin/bash
NOW=$( date '+%Y_%m_%d-%H_%M_%S' )
# ros2 bag record --start-paused \
#     --compression-format zstd \
#     --compression-mode file \
#     -a \
#     -o "/robomaster_cv/rosbags/rosbag2_$NOW" \
#     -e /robot/rs2 \
#     -x "(.*compressed)|(.*compressedDepth)|(.*theora)" \
#     --qos-profile-overrides-path ./override.yaml

# ros2 bag record --start-paused \
#     --compression-format zstd \
#     --compression-mode file \
#     -o "/robomaster_cv/rosbags/rosbag2_$NOW" \
#     --qos-profile-overrides-path ./override.yaml \
#     /robot/rs2/color/image_raw

# ros2 bag record --start-paused \
#     --compression-format zstd \
#     --compression-mode file \
#     -o "/robomaster_cv/rosbags/rosbag2_$NOW" \
#     -e /robot/rs2 

# ros2 bag record --start-paused \
#     -o "/robomaster_cv/rosbags/rosbag2_$NOW" \
#     --compression-format zstd \
#     --compression-mode file \
#     /robot/rs2/aligned_depth_to_color/camera_info \
#     /robot/rs2/aligned_depth_to_color/image_raw \
#     /robot/rs2/color/camera_info \
#     /robot/rs2/color/image_raw/compressed \
#     /robot/rs2/color/metadata \
#     /robot/rs2/depth/camera_info \
#     /robot/rs2/depth/image_rect_raw \
#     /robot/rs2/extrinsics/depth_to_color \
#     /robot/rs2/rgbd

# ros2 bag record --start-paused \
#     -o "/robomaster_cv/rosbags/rosbag2_$NOW" \
#     --compression-format zstd \
#     --compression-mode file \
#     /robot/rs2/color/camera_info \
#     /robot/rs2/color/image_raw/compressed \
#     /robot/rs2/color/metadata \
#     /robot/rs2/depth/camera_info \
#     /robot/rs2/depth/image_rect_raw \
#     /robot/rs2/extrinsics/depth_to_color \
#     /robot/rs2/align_depth_to_color/camera_info \
#     /robot/rs2/rgbd

ros2 bag record --start-paused \
    -o "/robomaster_cv/rosbags/rosbag2_$NOW" \
    --compression-format zstd \
    --compression-mode file \
    /robot/rs2/color/camera_info \
    /robot/rs2/color/image_raw/compressed \
    /robot/rs2/color/metadata \
    /robot/rs2/depth/camera_info \
    /robot/rs2/extrinsics/depth_to_color \
    /robot/rs2/align_depth_to_color/camera_info \
    /robot/rs2/rgbd

ros2 param dump /robot/rs2 > "/robomaster_cv/rosbags/rosbag2_$NOW/robot__rs2.yaml"

# ros2 bag record --start-paused \
#     -o "/robomaster_cv/rosbags/rosbag2_$NOW" \
#     --compression-format zstd \
#     --compression-mode file \
#     /robot/rs2/color/image_raw/compressed



# /parameter_events [rcl_interfaces/msg/ParameterEvent]
# /robot/rs2/aligned_depth_to_color/camera_info [sensor_msgs/msg/CameraInfo]
# /robot/rs2/aligned_depth_to_color/image_raw [sensor_msgs/msg/Image]
# /robot/rs2/aligned_depth_to_color/image_raw/compressed [sensor_msgs/msg/CompressedImage]
# /robot/rs2/aligned_depth_to_color/image_raw/compressedDepth [sensor_msgs/msg/CompressedImage]
# /robot/rs2/aligned_depth_to_color/image_raw/theora [theora_image_transport/msg/Packet]
# /robot/rs2/color/camera_info [sensor_msgs/msg/CameraInfo]
# /robot/rs2/color/image_raw [sensor_msgs/msg/Image]
# /robot/rs2/color/image_raw/compressed [sensor_msgs/msg/CompressedImage]
# /robot/rs2/color/image_raw/compressedDepth [sensor_msgs/msg/CompressedImage]
# /robot/rs2/color/image_raw/theora [theora_image_transport/msg/Packet]
# /robot/rs2/color/metadata [realsense2_camera_msgs/msg/Metadata]
# /robot/rs2/depth/camera_info [sensor_msgs/msg/CameraInfo]
# /robot/rs2/depth/image_rect_raw [sensor_msgs/msg/Image]
# /robot/rs2/depth/image_rect_raw/compressed [sensor_msgs/msg/CompressedImage]
# /robot/rs2/depth/image_rect_raw/compressedDepth [sensor_msgs/msg/CompressedImage]
# /robot/rs2/depth/image_rect_raw/theora [theora_image_transport/msg/Packet]
# /robot/rs2/depth/metadata [realsense2_camera_msgs/msg/Metadata]
# /robot/rs2/extrinsics/depth_to_color [realsense2_camera_msgs/msg/Extrinsics]
# /robot/rs2/rgbd [realsense2_camera_msgs/msg/RGBD]
# /rosout [rcl_interfaces/msg/Log]
# /tf_static [tf2_msgs/msg/TFMessage]
