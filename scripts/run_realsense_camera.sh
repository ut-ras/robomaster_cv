ros2 launch launch/realsense_launch.py rgb_camera.profile:=1280x720x30 depth_module.profile:=1280x720x30 rgb_camera.enable_auto_exposure:=false rgb_camera.exposure:=20 enable_sync:=true align_depth.enable:=true enable_rgbd:=true
# ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=1280x720x30 depth_module.profile:=1280x720x30 enable_color:=true enable_depth:=true enable_sync:=true camera_namespace:=robot camera_name:=rs2
# ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=1280x720x30 depth_module.profile:=1280x720x30 enable_color:=true enable_depth:=true enable_sync:=true camera_namespace:=robot camera_name:=rs2 align_depth.enable:=true enable_rgbd:=true
# ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=424x240x60 depth_module.profile:=424x240x60 enable_color:=true enable_depth:=true enable_sync:=true camera_namespace:=robot camera_name:=rs2 align_depth.enable:=true enable_rgbd:=true color_qos:=SERVICES_DEFAULT depth_qos:=SENSOR_DEFAULT
# ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=1280x720x30 depth_module.profile:=1280x720x30 enable_color:=true enable_depth:=true enable_sync:=true camera_namespace:=robot camera_name:=rs2 align_depth.enable:=true enable_rgbd:=true color_qos:=SERVICES_DEFAULT depth_qos:=SENSOR_DEFAULT
# ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=1280x720x30 depth_module.profile:=1280x720x30 enable_color:=true enable_depth:=true enable_sync:=true camera_namespace:=robot camera_name:=rs2 align_depth.enable:=true enable_rgbd:=true rgb_camera.exposure:=15
# ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=1280x720x30 depth_module.profile:=1280x720x6 enable_color:=true enable_depth:=true camera_namespace:=robot camera_name:=rs2
# ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=1280x720x30 depth_module.profile:=1280x720x30 enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true camera_namespace:=robot camera_name:=rs2
# ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=960x540x60 depth_module.profile:=1280x720x30 enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true camera_namespace:=robot camera_name:=rs2
# ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true camera_namespace:=robot camera_name:=rs2
# ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true camera_namespace:=robot camera_name:=rs2


## RGB Camera Profiles:
# 1280x720x15
# 1280x720x30
# 1280x720x6
# 1920x1080x15
# 1920x1080x30
# 1920x1080x6
# 320x180x30
# 320x180x6
# 320x180x60
# 320x240x30
# 320x240x6
# 320x240x60
# 424x240x15
# 424x240x30
# 424x240x6
# 424x240x60
# 640x360x15
# 640x360x30
# 640x360x6
# 640x360x60
# 640x480x15
# 640x480x30
# 640x480x6
# 640x480x60
# 848x480x15
# 848x480x30
# 848x480x6
# 848x480x60
# 960x540x15
# 960x540x30
# 960x540x6
# 960x540x60
#
## Depth Camera Profiles:
# 1280x720x15
# 1280x720x30
# 1280x720x6
# 1280x800x15
# 1280x800x25
# 1280x800x30
# 256x144x300
# 256x144x90
# 424x240x15
# 424x240x30
# 424x240x6
# 424x240x60
# 424x240x90
# 480x270x15
# 480x270x30
# 480x270x6
# 480x270x60
# 480x270x90
# 640x360x15
# 640x360x30
# 640x360x6
# 640x360x60
# 640x360x90
# 640x400x15
# 640x400x25
# 640x480x15
# 640x480x30
# 640x480x6
# 640x480x60
# 640x480x90
# 848x100x100
# 848x100x300
# 848x480x15
# 848x480x30
# 848x480x6
# 848x480x60
# 848x480x90
