cmake_minimum_required(VERSION 3.8)
project(detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(realsense2 REQUIRED)

# Add video_publisher executable
add_executable(video_publisher src/video_publisher.cpp)
ament_target_dependencies(video_publisher rclcpp sensor_msgs vision_msgs cv_bridge OpenCV)
target_include_directories(video_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(video_publisher PUBLIC c_std_99 cxx_std_17)

# Add detection_node executable
add_executable(detection_node src/detection_node.cpp)
ament_target_dependencies(detection_node rclcpp sensor_msgs vision_msgs cv_bridge OpenCV realsense2 realsense2_camera_msgs)
# Ensure the realsense2 library is linked correctly
target_link_libraries(detection_node realsense2::realsense2)
target_include_directories(detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(detection_node PUBLIC c_std_99 cxx_std_17)

# Install both executables
install(TARGETS video_publisher detection_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
