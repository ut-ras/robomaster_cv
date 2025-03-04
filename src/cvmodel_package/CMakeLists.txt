cmake_minimum_required(VERSION 3.8)
project(cvmodel_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add video_publisher executable
add_executable(video_publisher src/video_publisher.cpp)
ament_target_dependencies(video_publisher rclcpp sensor_msgs cv_bridge OpenCV)
target_include_directories(video_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(video_publisher PUBLIC c_std_99 cxx_std_17)

# Add cvmodel_node executable
add_executable(cvmodel_node src/cvmodel_node.cpp)
ament_target_dependencies(cvmodel_node rclcpp sensor_msgs cv_bridge OpenCV)
target_include_directories(cvmodel_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cvmodel_node PUBLIC c_std_99 cxx_std_17)

# Install both executables
install(TARGETS video_publisher cvmodel_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
