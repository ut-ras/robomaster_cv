cmake_minimum_required(VERSION 3.8)
project(classical)
set(CMAKE_CXX_STANDARD 17)
 
# Dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  realsense2_camera_msgs
  OpenCV
  cv_bridge
  stampede_msgs
)

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

include_directories(include)

# classical executable
add_executable(classical 
  src/classical.cpp
)
ament_target_dependencies(classical 
  ${DEPENDENCIES}
)

target_include_directories(classical
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS
  classical
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION lib
)

ament_package()