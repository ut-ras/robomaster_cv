cmake_minimum_required(VERSION 3.8)
project(dji_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stampede_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Build a library for the DJI Serial protocol functionality.
add_library(${PROJECT_NAME}_lib
  src/dji_serial.cpp
  src/crc.cpp
)

add_executable(dji_serial
  src/dji_serial.cpp
  src/crc.cpp
)

add_executable(test_serial
  src/test_serial.cpp
)

add_executable(pose_node
  src/pose_node.cpp
)

add_executable(color_node
  src/color_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib rclcpp std_msgs stampede_msgs)
ament_target_dependencies(dji_serial rclcpp std_msgs stampede_msgs)
ament_target_dependencies(test_serial rclcpp std_msgs stampede_msgs)
ament_target_dependencies(pose_node rclcpp std_msgs stampede_msgs tf2)
ament_target_dependencies(color_node rclcpp std_msgs stampede_msgs tf2)

install(TARGETS ${PROJECT_NAME}_lib dji_serial test_serial pose_node color_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip cpplint for now (adjust if needed)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
