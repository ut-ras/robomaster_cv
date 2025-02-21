cmake_minimum_required(VERSION 3.8)
project(isaac_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)  # Moved this above to avoid errors

# Create the executable only ONCE
add_executable(isaac_node src/isaac_node.cpp)

# Set include directories
target_include_directories(isaac_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dependencies
ament_target_dependencies(isaac_node rclcpp sensor_msgs)

# Install the executable
install(TARGETS isaac_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()