cmake_minimum_required(VERSION 3.8)
project(isaac_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Build your executable from your cpp file
add_executable(isaac_node src/isaac_node.cpp)

# Include directories (this is standard)
target_include_directories(isaac_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dependencies (ament_target_dependencies is preferred over target_link_libraries)
ament_target_dependencies(isaac_node rclcpp)

# Install the executable
install(TARGETS isaac_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

