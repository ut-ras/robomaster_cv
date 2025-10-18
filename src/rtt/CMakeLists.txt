cmake_minimum_required(VERSION 3.8)
project(rtt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stampede_msgs REQUIRED)

# Build the RTT node executable from src/rtt.cpp
add_executable(rtt_node src/rtt.cpp)

target_include_directories(rtt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(rtt_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(rtt_node rclcpp std_msgs)

install(TARGETS rtt_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

