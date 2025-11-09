cmake_minimum_required(VERSION 3.8)
project(autonav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(autonav_node src/autonav_node.cpp)
ament_target_dependencies(autonav_node rclcpp std_msgs vision_msgs OpenCV)
target_include_directories(autonav_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(autonav_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(fake_robot_publisher src/fake_robot_publisher.cpp)
ament_target_dependencies(fake_robot_publisher rclcpp std_msgs vision_msgs)
target_include_directories(fake_robot_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fake_robot_publisher PUBLIC c_std_99 cxx_std_17)

install(TARGETS autonav_node fake_robot_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
