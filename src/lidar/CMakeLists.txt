cmake_minimum_required(VERSION 3.8)
project(lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(lidar_node src/lidar_node.cpp)
ament_target_dependencies(lidar_node rclcpp sensor_msgs)
target_include_directories(lidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lidar_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(lidar_plane src/lidar_plane.cpp)
ament_target_dependencies(lidar_plane rclcpp sensor_msgs)
target_link_libraries(lidar_plane ${OpenCV_LIBS})
target_include_directories(lidar_plane PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lidar_plane PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS lidar_node lidar_plane
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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
