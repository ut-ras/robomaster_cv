cmake_minimum_required(VERSION 3.8)
project(target_selector_pkg)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(visualization_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)  # Ensure OpenCV is found
find_package(vision_msgs REQUIRED)

# Declare Executables
add_executable(target_selector_node target_selector_node.cpp)



# Link Dependencies for target_selector_node
ament_target_dependencies(target_selector_node 
  rclcpp 
  std_msgs 
  vision_msgs
  visualization_msgs
)

# OpenCV linking
include_directories(${OpenCV_INCLUDE_DIRS})

# Compiler Features
target_compile_features(target_selector_node PUBLIC c_std_99 cxx_std_17)

# Install Executables


install(TARGETS target_selector_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
