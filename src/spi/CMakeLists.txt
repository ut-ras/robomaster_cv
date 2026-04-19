cmake_minimum_required(VERSION 3.8)
project(spi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(stampede_msgs REQUIRED)

pkg_check_modules(LIBFTDI REQUIRED libftdi1)

add_executable(spi_node src/spi_node.cpp)
add_executable(unpacking_node src/unpacking_node.cpp)
add_executable(packing_node src/packing_node.cpp)

ament_target_dependencies(spi_node rclcpp std_msgs stampede_msgs)
ament_target_dependencies(unpacking_node rclcpp std_msgs stampede_msgs)
ament_target_dependencies(packing_node rclcpp std_msgs stampede_msgs)

target_include_directories(spi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBFTDI_INCLUDE_DIRS}
  
  )

target_link_libraries(spi_node
  ${LIBFTDI_LIBRARIES}
)

target_compile_features(spi_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS spi_node unpacking_node packing_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
