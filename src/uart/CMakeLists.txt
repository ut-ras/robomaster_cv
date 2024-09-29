cmake_minimum_required(VERSION 3.8)
project(uart)
 
# Dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  stampede_msgs
)

# find_package(rosidl_default_generators REQUIRED)

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

include_directories(include)

# Serialib
add_library(serial SHARED
  src/serial/serial.cc
  src/serial/impl/unix.cc
)
target_include_directories(serial
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(serial HAS_LIBRARY_TARGET)
install(
  TARGETS serial
  EXPORT serial
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# uart_tx executable
add_executable(uart_tx 
  src/uart_tx.cpp
)
ament_target_dependencies(uart_tx 
  ${DEPENDENCIES}
)
target_link_libraries(uart_tx
  serial
)

target_include_directories(uart_tx
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# uart_rx executable
add_executable(uart_rx
  src/uart_rx.cpp
)
ament_target_dependencies(uart_rx 
  ${DEPENDENCIES}
)
target_link_libraries(uart_rx
  serial
)

target_include_directories(uart_rx
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# test_pub executable
add_executable(test_pub
  src/test_pub.cpp
)
ament_target_dependencies(test_pub 
  ${DEPENDENCIES}
)

target_include_directories(test_pub
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS
  uart_tx
  uart_rx
  test_pub
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION lib
)

ament_package()