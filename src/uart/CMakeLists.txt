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
add_executable(uart
  src/uart.cpp
)

add_executable(simulate_uart_rx
  src/simulate_uart_rx.cpp
)

ament_target_dependencies(uart
  ${DEPENDENCIES}
)

ament_target_dependencies(simulate_uart_rx
  ${DEPENDENCIES}
)

target_link_libraries(uart
  serial
)

target_include_directories(uart
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS
  uart
  simulate_uart_rx
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION lib
)

ament_package()