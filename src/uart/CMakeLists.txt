cmake_minimum_required(VERSION 3.8)
project(uart)
 
# Dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
)

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

include_directories(include)

# Serialib
add_library(serialib SHARED
  src/serial/serialib.cpp
)
target_include_directories(serialib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(serialib HAS_LIBRARY_TARGET)
install(
  TARGETS serialib
  EXPORT serialib
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
  serialib
)

target_include_directories(uart_tx
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS
  uart_tx
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION lib
)

ament_package()