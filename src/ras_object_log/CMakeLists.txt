cmake_minimum_required(VERSION 3.8)
project(ras_object_log)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
# find_package(vision_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
enable_language(CUDA)
find_package(CUDA REQUIRED)
# find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(stampede_msgs REQUIRED)
# ament_export_dependencies(vision_msgs geometry_msgs rclcpp stampede_msgs)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(CUDA_NVCC_FLAGS 
  "-arch=sm_87"
)
include_directories(/usr/local/cuda-11.4/targets/aarch64-linux/include/)

# add_library(object-log SHARED
#   src/object-log/ArmorPlate.cu
#   src/object-log/BoundingBox.cu
#   src/object-log/Kalman.cu
#   src/object-log/ObjectLog.cu
# )
# target_compile_options(object-log PRIVATE
#   $<$<COMPILE_LANGUAGE:CUDA>:${CUDA_NVCC_FLAGS}>
# )
# target_compile_definitions(object-log PUBLIC
#   GPU=1
# )
# target_link_libraries(object-log PUBLIC
#   cuda
#   cublas
# )
# set_target_properties(object-log PROPERTIES LINKER_LANGUAGE C)
# target_include_directories(object-log PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
#   $<INSTALL_INTERFACE:include>
# )
# target_compile_features(object-log PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_export_targets(object-log HAS_LIBRARY_TARGET)
# install(
#   TARGETS object-log
#   EXPORT object-log
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

add_executable(object_log 
  src/object_log.cpp
)
ament_target_dependencies(object_log 
  rclcpp 
  stampede_msgs
)
target_link_libraries( object_log
${catkin_LIBRARIES}
# cuda
${CMAKE_SOURCE_DIR}/src/object-log/libobjectlog.so
)

target_include_directories(object_log 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(object_log PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS object_log
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
