cmake_minimum_required(VERSION 3.8)
project(remote_sensor_sdk)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# imu sensor sdk
add_library(remote_sensor_sdk SHARED
  src/sbus_remote.cpp
)
target_include_directories(remote_sensor_sdk PUBLIC
  include
)

# Install libraries
install(TARGETS remote_sensor_sdk
  EXPORT remote_sensor_sdk
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
