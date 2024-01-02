cmake_minimum_required(VERSION 3.8)
project(imu_sensor_sdk)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# imu sensor sdk
add_library(imu_sensor_sdk SHARED
  src/mahony_ahrs.cpp
)
target_include_directories(imu_sensor_sdk PUBLIC
  include
)

# Install libraries
install(TARGETS imu_sensor_sdk
  EXPORT imu_sensor_sdk
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


