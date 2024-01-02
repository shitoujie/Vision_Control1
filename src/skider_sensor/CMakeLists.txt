cmake_minimum_required(VERSION 3.8)
project(skider_sensor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(sdk/imu)
add_subdirectory(sdk/remote)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(skider_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# NODE 1 : imu_sensor_node
add_executable(imu_sensor_node
  src/imu_sensor_node.cpp
)
target_link_libraries(imu_sensor_node
  imu_sensor_sdk
)
target_include_directories(imu_sensor_node PUBLIC
  include/skider_sensor
)
ament_target_dependencies(imu_sensor_node
  rclcpp
  rclcpp_components
  skider_interface
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
)

# NODE 2 : remote_sensor_node
add_executable(remote_sensor_node
  src/remote_sensor_node.cpp
)
target_link_libraries(remote_sensor_node
  remote_sensor_sdk
)
target_include_directories(remote_sensor_node PUBLIC
  include/skider_sensor
)
ament_target_dependencies(remote_sensor_node
  rclcpp
  rclcpp_components
  skider_interface
  sensor_msgs

)

# Install executable nodes
install(TARGETS imu_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS remote_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
