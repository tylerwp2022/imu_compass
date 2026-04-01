cmake_minimum_required(VERSION 3.8)
project(imu_compass)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================================================================
# Dependencies
# ==============================================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)   # Imu, NavSatFix, NavSatStatus
find_package(std_msgs REQUIRED)      # Float64, String

# ==============================================================================
# Executable
# ==============================================================================

add_executable(imu_compass_node
  src/imu_compass_node.cpp
)

target_include_directories(imu_compass_node
  PRIVATE
    include
)

ament_target_dependencies(imu_compass_node
  rclcpp
  sensor_msgs
  std_msgs
)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS imu_compass_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
