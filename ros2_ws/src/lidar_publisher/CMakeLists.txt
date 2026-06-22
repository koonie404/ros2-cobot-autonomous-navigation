cmake_minimum_required(VERSION 3.8)
project(lidar_publisher)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lidar_publisher
  lidar_publisher.cpp
  analysis_ex.cpp
)

target_include_directories(lidar_publisher PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_target_dependencies(lidar_publisher
  rclcpp
  sensor_msgs
)

install(TARGETS lidar_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
