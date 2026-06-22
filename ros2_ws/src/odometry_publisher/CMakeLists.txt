cmake_minimum_required(VERSION 3.8)

project(odometry_publisher)

# =========================================
# C++17
# =========================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =========================================
# Node Name
# =========================================

set(NODE_NAME _42_odometry_publisher)

# =========================================
# Find Packages
# =========================================

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(nav_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(tf2_ros REQUIRED)

find_package(Threads REQUIRED)

# =========================================
# Executable
# =========================================

add_executable(${NODE_NAME}
    ${NODE_NAME}.cpp
)

# =========================================
# ROS2 Dependencies
# =========================================

ament_target_dependencies(${NODE_NAME}

    rclcpp

    std_msgs

    sensor_msgs

    nav_msgs

    geometry_msgs

    tf2_ros
)

# =========================================
# pthread
# =========================================

target_link_libraries(${NODE_NAME}

    Threads::Threads
)

# =========================================
# Install (Optional)
# =========================================

install(TARGETS

    ${NODE_NAME}

    DESTINATION lib/${PROJECT_NAME}
)

ament_package()