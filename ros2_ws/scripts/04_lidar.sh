#!/bin/bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 run lidar_publisher lidar_publisher
