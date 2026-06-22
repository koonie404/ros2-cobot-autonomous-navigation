#!/bin/bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial0 -b 115200
