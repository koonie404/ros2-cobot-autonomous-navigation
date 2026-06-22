#!/bin/bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

rviz2 -d /root/ros2_ws/src/co_bot_2_description/rviz/slam.rviz
