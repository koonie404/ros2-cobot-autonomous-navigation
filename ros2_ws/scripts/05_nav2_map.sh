#!/bin/bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 launch nav2_bringup bringup_launch.py \
  slam:=False \
  map:=/root/ros2_ws/maps/map.yaml \
  params_file:=/root/ros2_ws/install/co_bot_2_description/share/co_bot_2_description/config/navigation.yaml
