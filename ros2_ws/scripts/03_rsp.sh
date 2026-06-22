#!/bin/bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 launch co_bot_2_description rsp.launch.py
