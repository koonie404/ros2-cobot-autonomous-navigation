#!/bin/bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
