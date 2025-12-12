#!/bin/bash
export PATH=/usr/bin:$PATH
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "123" | sudo -S chmod 777 /dev/ttyUSB0
ros2 run grasp_topic xinhai_connect.py
