#!/bin/bash
export PATH=/usr/bin:$PATH
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:=192.168.5.111 port:=1448
