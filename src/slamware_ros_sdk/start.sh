#!/bin/bash

# SLAMWARE ROS2 启动脚本（简化版）

export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
source /home/yofo/ros2_ws_demo/install/setup.bash  # 绝对路径（核心修改）
export ROS_DOMAIN_ID=0

echo "启动 SLAMWARE ROS SDK 服务器节点..."
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.xml ip_address:=192.168.5.111 port:=1445