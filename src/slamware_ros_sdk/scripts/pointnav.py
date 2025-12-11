#!/usr/bin/env python3

import sys
print("\n===== ROS 2 Python path =====")
print(f"sys.executable: {sys.executable}")
print(f"version: {sys.version.split()[0]}")
print("=============================\n")

import rclpy
from rclpy.node import Node
# import cv2
# import numpy as np
import os
script_dir = os.path.dirname(os.path.realpath(__file__))
os.chdir(script_dir)

# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
# from grasp_topic.msg import Camera, Pose 
# import pyrealsense2 as rs
# from std_msgs.msg import String, Bool
# from collections import deque
# import time

sys.path.append('/home/yofo/DucoCobotAPI') 
from DucoCobotAPI_py.SlamwareInterface_ros2 import SlamwareInterface

class PointNav(Node):
     # 构造函数：各种硬件和ros节点的初始化
    def __init__(self):
        super().__init__('pointnav')
        self.mobile_base = SlamwareInterface()
    def nav(self):
        pose=Pose(position=Point(x=4.05, y=0.31, z=-0.39),orientation=Quaternion(x=0.000000, y=0.000000, z=0.732611, w=0.680648))
        self.mobile_base.nav_to_pose(pose)

def main(args=None):
    rclpy.init(args=args)
    
    node = PointNav()
    node.nav()
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
