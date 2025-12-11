import rospy
import cv2
import numpy as np
import os
script_dir = os.path.dirname(os.path.realpath(__file__))
os.chdir(script_dir)

import sys
sys.path.append('/home/adminpc/DucoCobotAPI')
from DucoCobotAPI_py.SiasunRobot import SiasunRobotPythonInterface
from DucoCobotAPI_py.DHGrasperInterface import DHGrasperInterface
sys.path.append('/home/adminpc/catkin_ws/src/grasp_topic')
import time

robot = SiasunRobotPythonInterface()

init = [-49, -20, 115, 277, -57.2, -188]

robot.moveJ(init)

