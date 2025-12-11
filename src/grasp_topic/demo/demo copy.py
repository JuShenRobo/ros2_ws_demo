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
obs_joint = [0.0,-13.97,79.06,144.27,90,0.0]
pre_target = [-45.8, 3.16, 61.42, 117.36, 70.28, -1.59]
target_joint = [-43.62, 17.72, 43.65, 120.55, 72.46, -1.53]
# robot.moveJ(obs_joint)
# time.sleep(0.2)
# robot.moveJ(pre_target)
# robot.moveJ(target_joint)
# robot.moveJ(obs_joint)

init = [-49, -20, 115, 277, -57.2, -188]
card_pos = [-44.46, 6.24, 84.08, 301.87, -50.73,-203.87]
back = [-46.64, 2.88, 87.59, 302.71, -48.89, -205.41]
button = [-40.45, 16.03, 102.39, 272.17, -54.22, -201.22]



robot.moveJ(init)
robot.moveJ(card_pos, vel=100, acc=100)
robot.moveJ(back)
robot.moveJ(button, vel=100, acc=100)
robot.moveJ(init)
