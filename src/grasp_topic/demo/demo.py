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

gripper = DHGrasperInterface()
# gripper.set_force(5)
gripper.move_to(500)

robot = SiasunRobotPythonInterface()


init = [-49, -20, 115, 277, -57.2, -188]
card_pos = [-35.58, 47.9, 44.4, 265.4, -58.9,-180.32]
back = [-38.68, 34.4, 66.4, 256.9, -56.8,-180.32]
button = [-35.62, 50.95, 46.07, 283.4, -69.9, -191.2]



robot.moveJ(init, vel=100, acc=100)
robot.moveJ(card_pos, vel=100, acc=100)
robot.moveJ(back, vel=100, acc=100)
robot.moveJ(button)
robot.moveJ(init)
