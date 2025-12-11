#!/home/adminpc/anaconda3/envs/py3.10/bin/python 

import sys
print("\n===== ROS Python path =====")
print(f"sys.executable: {sys.executable}")
print(f"version: {sys.version.split()[0]}")
print("=============================\n")


import rospy
import cv2
import numpy as np
import os
script_dir = os.path.dirname(os.path.realpath(__file__))
os.chdir(script_dir)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist,PoseStamped
from grasp_topic.msg import camera, pose
import pyrealsense2 as rs

sys.path.append('/home/adminpc/DucoCobotAPI')
from DucoCobotAPI_py.SiasunRobot import SiasunRobotPythonInterface
from DucoCobotAPI_py.DHGrasperInterface import DHGrasperInterface
from DucoCobotAPI_py.SlamwareInterface import SlamwareInterface
sys.path.append('/home/adminpc/catkin_ws/src/grasp_topic')
from utils.script_utils import euler_to_rotation_matrix, rotation_matrix_to_euler, getch
import time

class MobileManipulatorInterface(object):
    def __init__(self, verbose=True):
        self.verbose = verbose
        self.echo_info('========== Mobile manipulator initializing ... ==========')

        self.echo_info('=> ROS node initializing ...')
        self.node = rospy.init_node('mobile_manipulator')
        self.pub_image = rospy.Publisher('/image', camera, queue_size=10)
        self.echo_info('=> ROS node initialized!')

        self.echo_info('=> Robot arm initializing ...')
        self.robot = SiasunRobotPythonInterface()
        self.obs_joint = [0.0,-13.97,79.06,144.27,90,0.0]
        self.hold_joint = [0.0,-13.97,79.06,115.16,90,0.0]
        
        # self.obs_joint_button = [0.0,-13.97,79.06,118.27,90,90.0]
        # self.obs_joint_button = [0,-34.5,89.36,128.27,115,90.7]

        # self.obs_joint_button = [0,-34.5,89.36,128.27,115,0.]
        self.obs_joint_button = [0,-44.5,113.36,113.27,115,0.]

        
        self.echo_info('=> Robot arm initialized!')

        self.echo_info('=> Gripper initializing ...')
        self.gripper = DHGrasperInterface()
        self.gripper.set_force(5)
        self.echo_info('=> Gripper initialized!')
        
        self.echo_info('=> Camera initializing ...')
        self.camera = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        profile = self.camera.start(config)
        self.align = rs.align(rs.stream.color)
        self.echo_info('=> Camera initialized!')

        self.echo_info('=> Mobile base initializing ...')
        self.mobile_base = SlamwareInterface()
        self.echo_info('=> Mobile base initialized.')

        self.obj_pose = None
        self.operate_pose = None
        self.echo_info('========== Mobile manipulator initialized! ==========')
        
        self.button_pose = None
        self.get_floor = False
        
    def echo_info(self, info):
        if self.verbose:
            print(info)

    def deliver_image(self, color_image, depth_data):
        self.echo_info('- Image delivering ...')
        img_msg = camera()
        img_msg.size = list(color_image.shape)
        size = img_msg.size[0]*img_msg.size[1]*img_msg.size[2]
        img_msg.color = color_image.reshape(size).tolist()
        depth_npy = depth_data.astype(np.uint16)
        img_msg.depth = depth_npy.flatten().tolist()
        self.pub_image.publish(img_msg)
        self.echo_info('- Image delivered!')

    def grasp_obj(self, x_bias=-50, y_bias=60, z_bias=20):
        '''
        bias: transform between estimated pose and actual TCP (mm)
        '''
        self.echo_info('- Grasping begin ...')
        self.echo_info('-- go to observation pose')
        self.robot.moveJ(self.obs_joint)
        self.echo_info('-- open gripper')
        self.gripper.move_to(1000)

        if self.obj_pose is None:
            print('ERROR: obj_pose is None')
            return
        pose = self.obj_pose
        translation = pose[:3, 3]*1000
        rotation = pose[:3, :3]
        euler_angles = rotation_matrix_to_euler(rotation)
        
        transformed_pose = [float(translation[0]) + x_bias, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
                        [90.04, 2.91, 88.81]
                        #    [float(euler_angles[0]), float(euler_angles[1]), float(euler_angles[2])]
        catch_ready_pose = [float(translation[0]) + x_bias - 100, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
                        [90.04, 2.91, 88.81]
                        #    [float(euler_angles[0]), float(euler_angles[1]), float(euler_angles[2])]

        self.operate_pose = transformed_pose

        # print("transformed_pose:", transformed_pose)
        self.echo_info('- Grasp object at:')
        self.echo_info(transformed_pose)

        ret = self.robot.moveJ_pose(catch_ready_pose)
        ret = self.robot.moveJ_pose(transformed_pose)

        self.gripper.move_to(0)

        self.robot.moveJ(self.hold_joint)
        self.echo_info('- Grasping finished!')

    def press_button(self, x_bias=-28, y_bias=48, z_bias=5): # x, y middle is -45, 60
        '''
        bias: transform between estimated pose and actual TCP (mm)
        '''
        self.echo_info('- Press begin ...')
        self.echo_info('-- go to observation pose')
        self.robot.moveJ(self.obs_joint_button)
        self.echo_info('-- close gripper')
        self.gripper.move_to(0)

        if self.button_pose is None:
            print('ERROR: obj_pose is None')
            return
        pose = self.button_pose
        translation = pose[:3, 3]*1000
        rotation = pose[:3, :3]
        euler_angles = rotation_matrix_to_euler(rotation)
        

        # transformed_pose = [float(translation[0]) + x_bias, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
        #                 [90.04, 2.91, 88.81]
        # catch_ready_pose = [float(translation[0]) + x_bias - 100, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
        #                 [90.04, 2.91, 88.81]

        # transformed_pose = [float(translation[0]) + x_bias, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
        #                 [120.04, 0.91, 88.81]
        # catch_ready_pose = [float(translation[0]) + x_bias - 100, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
        #                 [120.04, 0.91, 88.81]

        transformed_pose = [float(translation[0]) + x_bias, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
                        [65.64, -10.1, 64.81]
        catch_ready_pose = [float(translation[0]) + x_bias - 100, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
                        [65.64, -10.1, 64.81]

        
        self.operate_pose = transformed_pose

        # print("transformed_pose:", transformed_pose)
        self.echo_info('- Press button at:')
        self.echo_info(transformed_pose)

        ret = self.robot.moveJ_pose(catch_ready_pose)
        time.sleep(1)
        ret = self.robot.moveJ_pose(transformed_pose)
        time.sleep(2)
        
        ret = self.robot.moveJ_pose(catch_ready_pose)

        self.robot.moveJ(self.obs_joint_button)
        
        self.echo_info('- Press finished!')
        
    
    def place_at_fixed_height(self, height):
        self.echo_info('- Object placing ...')
        if self.operate_pose is None:
            print('ERROR: operate_pose is None!')
            return True
        # world coord 0.85m -> base coord 0.2m
        target_height = (height * 1000) - 650
        drop_pose = [self.operate_pose[0], self.operate_pose[1], target_height, self.operate_pose[3], self.operate_pose[4], self.operate_pose[5]]
        post_drop_pose = [self.operate_pose[0] - 100, self.operate_pose[1], target_height, self.operate_pose[3], self.operate_pose[4], self.operate_pose[5]]
        self.robot.moveJ_pose(drop_pose)
        self.gripper.move_to(1000)
        rospy.sleep(1)
        self.robot.moveJ_pose(post_drop_pose)
        self.robot.moveJ(self.obs_joint)
        self.echo_info('- Object placed!')

    def estimate_obj_callback(self, pose):
        rospy.loginfo('Subscriber: obj pose received!')
        obj2eye = np.array(pose.pose).reshape(4, 4)
        # eye2tcp
        eye2tcp_path = os.path.join("/home/adminpc/robotics/eye_hand_calib/data", "eye2tcp_matrix.txt")
        eye2tcp = np.loadtxt(eye2tcp_path)
        eye2tcp[:3, 3] = eye2tcp[:3, 3] / 1000 # convert to meters
        obj2tcp = eye2tcp @ obj2eye
        obj_pose = self.tcp2base @ obj2tcp
        self.obj_pose = obj_pose
        
    def estimate_button_callback(self, pose):
        rospy.loginfo('Subscriber: obj pose received!')
        obj2eye = np.array(pose.pose).reshape(4, 4)
        # eye2tcp
        eye2tcp_path = os.path.join("/home/adminpc/robotics/eye_hand_calib/data", "eye2tcp_matrix.txt")
        eye2tcp = np.loadtxt(eye2tcp_path)
        eye2tcp[:3, 3] = eye2tcp[:3, 3] / 1000 # convert to meters
        obj2tcp = eye2tcp @ obj2eye
        button_pose = self.tcp2base @ obj2tcp
        self.button_pose = button_pose
    
    def get_observation(self, is_button=False):
        """
        Get current camera observation with aligned color and depth images.
        
        Returns:
            tuple: (color_image, depth_colormap, depth_data)
                   - color_image: BGR color image (numpy array)
                   - depth_colormap: Color-mapped depth image for visualization
                   - depth_data: Raw depth data (numpy array)
        """
        try:
            if is_button:
                self.robot.moveJ(self.obs_joint_button)
            else:
                self.robot.moveJ(self.obs_joint)
            self.echo_info('- Photo taking ...')
            # Wait for frames and align them
            frames = self.camera.wait_for_frames()
            aligned_frames = self.align.process(frames)

            time.sleep(0.5)
            
            # Get depth and color frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                print('ERROR: Image is None!')
                return None, None, None
            
            # Convert images to numpy arrays
            depth_data = np.asanyarray(depth_frame.get_data(), dtype="float16")
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Apply colormap to depth image
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            self.echo_info('- Photo taken!')
            return color_image, depth_colormap, depth_data
            
        except Exception as e:
            print(f"Error getting observation: {e}")
            return None, None, None
    
    def observe_floor(self, target):
        rospy.loginfo('Subscriber: floor number received!')
        floors = np.array(pose.pose).tolist()
        if 6 in floors:
            self.get_floor = True

    def instruction(self):
        print("Optional choice:\n" \
              "-> Mobile base:\n" \
              "---> 0: Go to cafe counter; 1: Go to lab table; 2: Print Pose.\n" \
              "-> Arm:\n" \
              "---> 3: Grasp white cup; 4: Place cup; 5: Print Pose.\n" \
              "-> Gripper:\n" \
              "---> 6: Close; 7: Open; 8: Print Pose.\n" \
              "---> 9: Press button" \
              "-> Q: Exit.\n" \
              "Please enter from keyboard (NO NEED TO ENTER) >>")

    def run(self):
        print("\n========================================")
        print("<<< Mobile manipulator scripts begin >>>")
        self.instruction()

        while not rospy.is_shutdown():
            key = getch()
            if key == '0':
                self.echo_info('Operating: Navigating to cafe counter ...')
                self.mobile_base.nav_to_obj('counter')
            elif key == '1':
                self.echo_info('Operating: Navigating to cafe lab table ...')
                self.mobile_base.nav_to_obj('center_table')
            elif key == '2':
                cur_pose = self.mobile_base.get_current_pose()
                print('Mobile base current pose:', cur_pose)
            elif key == '3':
                self.echo_info('Operating: Grasping white cup ...')
                color_image, depth_colormap, depth_data = self.get_observation()
                self.tcp2base = self.robot.get_RT_matrix()
                self.tcp2base[:3, 3] = self.tcp2base[:3, 3] / 1000
                self.deliver_image(color_image, depth_data)
                
                sub_obj_pose = rospy.Subscriber('/pose', pose, self.estimate_obj_callback, callback_args=None, queue_size=1)
                rospy.wait_for_message("/pose", pose, timeout=None)
                sub_obj_pose.unregister()  # one time subscribe

                self.grasp_obj()
            elif key == '4':
                self.echo_info('Operating: Placing white cup ...')
                self.place_at_fixed_height(0.75) # cafe counter: 0.90，lab center table: 0.75，exp table: 0.85
            elif key == '5':
                arm_pose = self.robot.get_RT_matrix()
                print('Robot arm current pose:\n', arm_pose)
            elif key == '6':
                self.echo_info('Operating: Closing gripper ...')
                self.gripper.move_to(0)
            elif key == '7':
                self.echo_info('Operating: Opening gripper ...')
                self.gripper.move_to(1000)
            elif key == '8':
                self.echo_info('Operating: Closing gripper ...')
                print('Gripper current pose:', self.gripper.get_position())
            elif key == '9':
                self.echo_info('Operating: Press button ...')
                self.robot.moveJ(self.obs_joint_button)
                time.sleep(0.5)
                color_image, depth_colormap, depth_data = self.get_observation(is_button=True)
                self.tcp2base = self.robot.get_RT_matrix()
                self.tcp2base[:3, 3] = self.tcp2base[:3, 3] / 1000
                self.deliver_image(color_image, depth_data)
                
                sub_obj_pose = rospy.Subscriber('/button_pose', pose, self.estimate_button_callback, callback_args=None, queue_size=1)
                rospy.wait_for_message("/button_pose", pose, timeout=None)
                sub_obj_pose.unregister()  # one time subscribe

                self.press_button()

                bool_get_floor = False
                while not bool_get_floor:
                    color_image, depth_colormap, depth_data = self.get_observation(is_button=True)
                    self.deliver_image(color_image, depth_data)
                    floor_num = rospy.Subscriber('/floor', pose, self.observe_floor, callback_args=None, queue_size=1)
                    rospy.wait_for_message("/floor", pose, timeout=None)
                    floor_num.unregister()  # one time subscribe
                    
                    # self.observe_floor(floor_num)
                    if self.get_floor:
                        self.echo_info('Got to the target floor ...')
                        bool_get_floor = True
                        # break
                    time.sleep(1)
                
            elif key == 'A':
                self.mobile_base.rec_loc(5.3, -5.36, 9.5, -17)
                
            elif key == 'Q':
                break
            else:
                print('Invalid input. Retry >>')
                continue

            print("continue ...\n")
            self.instruction()


import rospy
from std_msgs.msg import String, Bool
from collections import deque

class DedupePlanSubscriber:
    def __init__(self):
        self.mobile_manipulator = MobileManipulatorInterface()
        rospy.init_node('mobile_manipulator')
        
        # 存储最近处理过的命令（避免重复处理）
        self.processed_commands = deque(maxlen=100)  # 保存最近100条命令
        
        # 发布确认消息
        self.ack_pub = rospy.Publisher('/plan_ack', Bool, queue_size=1)
        
        # 持续订阅plan话题
        self.plan_sub = rospy.Subscriber('/plan', String, self.plan_callback, queue_size=10)
        self.sub_obj_pose = rospy.Subscriber('/pose', pose, self.mobile_manipulator.estimate_obj_callback, callback_args=None, queue_size=1)

        rospy.loginfo("Dedupe subscriber ready (will skip duplicate commands)")

    def plan_callback(self, msg):
        """处理新命令（跳过重复命令）"""
        if msg.data in self.processed_commands:
            rospy.loginfo(f"Skipping duplicate command: {msg.data}")
            return
            
        rospy.loginfo(f"Processing new command: {msg.data}")
        self.processed_commands.append(msg.data)
        
        success = self.execute_command(msg.data)
        
        # 发送确认
        ack = Bool()
        ack.data = success
        self.ack_pub.publish(ack)

    def execute_command(self, command):
        try:
            rospy.loginfo(f"Executing: {command}")
            # 这里添加实际命令执行代码
            if "observe" in command:
                color_image, depth_colormap, depth_data = self.mobile_manipulator.get_observation()
                self.mobile_manipulator.tcp2base = self.mobile_manipulator.robot.get_RT_matrix()
                self.mobile_manipulator.tcp2base[:3, 3] = self.mobile_manipulator.tcp2base[:3, 3] / 1000
                self.mobile_manipulator.deliver_image(color_image, depth_data)
            elif "go to" in command:
                obj = command.split(':')[-1].lstrip()
                self.mobile_manipulator.mobile_base.nav_to_obj(obj)
            elif "pick" in command:
                print("wait for pose")
                print(self.mobile_manipulator.obj_pose)
                # rospy.wait_for_message("/pose", pose, timeout=None)
                self.sub_obj_pose.unregister()
                print("pose recieved")
                self.mobile_manipulator.grasp_obj()
            elif "place" in command:
                self.mobile_manipulator.place_at_fixed_height(0.75)
            return True
        except Exception as e:
            rospy.logerr(f"Command failed: {str(e)}")
            return False

# if __name__ == '__main__':
#     DedupePlanSubscriber()
#     rospy.spin()

if __name__ == '__main__':
    baseline = MobileManipulatorInterface()
    baseline.run()