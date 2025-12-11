#!/usr/bin/env python3


import sys
print("\n===== ROS 2 Python path =====")
print(f"sys.executable: {sys.executable}")
print(f"version: {sys.version.split()[0]}")
print("=============================\n")

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
script_dir = os.path.dirname(os.path.realpath(__file__))
os.chdir(script_dir)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from grasp_topic.msg import Camera, Pose 
import pyrealsense2 as rs
from std_msgs.msg import String, Bool
from collections import deque
import time

sys.path.append('/home/yofo/DucoCobotAPI') # debug：替换为实际路径
from DucoCobotAPI_py.SiasunRobot import SiasunRobotPythonInterface
from DucoCobotAPI_py.DHGrasperInterface import DHGrasperInterface
from DucoCobotAPI_py.SlamwareInterface_ros2 import SlamwareInterface # debug
sys.path.append('/home/yofo/ros2_ws/src/grasp_topic') # debug：替换为实际路径
from utils.script_utils import euler_to_rotation_matrix, rotation_matrix_to_euler, getch

import time

from control_host_model import Button_yolo

'''MobileManipulatorInterface类是一个手动交互模式, 通过按键来触发任务；
    它在控制主机上主动发布任务;
    采用临时订阅模式, 只在特定用户操作时（按下按键）需要数据
'''
class MobileManipulatorInterface(Node):
     # 构造函数：各种硬件和ros节点的初始化
    def __init__(self, verbose=True):
        super().__init__('mobile_manipulator')
        # 控制输出调试信息
        self.verbose = verbose
        self.echo_info('========== Mobile manipulator initializing ... ==========')

        self.echo_info('=> ROS 2 node initializing ...')
        self.pub_image = self.create_publisher(Camera, '/image', 10)
        self.echo_info('=> ROS 2 node initialized!')

        self.echo_info('=> Robot arm initializing ...')
        self.robot = SiasunRobotPythonInterface()
        self.obs_joint = [-21.57, 4.97, 109.53, 94.10, 79.46, -5.09]
        # self.obs_joint = [0.0, -13.97, 79.06, 144.27, 90, 0.0]

        # self.hold_joint = [0.0, -13.97, 79.06, 115.16, 90, 0.0]
        self.hold_joint = [-11.95, -9.83, 97.83, 91.42, 84, -5.09]
        # self.obs_joint_button = [0, -44.5, 113.36, 113.27, 115, 0.]
        self.obs_joint_button = [150, 23.5, -121.36, 90.27, 101, 175.]
        self.echo_info('=> Robot arm initialized!')

        self.echo_info('=> Gripper initializing ...') # debug
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

        self.echo_info('=> Mobile base initializing ...') # debug
        self.mobile_base = SlamwareInterface()
        self.echo_info('=> Mobile base initialized.')

        self.obj_pose = None
        self.operate_pose = None
        self.button_pose = None
        self.card_pose = None
        self.get_floor = False
        
        self.button_yolo = Button_yolo()
        
        # 创建订阅者
        self.obj_pose_sub = self.create_subscription(
            Pose, 
            '/pose', 
            self.estimate_obj_callback, 
            10)
            
        self.button_pose_sub = self.create_subscription(
            Pose, 
            '/button_pose', 
            self.estimate_button_callback, 
            10)
            
        self.floor_sub = self.create_subscription(
            Pose, 
            '/floor', 
            self.observe_floor, 
            10)
            
        self.echo_info('========== Mobile manipulator initialized! ==========')
        
    def echo_info(self, info):
        if self.verbose:
            self.get_logger().info(info)

    def deliver_image(self, color_image, depth_data):
        self.echo_info('- Image delivering ...')
        img_msg = Camera()
        img_msg.size = list(color_image.shape)
        size = img_msg.size[0]*img_msg.size[1]*img_msg.size[2]
        img_msg.color = color_image.reshape(size).tolist()
        depth_npy = depth_data.astype(np.uint16)
        img_msg.depth = depth_npy.flatten().tolist()
        self.pub_image.publish(img_msg)
        self.echo_info('- Image delivered!')

    '''利用订阅得到的pose抓取: 转换pose到6D --> 机械臂移动到观察姿态 --> 移动到catch_ready姿态 --> 
        移动到抓取姿态,夹爪闭合 --> 移动到持握姿态
    '''
    def grasp_obj(self, x_bias=22.613, y_bias=34.633, z_bias=10.072):   
        '''
        bias: transform between estimated pose and actual TCP (mm)
        '''
        self.echo_info('- Grasping begin ...')
        self.echo_info('-- go to observation pose')
        self.robot.moveJ(self.obs_joint)
        self.echo_info('-- open gripper')
        self.gripper.move_to(1000) # debug

        if self.obj_pose is None:
            self.get_logger().error('ERROR: obj_pose is None')
            return
        pose = self.obj_pose
        translation = pose[:3, 3]*1000
        rotation = pose[:3, :3]
        euler_angles = rotation_matrix_to_euler(rotation)
        
        # transformed_pose = [float(translation[0]) + x_bias, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
        #                 [90.04, 2.91, 88.81]
        # catch_ready_pose = [float(translation[0]) + x_bias - 100, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
        #                 [90.04, 2.91, 88.81]
        transformed_pose = [float(translation[0]) + x_bias, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
                        [0.81, -17.5, 0.87]
        catch_ready_pose = [float(translation[0]) + x_bias - 100, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
                        [0.81, -17.5, 0.87]

        self.operate_pose = transformed_pose

        self.echo_info('- Grasp object at:')
        self.echo_info(str(transformed_pose))

        print("catch_ready_pose:", catch_ready_pose)
        ret = self.robot.moveJ_pose(catch_ready_pose)
        ret = self.robot.moveJ_pose(transformed_pose)

        self.gripper.move_to(0) # debug

        self.robot.moveJ(self.hold_joint)
        self.echo_info('- Grasping finished!')

    def reset_grasp_pose(self, x_bias=0, y_bias=0, z_bias=0):   
        '''
        bias: transform between estimated pose and actual TCP (mm)
        '''
        self.echo_info('- Grasping begin ...')
        self.echo_info('-- go to observation pose')
        self.robot.moveJ(self.obs_joint)
        self.echo_info('-- open gripper')
        self.gripper.move_to(1000) # debug

        if self.obj_pose is None:
            self.get_logger().error('ERROR: obj_pose is None')
            return
        pose = self.obj_pose
        translation = pose[:3, 3]*1000
        rotation = pose[:3, :3]
        euler_angles = rotation_matrix_to_euler(rotation)
        
        # transformed_pose = [float(translation[0]) + x_bias, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
        #                 [90.04, 2.91, 88.81]
        # catch_ready_pose = [float(translation[0]) + x_bias - 100, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
        #                 [90.04, 2.91, 88.81]
        transformed_pose = [float(translation[0]) + x_bias, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
                        [18.93, -19.17, -61.98]
        catch_ready_pose = [float(translation[0]) + x_bias - 100, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
                        [18.93, -19.17, -61.98]

        self.operate_pose = transformed_pose

        self.echo_info('- Grasp object at:')
        self.echo_info(str(transformed_pose))

        print("pose before reset:", transformed_pose)
    
    def press_button_up(self, x_bias=-70, y_bias=130, z_bias=-40):
        '''
        bias: transform between estimated pose and actual TCP (mm)
        '''
        self.echo_info('- Press begin ...')
        self.echo_info('-- go to observation pose')
        self.robot.moveJ(self.obs_joint_button)
        self.echo_info('-- close gripper')
        self.gripper.move_to(1000)

        if self.button_pose is None:
            self.get_logger().error('ERROR: button_pose is None')
            return
        
        if self.card_pose is None:
            self.get_logger().error('ERROR: card_pose is None')
            return

        pose = self.button_pose
        # card_pose = self.card_pose

        translation = pose[:3, 3]*1000
        # card_translation = card_pose[:3, 3]*1000
        # rotation = pose[:3, :3]
        # euler_angles = rotation_matrix_to_euler(rotation)

        print("translation", translation)


        button_pose = [float(translation[0]) + x_bias+15, float(translation[1]) + y_bias-65, float(translation[2]) + z_bias] + \
                        [-24.,18.,-5.]
                        # [-27., 20., 15.]
                        #    [float(euler_angles[0]), float(euler_angles[1]), float(euler_angles[2])]
        

        self.echo_info('- Press button at:')
        self.echo_info(str(button_pose))

        ret = self.robot.moveJ_pose(button_pose)

        # ret = self.robot.moveJ_pose(card_pose, vel=100, acc=100)
        # ret = self.robot.moveJ_pose(back_pose, vel=100, acc=100)
        # ret = self.robot.moveJ_pose(button_pose, vel=100, acc=100)
        
        self.robot.moveJ(self.obs_joint_button)
        
        self.echo_info('- Press finished!')

    def press_button(self, x_bias=-70, y_bias=130, z_bias=-40):
        '''
        bias: transform between estimated pose and actual TCP (mm)
        '''
        self.echo_info('- Press begin ...')
        self.echo_info('-- go to observation pose')
        self.robot.moveJ(self.obs_joint_button)
        self.echo_info('-- close gripper')
        self.gripper.move_to(1000)

        if self.button_pose is None:
            self.get_logger().error('ERROR: button_pose is None')
            return
        
        if self.card_pose is None:
            self.get_logger().error('ERROR: card_pose is None')
            return

        pose = self.button_pose
        card_pose = self.card_pose

        translation = pose[:3, 3]*1000
        card_translation = card_pose[:3, 3]*1000
        # rotation = pose[:3, :3]
        # euler_angles = rotation_matrix_to_euler(rotation)

        print("translation", translation)

        card_pose = [float(card_translation[0]) + x_bias, float(card_translation[1]) + y_bias+40, float(card_translation[2]) + z_bias] + \
                        [-45., -10., 10.]
                        #    [float(euler_angles[0]), float(euler_angles[1]), float(euler_angles[2])]

        back_pose = [float(card_translation[0]) + x_bias - 120, float(card_translation[1]) + y_bias, float(card_translation[2]) + z_bias] + \
                        [-45., -10., 10.]

        button_pose = [float(translation[0]) + x_bias+15, float(translation[1]) + y_bias-65, float(translation[2]) + z_bias] + \
                        [-24.,18.,-5.]
                        # [-27., 20., 15.]
                        #    [float(euler_angles[0]), float(euler_angles[1]), float(euler_angles[2])]
        

        self.echo_info('- Press button at:')
        self.echo_info(str(button_pose))

        ret = self.robot.moveJ_pose(card_pose)
        ret = self.robot.moveJ_pose(back_pose)
        ret = self.robot.moveJ_pose(button_pose)

        # ret = self.robot.moveJ_pose(card_pose, vel=100, acc=100)
        # ret = self.robot.moveJ_pose(back_pose, vel=100, acc=100)
        # ret = self.robot.moveJ_pose(button_pose, vel=100, acc=100)
        
        self.robot.moveJ(self.obs_joint_button)
        
        self.echo_info('- Press finished!')
    
    '''放置杯子: 放置姿态设置为抓取姿态, 即放置在桌子上的位置完全相同, 除了需要设置桌子的高度；
        移动到放置姿态 --> 松开夹爪 --> 机械臂后撤 --> 移动到观察姿态
    '''
    def place_at_fixed_height(self, height):
        self.echo_info('- Object placing ...')
        if self.operate_pose is None:
            self.get_logger().error('ERROR: operate_pose is None!')
            return True
        # world coord 0.85m -> base coord 0.2m
        target_height = (height * 1000) - 650 # 从真实世界的高度转换到机械臂基座坐标系的高度
        drop_pose = [self.operate_pose[0], self.operate_pose[1], target_height, self.operate_pose[3], self.operate_pose[4], self.operate_pose[5]]
        post_drop_pose = [self.operate_pose[0] - 100, self.operate_pose[1], target_height, self.operate_pose[3], self.operate_pose[4], self.operate_pose[5]]
        self.robot.moveJ_pose(drop_pose)
        self.gripper.move_to(1000) # debug
        time.sleep(1)
        self.robot.moveJ_pose(post_drop_pose)
        self.robot.moveJ(self.obs_joint)
        self.echo_info('- Object placed!')

    # 处理接收到pose的回调函数：从相机坐标系到机械臂基座坐标系的转换
    def estimate_obj_callback(self, pose_msg):
        self.get_logger().info('Subscriber: obj pose received!')
        obj2eye = np.array(pose_msg.pose).reshape(4, 4)
        # eye2tcp
        eye2tcp_path = os.path.join("/home/yofo/robotics/eye_hand_calib/data", "eye2tcp_matrix.txt") # debug
        eye2tcp = np.loadtxt(eye2tcp_path)
        eye2tcp[:3, 3] = eye2tcp[:3, 3] / 1000 # convert to meters
        obj2tcp = eye2tcp @ obj2eye
        obj_pose = self.tcp2base @ obj2tcp
        self.obj_pose = obj_pose
        
    def estimate_button_callback(self, pose_msg, pose_card):
        self.get_logger().info('Subscriber: button pose received!')
        obj2eye = pose_msg
        eye2tcp_path = os.path.join("/home/yofo/robotics/eye_hand_calib/data", "eye2tcp_matrix.txt")
        eye2tcp = np.loadtxt(eye2tcp_path)
        eye2tcp[:3, 3] = eye2tcp[:3, 3] / 1000 # convert to meters
        obj2tcp = eye2tcp @ obj2eye
        button_pose = self.tcp2base @ obj2tcp
        self.button_pose = button_pose

        obj2eye = pose_card
        obj2tcp = eye2tcp @ obj2eye
        card_pose = self.tcp2base @ obj2tcp
        self.card_pose = card_pose
        
    
    # 移动到观察位姿，进行拍照，还没有发布图片
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
                self.get_logger().error('ERROR: Image is None!')
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
            self.get_logger().error(f"Error getting observation: {e}")
            return None, None, None
    
    def observe_floor(self, pose_msg):
        self.get_logger().info('Subscriber: floor number received!')
        floors = np.array(pose_msg.pose).tolist()
        if 6 in floors:
            self.get_floor = True

    # 打印出所有支持的功能
    def instruction(self):
        print("Optional choice:\n" \
              "-> Mobile base:\n" \
              "---> 0: Go to pick tabel; 1: Go to place table; 2: Print Pose.\n" \
              "-> Arm:\n" \
              "---> 3: Grasp white cup; 4: Place cup; 5: Print Pose.\n" \
              "-> Gripper:\n" \
              "---> 6: Close; 7: Open; 8: Print Pose.\n" \
              "---> 9: Press button" \
              "---> T: Reset Grasp pose" \
              "-> Q: Exit.\n" \
              "Please enter from keyboard (NO NEED TO ENTER) >>")

    # 最终循环执行交互式任务的函数
    def run(self):
        print("\n========================================")
        print("<<< Mobile manipulator scripts begin >>>")
        self.instruction()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)  # 处理ROS消息
            
            if not sys.stdin.isatty():
                continue
                
            if os.name == 'nt':
                import msvcrt
                if msvcrt.kbhit():
                    key = msvcrt.getch().decode('utf-8')
                else:
                    continue
            else:
                key = getch()
                
            if key == '0':
                self.echo_info('Operating: Navigating to pick table ...')
                self.mobile_base.nav_to_obj('pick_table') # debug
            elif key == '1':
                self.echo_info('Operating: Navigating to place table ...')
                self.mobile_base.nav_to_obj('place_table') # debug
            elif key == '2':
                cur_pose = self.mobile_base.get_current_pose() # debug
                print('Mobile base current pose:', cur_pose)
            elif key == '3':
                self.echo_info('Operating: Grasping white cup ...')
                color_image, depth_colormap, depth_data = self.get_observation()
                self.tcp2base = self.robot.get_RT_matrix()
                self.tcp2base[:3, 3] = self.tcp2base[:3, 3] / 1000
                self.deliver_image(color_image, depth_data)
                
                # 创建一个Future来等待消息
                future = rclpy.task.Future()
                
                def callback(msg):
                    self.estimate_obj_callback(msg)
                    future.set_result(True)
                
                subscription = self.create_subscription(
                    Pose, 
                    '/pose', 
                    callback, 
                    10)
                    
                # 等待消息
                rclpy.spin_until_future_complete(self, future)
                self.destroy_subscription(subscription)
                print("得到传输位姿态：", self.obj_pose) # debug
                self.grasp_obj() # debug
            elif key == '4':
                self.echo_info('Operating: Placing white cup ...')
                self.place_at_fixed_height(0.75) # cafe counter: 0.90，lab center table: 0.75，exp table: 0.85
            elif key == '5':
                arm_pose = self.robot.get_RT_matrix()
                print('Robot arm current pose:\n', arm_pose)
            elif key == '6':
                self.echo_info('Operating: Closing gripper ...')
                self.gripper.move_to(0) # debug
            elif key == '7':
                self.echo_info('Operating: Opening gripper ...')
                self.gripper.move_to(1000) # debug
            elif key == '8':
                self.echo_info('Operating: Closing gripper ...')
                print('Gripper current pose:', self.gripper.get_position()) # debug
            elif key == '9':
                self.echo_info('Operating: Press button ...')
                self.robot.moveJ(self.obs_joint_button)
                time.sleep(0.5)
                color_image, depth_colormap, depth_data = self.get_observation(is_button=True)
                self.tcp2base = self.robot.get_RT_matrix()
                self.tcp2base[:3, 3] = self.tcp2base[:3, 3] / 1000
                # self.deliver_image(color_image, depth_data)

                # print(color_image.shape, depth_data.shape)
                # print(depth_data.min(),depth_data.max()) # 0.0 9540.0
                cv2.imwrite("rgb.png", color_image)
                # cv2.imwrite("dep.png", depth_data)
                np.save("dep.npy", depth_data)
                
                # 创建一个Future来等待按钮位置消息
                # future = rclpy.task.Future()
                
                # def callback(msg):
                #     self.estimate_button_callback(msg)
                #     future.set_result(True)
                
                pose, pose_card = self.button_yolo.run_yolo(8)
                # print(pose)
                self.estimate_button_callback(pose, pose_card)
                
                # button_sub = self.create_subscription(
                #     Pose, 
                #     '/button_pose', 
                #     callback, 
                #     10)
                    
                # # 等待消息
                # rclpy.spin_until_future_complete(self, future)
                # self.destroy_subscription(button_sub)

                self.press_button()

                # bool_get_floor = False
                # while not bool_get_floor and rclpy.ok():
                #     color_image, depth_colormap, depth_data = self.get_observation(is_button=True)
                #     self.deliver_image(color_image, depth_data)
                    
                #     # 创建一个Future来等待楼层消息
                #     floor_future = rclpy.task.Future()
                    
                #     def floor_callback(msg):
                #         self.observe_floor(msg)
                #         floor_future.set_result(True)
                    
                #     floor_sub = self.create_subscription(
                #         Pose, 
                #         '/floor', 
                #         floor_callback, 
                #         10)
                        
                #     # 等待消息
                #     rclpy.spin_until_future_complete(self, floor_future)
                #     self.destroy_subscription(floor_sub)
                    
                #     if self.get_floor:
                #         self.echo_info('Got to the target floor ...')
                #         bool_get_floor = True
                #     time.sleep(1)

            elif key == 'T':
                self.echo_info('Operating: Reset Grasp pose ...')
                color_image, depth_colormap, depth_data = self.get_observation()
                self.tcp2base = self.robot.get_RT_matrix()
                self.tcp2base[:3, 3] = self.tcp2base[:3, 3] / 1000
                self.deliver_image(color_image, depth_data)
                
                # 创建一个Future来等待消息
                future = rclpy.task.Future()
                
                def callback(msg):
                    self.estimate_obj_callback(msg)
                    future.set_result(True)
                
                subscription = self.create_subscription(
                    Pose, 
                    '/pose', 
                    callback, 
                    10)
                    
                # 等待消息
                rclpy.spin_until_future_complete(self, future)
                self.destroy_subscription(subscription)
                # print("得到传输位姿态：", self.obj_pose) # debug
                self.reset_grasp_pose() # debug
                
            elif key == 'A':
                self.mobile_base.rec_loc(5.3, -5.36, 9.5, -17) # debug 
                
            elif key.upper() == 'Q':
                break
            else:
                print('Invalid input. Retry >>')
                continue

            print("continue ...\n")
            self.instruction()


'''DedupePlanSubscribers是一个自动处理任务序列, 采取的是事件驱动模式；
    视觉主机端发布任务列表, 控制端负责监听执行；
    采用持久订阅模式，需要持续监听命令流；
'''
class DedupePlanSubscriber(Node):
    def __init__(self):
        super().__init__('dedupe_plan_subscriber')
        self.mobile_manipulator = MobileManipulatorInterface()
        
        # 存储最近处理过的命令（避免重复处理）
        self.processed_commands = deque(maxlen=100)  # 保存最近100条命令
        
        # 发布确认消息
        self.ack_pub = self.create_publisher(Bool, '/plan_ack', 1)
        
        # 持续订阅plan话题
        self.plan_sub = self.create_subscription(
            String, 
            '/plan', 
            self.plan_callback, 
            10)
            
        self.obj_pose_sub = self.create_subscription(
            Pose, 
            '/pose', 
            self.mobile_manipulator.estimate_obj_callback, 
            10)

        self.get_logger().info("Dedupe subscriber ready (will skip duplicate commands)")


     # 最终就是通过这个回调函数实现自动化任务处理！！！
    '''控制主机监听到视觉主机发来的单个plan --> 判断是否是重复命令 --> 执行新命令，发布执行成功状态 --> 
        视觉主机接收到当前任务执行成功，发布下一个任务
    '''
    def plan_callback(self, msg):
        """处理新命令（跳过重复命令）"""
        if msg.data in self.processed_commands:
            self.get_logger().info(f"Skipping duplicate command: {msg.data}")
            return
            
        self.get_logger().info(f"Processing new command: {msg.data}")
        self.processed_commands.append(msg.data)
        
        success = self.execute_command(msg.data)
        
        # 发送确认
        ack = Bool()
        ack.data = success
        self.ack_pub.publish(ack)

    def execute_command(self, command):
        try:
            self.get_logger().info(f"Executing: {command}")
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
                self.get_logger().info("wait for pose")
                self.get_logger().info(str(self.mobile_manipulator.obj_pose))
                # 创建Future等待消息
                future = rclpy.task.Future()
                
                def callback(msg):
                    self.mobile_manipulator.estimate_obj_callback(msg)
                    future.set_result(True)
                
                subscription = self.create_subscription(
                    Pose, 
                    '/pose', 
                    callback, 
                    10)
                    
                # 等待消息
                rclpy.spin_until_future_complete(self, future)
                self.destroy_subscription(subscription)
                
                self.get_logger().info("pose received")
                self.mobile_manipulator.grasp_obj()
            elif "place" in command:
                self.mobile_manipulator.place_at_fixed_height(0.75)
            return True
        except Exception as e:
            self.get_logger().error(f"Command failed: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    # 选择使用哪个入口点
    use_dedupe_subscriber = False  # 设置为True使用DedupePlanSubscriber
    
    if use_dedupe_subscriber:
        node = DedupePlanSubscriber()
        rclpy.spin(node)
        node.destroy_node()
    else:
        node = MobileManipulatorInterface()
        node.run()
        node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
