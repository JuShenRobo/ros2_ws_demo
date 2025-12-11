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
import json
import time
from typing import Dict, List, Optional

script_dir = os.path.dirname(os.path.realpath(__file__))
os.chdir(script_dir)

# ROS messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,PoseStamped, Point, Quaternion, Twist
from grasp_topic.msg import Camera, Pose as GraspPose
import pyrealsense2 as rs
from std_msgs.msg import String, Bool
from collections import deque

# Robot API
sys.path.append('/home/yofo/DucoCobotAPI')
# from DucoCobotAPI_py.SiasunRobot import SiasunRobotPythonInterface
# from DucoCobotAPI_py.DHGrasperInterface import DHGrasperInterface
from DucoCobotAPI_py.SlamwareInterface_ros2 import SlamwareInterface

# YOLO model
# from utils.script_utils import euler_to_rotation_matrix, rotation_matrix_to_euler, getch
# from control_host_model import Button_yolo

# map
from slamware_ros_sdk.srv import SyncSetStcm
from rclpy.callback_groups import ReentrantCallbackGroup
import select
import tty
import termios

def kbhit():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class MobileManipulatorInterface(Node):
    def __init__(self, verbose=True):
        super().__init__('mobile_manipulator')

        self.verbose = verbose
        self.echo_info('========== Mobile manipulator initializing ... ==========')

        # === 参数声明 (用于区分运行模式) ===
        # self.declare_parameter('mode', 'interactive')  # 'interactive' or 'autonomous'
        # self.declare_parameter('start_floor', 1)
        # self.declare_parameter('target_floor', 2)
        # self.declare_parameter('nav_sequence', [])  # list of {"floor": int, "waypoint": str, "arm_pose": str}

        # self.mode = self.get_parameter('mode').get_parameter_value().string_value
        # self.start_floor = self.get_parameter('start_floor').get_parameter_value().integer_value
        # self.target_floor = self.get_parameter('target_floor').get_parameter_value().integer_value
        # self.nav_sequence = self.get_parameter('nav_sequence').get_parameter_value().string_array_value
        self.mode=""
        self.start_floor=1
        self.target_floor=2
        self.nav_sequence=['1','2']

        ##relocalize
        self.floor_maps_dict = {'1':"/home/yofo/ros2_ws_demo/src/slamware_ros_sdk/maps/floor1 5.12_1.stcm",
                                '2':"/home/yofo/ros2_ws_demo/src/slamware_ros_sdk/maps/floor2-5.12_1.stcm",
                                '3':"/home/yofo/ros2_ws_demo/src/slamware_ros_sdk/maps/floor3_1207.stcm"}
        self.set_stcm_client = self.create_client(SyncSetStcm, '/sync_set_stcm') #ros2 service list | grep stcm.
        while not self.set_stcm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('等待 sync_set_stcm 服务...')


        # === 硬件初始化 ===
        # self.echo_info('=> ROS 2 node initializing ...')
        # self.pub_image = self.create_publisher(Camera, '/image', 10)

        # self.echo_info('=> Robot arm initializing ...')
        # self.robot = SiasunRobotPythonInterface()
        # self.obs_joint = [-21.57, 4.97, 109.53, 94.10, 79.46, -5.09]
        # self.hold_joint = [-11.95, -9.83, 97.83, 91.42, 84, -5.09]
        # self.obs_joint_button = [150, 23.5, -121.36, 90.27, 101, 175.]

        # self.echo_info('=> Gripper initializing ...')
        # self.gripper = DHGrasperInterface()
        # self.gripper.set_force(5)

        # self.echo_info('=> Camera initializing ...')
        # self.camera = rs.pipeline()
        # config = rs.config()
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # profile = self.camera.start(config)
        # self.align = rs.align(rs.stream.color)

        self.echo_info('=> Mobile base initializing ...')
        self.mobile_base = SlamwareInterface() #这个接口得符合ros2库安装，然后索引到

        # === 数据存储 ===
        self.obj_pose = None
        self.button_pose = None
        self.card_pose = None
        self.tcp2base = None
        self.get_floor = False
        self.object_state = 'empty'
        self.operate_pose = None
        self.current_floor = self.start_floor  # 当前所在楼层
        # self.pose_config = self.load_pose_config()
        # self.button_yolo = Button_yolo()

        # === 订阅者 ===
        # self.create_subscription(GraspPose, '/pose', self.estimate_obj_callback, 10)
        # self.create_subscription(GraspPose, '/button_pose', self.estimate_button_callback, 10)
        # self.create_subscription(GraspPose, '/floor', self.observe_floor, 10)

        self.echo_info('========== Mobile manipulator initialized! ==========')

    def _load_map_for_floor(self, floor_key_str):
        if floor_key_str not in self.floor_maps_dict:
            self.get_logger().error(f"未定义楼层 '{floor_key_str}' 的地图文件.")
            return False

        map_file_path = self.floor_maps_dict[floor_key_str]
        self.get_logger().info(f"为楼层 '{floor_key_str}' 加载地图: {map_file_path}")

        if not os.path.isfile(map_file_path):
            self.get_logger().error(f"地图文件不存在: {map_file_path}")
            return False

        try:
            with open(map_file_path, 'rb') as f:
                map_data = f.read()
        except Exception as e:
            self.get_logger().error(f"读取地图文件失败: {e}")
            return False

        # 构造请求
        req = SyncSetStcm.Request()
        req.raw_stcm = map_data
        req.robot_pose.position = Point(x=0.0, y=0.0, z=0.0)
        req.robot_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # 同步调用服务（关键修改在这里！）
        future = self.set_stcm_client.call_async(req)

        # 等待服务返回（同步阻塞，但简单可靠）
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"楼层 '{floor_key_str}' 地图加载成功.")
            # 等待3秒（可以用 time.sleep，因为已经是同步逻辑）
            import time
            time.sleep(3.0)
            return True
        else:
            self.get_logger().error("服务调用失败或超时")
            return False
        

    def echo_info(self, info):
        if self.verbose:
            self.get_logger().info(info)

    def deliver_image(self, color_image, depth_data):
        img_msg = Camera()
        img_msg.size = list(color_image.shape)
        size = img_msg.size[0] * img_msg.size[1] * img_msg.size[2]
        img_msg.color = color_image.reshape(size).tolist()
        depth_npy = depth_data.astype(np.uint16)
        img_msg.depth = depth_npy.flatten().tolist()
        self.pub_image.publish(img_msg)

    def load_pose_config(self):
        config_dir = os.path.abspath(os.path.join(script_dir, '..', 'config'))
        config_path = os.path.join(config_dir, 'coffee_positions.json')
        default_config = {
            "put_in_pocket": {
                "approach": {"pose": None, "gripper": None},
                "drop": {"pose": None, "gripper": None},
                "retreat": {"pose": None, "gripper": None}
            },
            "elevator_observation_ready": {"pose": None, "gripper": None},
            "table_ready_pose": {"pose": None, "gripper": None}
        }
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().warning(f'Failed to load pose config: {e}, using defaults.')
            data = default_config
        return {**default_config, **data}

    def switch_map(self, floor_num: int):
        """切换到指定楼层的地图"""
        map_name = f"floor_{floor_num}.slam"
        self.echo_info(f'=> Switching map to: {map_name}')
        # 假设 SlamwareInterface 提供了 load_map 方法
        success = self.mobile_base.load_map(map_name)  # 需要底层支持
        if not success:
            self.get_logger().error(f'Failed to load map for floor {floor_num}')
            return False
        time.sleep(2)  # 等待地图加载
        return True

    def relocalize_on_floor(self, initial_pose=None):
        """在当前楼层执行重定位"""
        self.echo_info("=> Performing relocalization...")
        # 可传入初始猜测位姿加快收敛
        success = self.mobile_base.relocalize(initial_pose)  # 假设接口存在
        if not success:
            self.get_logger().warn("Relocalization failed, retrying...")
            time.sleep(1)
            success = self.mobile_base.relocalize(initial_pose)
        return success

    def navigate_to_waypoint(self, pose):
        """导航至预设路点"""
        self.echo_info(f'=> Navigating to waypoint: {pose} on floor {self.current_floor}')
        return self.mobile_base.nav_to_pose(pose)

    def detect_and_press_elevator_button(self, target_floor: int):
        """识别并按压电梯按钮"""
        self.echo_info(f'=> Pressing elevator button for floor {target_floor}')

        self.go_to_elevator_observation()
        time.sleep(1)
        color_image, _, depth_data = self.get_observation(is_button=True)
        self.tcp2base = self.robot.get_RT_matrix()
        self.tcp2base[:3, 3] /= 1000.0

        cv2.imwrite("rgb.png", color_image)
        np.save("dep.npy", depth_data)

        if target_floor > self.current_floor:
            pose, card_pose, _ = self.button_yolo.run_yolo(8)  # up button
            self.estimate_button_callback(pose)
            self.press_button(x_bias=-70, y_bias=130, z_bias=-40)
        else:
            pose, _, _ = self.button_yolo.run_yolo(11, need_card=False)  # down button
            self.estimate_button_up_callback(pose)
            self.press_button_up(x_bias=-70, y_bias=130, z_bias=-40)

    def wait_for_target_floor(self, target_floor: int):
        """等待到达目标楼层"""
        self.echo_info(f'=> Waiting for arrival at floor {target_floor}')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.get_floor and target_floor == self.current_floor:
                self.echo_info('Arrived at target floor!')
                break
            time.sleep(1)

    def go_to_elevator_observation(self):
        pose_entry = self._move_pose_from_config('elevator_observation_ready', 'elevator observation ready pose')
        if pose_entry is None:
            try:
                self.robot.moveJ(self.obs_joint_button)
            except Exception as e:
                self.get_logger().error(f'Move failed: {e}')
                return False
        return True

    def get_observation(self, is_button=False):
        joints = self.obs_joint_button if is_button else self.obs_joint
        self.robot.moveJ(joints)
        self.echo_info('- Taking photo...')
        frames = self.camera.wait_for_frames()
        aligned_frames = self.align.process(frames)
        time.sleep(0.5)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None, None

        depth_data = np.asanyarray(depth_frame.get_data(), dtype="float16")
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_data, alpha=0.03), cv2.COLORMAP_JET)
        self.echo_info('- Photo taken!')
        return color_image, depth_colormap, depth_data

    def observe_floor(self, msg):
        floors = np.array(msg.pose).reshape(-1).tolist()
        if self.target_floor in floors:
            self.get_floor = True

    def estimate_obj_callback(self, msg):
        obj2eye = np.array(msg.pose).reshape(4, 4)
        eye2tcp = np.loadtxt("/home/yofo/robotics/eye_hand_calib/data/eye2tcp_matrix.txt")
        eye2tcp[:3, 3] /= 1000
        obj2tcp = eye2tcp @ obj2eye
        self.obj_pose = self.tcp2base @ obj2tcp

    def estimate_button_callback(self, msg):
        obj2eye = np.array(msg.pose).reshape(4, 4)
        eye2tcp = np.loadtxt("/home/yofo/robotics/eye_hand_calib/data/eye2tcp_matrix.txt")
        eye2tcp[:3, 3] /= 1000
        obj2tcp = eye2tcp @ obj2eye
        self.button_pose = self.tcp2base @ obj2tcp

    def estimate_button_up_callback(self, msg):
        self.estimate_button_callback(msg)  # 复用逻辑

    def press_button(self, x_bias=0, y_bias=0, z_bias=0):
        if self.button_pose is None:
            self.get_logger().error('No button pose!')
            return
        trans = self.button_pose[:3, 3] * 1000
        rot = rotation_matrix_to_euler(self.button_pose[:3, :3])
        target = [*trans + [x_bias, y_bias, z_bias], *rot[:3]]  # 简化处理
        self.robot.moveJ_pose(target)
        self.go_to_elevator_observation_after_press()

    def press_button_up(self, x_bias=0, y_bias=0, z_bias=0):
        self.press_button(x_bias, y_bias, z_bias)

    def go_to_elevator_observation_after_press(self):
        self.go_to_elevator_observation()

    def grasp_obj(self):
        if self.obj_pose is None:
            self.get_logger().error('No object pose!')
            return
        T = self.obj_pose[:3, 3] * 1000
        R = rotation_matrix_to_euler(self.obj_pose[:3, :3])
        target = [T[0], T[1], T[2], 0.81, -17.5, 0.87]
        approach = [target[0] - 100, target[1], target[2], *target[3:]]
        self.robot.moveJ(self.obs_joint)
        self.gripper.move_to(1000)
        self.robot.moveJ_pose(approach)
        self.robot.moveJ_pose(target)
        self.gripper.move_to(0)
        self.robot.moveJ(self.hold_joint)
        self.object_state = 'held'

    def place_at_fixed_height(self, height):
        if self.operate_pose is None:
            return
        h = height * 1000 - 650
        drop = [self.operate_pose[0], self.operate_pose[1], h, *self.operate_pose[3:]]
        retreat = [drop[0] - 100, *drop[1:]]
        self.robot.moveJ_pose(drop)
        self.gripper.move_to(1000)
        time.sleep(1)
        self.robot.moveJ_pose(retreat)
        self.robot.moveJ(self.obs_joint)
        self.object_state = 'empty'

    def store_object_in_pocket(self):
        self._move_pose_from_config('put_in_pocket', 'pocket approach', 'approach')
        self._move_pose_from_config('put_in_pocket', 'pocket drop', 'drop')
        self._move_pose_from_config('put_in_pocket', 'pocket retreat', 'retreat')
        self.object_state = 'stowed'

    def retrieve_object_from_pocket(self):
        self._move_pose_from_config('out_of_pocket', 'pocket approach', 'approach')
        self._move_pose_from_config('out_of_pocket', 'pocket grasp', 'grasp')
        self._move_pose_from_config('out_of_pocket', 'pocket retreat', 'retreat')
        self.object_state = 'held'

    def instruction(self):
        print(
            "Options:\n"
            "0: Pick table | 1: Place table | 2: Print mobile pose\n"
            "3: Grasp cup | 4: Place cup | 5: Print arm pose\n"
            "B: Pocket store | O: Retrieve | 6: Close gripper | 7: Open gripper\n"
            "9: Press button | T: Reset grasp | A: Rec loc | Q: Exit\n"
            "Enter command >> "
        )

    # =====================================================================================
    # ✅ 版本一：交互式 run() —— 键盘控制
    # =====================================================================================
    def run_interactive(self):
        # self.instruction()


        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # if kbhit():
            key = getch()

            # import msvcrt
            # if msvcrt.kbhit():
            #     key = msvcrt.getch().decode('utf-8')
            ##识别键盘字符
            #read one key until no more input
            # key = getch() if os.name != 'nt' else msvcrt.getch().decode('utf-8') if msvcrt.kbhit() else ''
            # if not key:
            #     continue
            # key_upper = key.upper()


            #--------------一楼----------------------
            ##字符为0：一楼重定位, left zhuzi qian 2.5 ge zhuantou
            if key == '0':
                re=self._load_map_for_floor('1')
                self.mobile_base.rec_loc(-5.00, 0.00, 8.00, -8.00) 
            ##字符为1: 导航到一楼电梯口
            elif key == '1':
                #20.474182, -32.363243, 0.000000
                #(0.000000, 0.000000, -0.718337, 0.695695)
                pose=Pose(position=Point(x=1.69, y=-4.08, z=0.03),orientation=Quaternion(x=0.000000, y=0.000000, z=-0.718337, w=0.695695))
                self.navigate_to_waypoint(pose)
            elif key == '2':
                pose=Pose(position=Point(x=-3.22, y=-4.52, z=-3.14),orientation=Quaternion(x=0.000000, y=0.000000, z=-0.718337, w=0.695695))
                self.navigate_to_waypoint(pose)
            # ##字符为2: 导航进到一楼电梯里
            # elif key == '3':
            #     pose=Pose(position=Point(x=-21.726056, y=-8.981730, z=0.000000),orientation=Quaternion(x=0.000000, y=0.000000, z=0.059014, w=0.998257))
            #     self.navigate_to_waypoint(pose)
            # ##字符为3: 导航出一楼电梯
            # elif key == '4':
            #     #Position: (-19.039021, -8.339689, 0.000000)
            #     #(0.000000, 0.000000, 0.038292, 0.999267)
            #     pose=Pose(position=Point(x=-19.039021, y=-8.339689, z=0.000000),orientation=Quaternion(x=0.000000, y=0.000000, z=0.038292, w=0.999267))
            #     self.navigate_to_waypoint(pose)
            # ##字符为4: 导航到一个位置
            # elif key == '5':
            #     pose=Pose(position=Point(x=-8.00, y=-6.97, z=-0.41),orientation=Quaternion(x=0.000000, y=0.000000, z=0.732611, w=0.680648))
            #     self.navigate_to_waypoint(pose)

            #--------------三楼----------------------
            # if key == '4':
            #     pose=Pose(position=Point(x=-9.949331, y=19.789711, z=0.000000),orientation=Quaternion(x=0.000000, y=0.000000, z=-0.704983, w=0.709224))
            #     self.navigate_to_waypoint(pose)
            # elif key == '5':  #right zhuzi , dianti qian di 2 ge gezi
            #     re=self._load_map_for_floor('3')
            #     self.mobile_base.rec_loc(-15.00, 25.00, 10.00,-10.00) 
            # elif key == '6':
            #     #Position: (-26.974116, 6.861987, 0.000000)
            #     #Orientation: (0.000000, 0.000000, -0.564007, 0.825770)
            #     pose=Pose(position=Point(x=-26.974116, y=6.861987, z=0.000000),orientation=Quaternion(x=0.000000, y=0.000000, z=-0.564007, w=0.825770))
            #     self.navigate_to_waypoint(pose)



            # if key == '0':
            #     pose=Pose(position=Point(x=-8.00, y=-6.97, z=-0.41),orientation=Quaternion(x=0.000000, y=0.000000, z=0.732611, w=0.680648))
            #     self.navigate_to_waypoint(pose)


            

            # elif key == '1':
            #     self.navigate_to_waypoint('place_table')
            # elif key == '2':
            #     print("Base pose:", self.mobile_base.get_current_pose())
            # elif key == '3':
            #     self.grasp_obj()
            # elif key == '4':
            #     self.place_at_fixed_height(0.75)
            # elif key == '5':
            #     print("Arm pose:", self.robot.get_RT_matrix())
            # elif key_upper == 'B':
            #     self.store_object_in_pocket()
            # elif key_upper == 'O':
            #     self.retrieve_object_from_pocket()
            # elif key == '6':
            #     self.gripper.move_to(0)
            # elif key == '7':
            #     self.gripper.move_to(1000)
            # elif key == '9':
            #     target = int(input("Target floor? "))
            #     self.detect_and_press_elevator_button(target)
            #     self.wait_for_target_floor(target)
            #     self.current_floor = target
            #     self.switch_map(target)
            #     self.relocalize_on_floor()
            # elif key_upper == 'T':
            #     self.reset_grasp_pose()
            # elif key_upper == 'A':
            #     self.mobile_base.rec_loc(5.3, -5.36, 9.5, -17)
            # elif key_upper == 'Q':
            #     break
            # else:
            #     print('Invalid input.')

            # self.instruction()


    # =====================================================================================
    # ✅ 版本二：自主式 run() —— 由 launch 文件驱动
    # =====================================================================================
    def run_autonomous(self):
        self.echo_info(f"Starting autonomous navigation from floor {self.start_floor} to {self.target_floor}")

        current_floor = self.start_floor
        self.current_floor = current_floor

        # 执行预定义的导航序列
        for step_str in self.nav_sequence:
            try:
                step = json.loads(step_str.replace("'", "\""))
            except Exception as e:
                self.get_logger().error(f"Invalid nav step: {step_str}, error: {e}")
                continue

            floor = step.get("floor")
            waypoint = step.get("waypoint")
            action = step.get("action")  # grasp, place, press_button, etc.

            # 切换地图与重定位
            if floor != current_floor:
                self.switch_map(floor)
                self.relocalize_on_floor()
                self.current_floor = floor
                current_floor = floor

            # 导航到路点
            if waypoint:
                self.navigate_to_waypoint(waypoint)

            # 执行动作
            if action == "press_elevator":
                self.detect_and_press_elevator_button(self.target_floor)
                self.wait_for_target_floor(self.target_floor)
            elif action == "grasp":
                self.grasp_obj()
            elif action == "place":
                self.place_at_fixed_height(0.75)
            elif action == "store_pocket":
                self.store_object_in_pocket()
            elif action == "retrieve_pocket":
                self.retrieve_object_from_pocket()

        self.echo_info("Autonomous mission completed.")


    # =====================================================================================
    #  主控 run() 分发器
    # =====================================================================================
    def run(self):
        if self.mode == 'autonomous':
            self.run_autonomous()
        else:
            self.run_interactive()


def main(args=None):
    rclpy.init(args=args)
    node = MobileManipulatorInterface()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"Error during execution: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()