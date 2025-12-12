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
from typing import Any, Dict, List, Optional, Sequence
script_dir = os.path.dirname(os.path.realpath(__file__))
os.chdir(script_dir)
import time
try:
    from openai import OpenAI
except ImportError:
    OpenAI = None
    try:
        import openai  # type: ignore
    except ImportError:
        openai = None
else:
    openai = None

from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from geometry_msgs.msg import Pose as GeoPose, Point, Quaternion, Twist, PoseStamped
from grasp_topic.msg import Camera, Pose 
import pyrealsense2 as rs
from std_msgs.msg import String, Bool
from collections import deque

sys.path.append('/home/yofo/DucoCobotAPI') # debug：替换为实际路径
from DucoCobotAPI_py.SiasunRobot import SiasunRobotPythonInterface
from DucoCobotAPI_py.DHGrasperInterface import DHGrasperInterface
from DucoCobotAPI_py.SlamwareInterface_ros2 import SlamwareInterface # debug
sys.path.append('/home/yofo/ros2_ws_demo/src/grasp_topic') # debug：替换为实际路径
from utils.script_utils import euler_to_rotation_matrix, rotation_matrix_to_euler, getch

from control_host_model import Button_yolo

sys.path.append('/home/yofo/ros2_ws_demo/src/slamware_ros_sdk') # debug：替换为实际路径
# map
from slamware_ros_sdk.srv import SyncSetStcm
from rclpy.callback_groups import ReentrantCallbackGroup
import select
import tty
import termios


'''MobileManipulatorInterface类是一个手动交互模式, 通过按键来触发任务；
    它在控制主机上主动发布任务;
    采用临时订阅模式, 只在特定用户操作时（按下按键）需要数据
'''

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
     # 构造函数：各种硬件和ros节点的初始化
    def __init__(self, verbose=True):
        super().__init__('mobile_manipulator')

        #<----manipulation-------
        # 控制输出调试信息
        self.verbose = verbose
        self.echo_info('========== Mobile manipulator initializing ... ==========')

        self.echo_info('=> ROS 2 node initializing ...')
        self.pub_image = self.create_publisher(Camera, '/image', 10)
        self.echo_info('=> ROS 2 node initialized!')

        self.echo_info('=> Robot arm initializing ...')
        self.robot = SiasunRobotPythonInterface()
        # self.obs_joint = [-21.57, 4.97, 109.53, 94.10, 79.46, -5.09] # 原抓取任务的观察位姿
        self.obs_joint = [-13.58, -21.84, 127.44, 99.98, 82.59, -5.08] # 原抓取任务的观察位姿
        # self.obs_joint = [172.75, 15.79, -80.59, 196.56, -90.81, -0.63] # 跨楼层抓取的观察位姿

        # self.obs_joint = [0.0, -13.97, 79.06, 144.27, 90, 0.0]

        # self.hold_joint = [0.0, -13.97, 79.06, 115.16, 90, 0.0]
        self.hold_joint = [-11.95, -9.83, 97.83, 91.42, 84, -5.09] # 原抓取任务的握持位姿
        # self.hold_joint = [175.68, 11.09, -94.23, 201.43, -90.00, -0.63] # 跨楼层抓取的观察位姿
        # self.obs_joint_button = [0, -44.5, 113.36, 113.27, 115, 0.]
        self.obs_joint_button = [150, 23.5, -121.36, 90.27, 101, -177.]
        self.restore_pose = [150.0, 45.11,-92.87, -5.6, 116.4, -177.19]
        self.echo_info('=> Robot arm initialized!')

        # 初始化 tcp2base，防止回调里找不到属性
        self.tcp2base = self.robot.get_RT_matrix()
        self.tcp2base[:3, 3] = self.tcp2base[:3, 3] / 1000

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


        #<------navigation------
 # self.echo_info('=> Mobile base initializing ...') # debug
        # self.mobile_base = SlamwareInterface()
        # self.echo_info('=> Mobile base initialized.')

        # self.obj_pose = None
        # self.operate_pose = None
        # self.button_pose = None
        # self.card_pose = None
        # self.get_floor = False
        # # Track whether the gripper currently holds an object for pocket logic.
        # self.object_state = 'empty'
        # # Load reusable poses for elevator and pocket operations.
        # self.pose_config = self.load_pose_config()
        # self.floor_maps_dict = {'1':"/home/yofo/ros2_ws_demo/src/slamware_ros_sdk/maps/try2.stcm",
        #                         '2':"/home/yofo/ros2_ws_demo/src/slamware_ros_sdk/maps/floor2.stcm",
        #                         '3':"/home/yofo/ros2_ws_demo/src/slamware_ros_sdk/maps/floor3.stcm"}
        # self.button_yolo = Button_yolo()
        
        # # 创建订阅者
        # self.obj_pose_sub = self.create_subscription(
        #     Pose, 
        #     '/pose', 
        #     self.estimate_obj_callback, 
        #     10)
            
        # self.button_pose_sub = self.create_subscription(
        #     Pose, 
        #     '/button_pose', 
        #     self.estimate_button_callback, 
        #     10)
            
        # self.floor_sub = self.create_subscription(
        #     Pose, 
        #     '/floor', 
        #     self.observe_floor, 
        #     10)
            
        # self.echo_info('========== Mobile manipulator initialized! ==========')
        self.echo_info('========== Mobile manipulator initializing ... ==========')
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
        self.current_floor = self.start_floor 
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
        self.echo_info('- Image delivering ...')
        img_msg = Camera()
        img_msg.size = list(color_image.shape)
        size = img_msg.size[0]*img_msg.size[1]*img_msg.size[2]
        img_msg.color = color_image.reshape(size).tolist()
        depth_npy = depth_data.astype(np.uint16)
        img_msg.depth = depth_npy.flatten().tolist()
        self.pub_image.publish(img_msg)
        self.echo_info('- Image delivered!')
        
    
        
    def store_object_in_pocket(self):
        """Execute the three-step pocket drop sequence with configurable gripper."""
        self.echo_info('- Stowing object into pocket ...')
        self.object_state = 'held'
        if self.object_state != 'held':
            self.get_logger().warning(f'Object state is "{self.object_state}", expected "held" before stowing.')

        if self._move_pose_from_config('put_in_pocket', 'pocket approach pose', subkey='approach') is None:
            return
        drop_entry = self._move_pose_from_config('put_in_pocket', 'pocket drop pose', subkey='approach')
        drop_entry = self._move_pose_from_config('put_in_pocket', 'pocket drop pose', subkey='drop')
        if drop_entry is None:
            return

        if drop_entry.get('gripper') is None:
            # Default to releasing if the config does not drive the gripper here.
            self.gripper.move_to(1000)
            time.sleep(0.3)

        self._move_pose_from_config('put_in_pocket', 'pocket retreat pose', subkey='retreat')
        self.object_state = 'stowed'
        self.operate_pose = None
        # self.go_to_elevator_observation()
        time.sleep(1)
        # drop_entry = self._move_pose_from_config('put_in_pocket', 'pocket approach pose', subkey='approach')
        self.echo_info('- Pocket stow finished.')    
        
    
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

    def _pose_to_text(self, pose: Optional[GeoPose]) -> str:
        if pose is None:
            return 'None'
        pos = pose.position
        ori = pose.orientation
        return f"pos=({pos.x:.2f},{pos.y:.2f},{pos.z:.2f}),ori=({ori.x:.2f},{ori.y:.2f},{ori.z:.2f},{ori.w:.2f})"

    def _normalize_pose_input(self, pose_data: Any) -> Optional[GeoPose]:
        if pose_data is None:
            return None
        if isinstance(pose_data, GeoPose):
            return pose_data
        if isinstance(pose_data, PoseStamped):
            return pose_data.pose
        if isinstance(pose_data, dict):
            if 'pose' in pose_data:
                return self._normalize_pose_input(pose_data['pose'])
            pos = pose_data.get('position', {})
            ori = pose_data.get('orientation', {})
            try:
                return GeoPose(
                    position=Point(float(pos.get('x', 0.0)), float(pos.get('y', 0.0)), float(pos.get('z', 0.0))),
                    orientation=Quaternion(
                        float(ori.get('x', 0.0)),
                        float(ori.get('y', 0.0)),
                        float(ori.get('z', 0.0)),
                        float(ori.get('w', 1.0))
                    )
                )
            except (TypeError, ValueError):
                return None
        if isinstance(pose_data, Sequence) and len(pose_data) == 7:
            try:
                return GeoPose(
                    position=Point(float(pose_data[0]), float(pose_data[1]), float(pose_data[2])),
                    orientation=Quaternion(
                        float(pose_data[3]), float(pose_data[4]), float(pose_data[5]), float(pose_data[6])
                    )
                )
            except (TypeError, ValueError):
                return None
        return None


    def _call_openai_planner(self, instruction: str, skills: List[Dict[str, Any]], model: str) -> Optional[Dict[str, Any]]:
        # from dotenv import load_dotenv
        # load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), ".env"))
        # api_key = os.environ.get('OPENAI_API_KEY')

        # 加载并打印加载状态
        # load_status = load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), ".env"))
        # print(f".env 文件是否加载成功：{load_status}")  # True=加载成功，False=文件不存在/无法读取

        # api_key = os.environ.get('OPENAI_API_KEY')
        # print(f"读取到的 API Key：{api_key}")  # 若打印 None，说明加载/格式有问题；若打印 sk-xxx，说明加载成功

        # if not api_key:
        #     self.get_logger().error('OPENAI_API_KEY is not set.')
        #     return None

        # print(skills)
        skill_lines = '\n'.join(
            f"- id:{s['id']} type:{s['type']} op:{s.get('operation')} pose:{s.get('pose')} {s.get('description')}"
            for s in skills
        )
        user_prompt = (
            f"Instruction: {instruction}\n"
            "Using only the skills below, return JSON {\"actions\": [...]}.\n"
            "Each action must have \"type\" (navigate|manipulate) and \"target\" (skill id).\n"
            "For manipulate include \"operation\" (pick/place/store) and optional \"height\" in meters.\n"
            f"Skills:\n{skill_lines}"
        )
        sys_prompt = 'Plan sequential steps for a mobile manipulator and answer with JSON only.'
        try:
            # if OpenAI is not None:
            #     client = OpenAI(api_key=api_key)
            #     resp = client.responses.create(
            #         model=model,
            #         input=[{'role': 'system', 'content': sys_prompt},
            #                {'role': 'user', 'content': user_prompt}],
            #         temperature=0.2,
            #     )
            #     text = ''.join(
            #         block.text
            #         for item in getattr(resp, 'output', [])
            #         for block in getattr(item, 'content', [])
            #         if getattr(block, 'type', None) == 'text'
            #     )
            # elif openai is not None:
            #     openai.api_key = api_key
            #     completion = openai.ChatCompletion.create(
            #         model=model,
            #         temperature=0.2,
            #         messages=[{'role': 'system', 'content': sys_prompt},
            #                   {'role': 'user', 'content': user_prompt}]
            #     )
            #     text = completion['choices'][0]['message']['content']
            # else:
            #     self.get_logger().error('OpenAI SDK not installed.')
            #     return None
            # client = OpenAI(
            #     api_key="", 
            #     # 以下为新加坡地域base_url，若使用北京地域的模型，需将base_url替换为https://dashscope.aliyuncs.com/compatible-mode/v1
            #     # base_url=""
            #     base_url=""
            # )
            # completion = client.chat.completions.create(
            #     model="qwen2.5-1.5b-instruct",
            #     messages=[{"role": "user", "content": "你是谁？"}]
            # )

            client = OpenAI(
                # openai系列的sdk，包括langchain，都需要这个/v1的后缀
                base_url='',
                api_key='',
            )
            chat_completion = client.chat.completions.create(
                messages=[
                                        {
                        "role":"system",
                        "content":sys_prompt
                    },
                    {
                        "role": "user",
                        "content":user_prompt
                    }
                ],
                model="gpt-3.5-turbo", # 如果是其他兼容模型，比如deepseek，直接这里改模型名即可，其他都不用动
            )
            text = chat_completion.choices[0].message.content #['choices'] #[0]['message']['content']
            print(text)

        except Exception as exc:
            self.get_logger().error(f'OpenAI call failed: {exc}')
            return None
        text = text.strip()
        start, end = text.find('{'), text.rfind('}')
        json_text = text[start:end + 1] if start != -1 and end != -1 else text
        try:
            return json.loads(json_text)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Planner output is not JSON: {exc}')
            self.get_logger().error(json_text)
            return None

    def _perform_pick_sequence(self) -> bool:
        try:
            color_image, _, depth_data = self.get_observation()
            self.tcp2base = self.robot.get_RT_matrix()
            self.tcp2base[:3, 3] = self.tcp2base[:3, 3] / 1000
            self.deliver_image(color_image, depth_data)
            future = rclpy.task.Future()

            def callback(msg):
                self.estimate_obj_callback(msg)
                if not future.done():
                    future.set_result(True)

            subscription = self.create_subscription(Pose, '/pose', callback, 10)
            rclpy.spin_until_future_complete(self, future)
            self.destroy_subscription(subscription)
            self.grasp_obj()
            return True
        except Exception as exc:
            self.get_logger().error(f'Pick pipeline failed: {exc}')
            return False

    def _perform_place_sequence(self, height: float) -> bool:
        try:
            self.place_at_fixed_height(height)
            return True
        except Exception as exc:
            self.get_logger().error(f'Place pipeline failed: {exc}')
            return False

    def execute_instruction_with_gpt(
        self,
        instruction: str,
        knowledge_base: Sequence[Dict[str, Any]],
        model: str = 'gpt-4o-mini',
        default_place_height: float = 0.75,
    ) -> bool:

        skills = [
            {
                "id": "yellow_table_nav",
                "type": "navigate",
                "pose": 
                GeoPose(position=Point(x=-1.301, y=0.136, z=0.0),orientation=Quaternion(x=0.000000, y=0.000000, z=0.711, w=0.703)),
                "description": "Base pose facing the yellow table."
            },
            {
                "id": "yellow_table_pick",
                "type": "manipulate",
                "operation": "pick",
                "description": "Use vision grasp pipeline on the yellow table cup."
            },
            {
                "id": "white_table_nav",
                "type": "navigate",
                "pose": GeoPose(position=Point(x=0.750, y=-0.843, z=0.0),orientation=Quaternion(x=0.000000, y=0.000000, z=0.684, w=0.729)),
                "description": "Base pose near the white table."
            },
            {
                "id": "white_table_place",
                "type": "manipulate",
                "operation": "place",
                "height": 0.78,
                "description": "Place the cup onto the white table (0.78 m)."
            }
        ]

        plan = self._call_openai_planner(instruction, skills, model)
        if not plan:
            return False

        print(plan)
        # return

        # plan={
            # "actions": [
                # {
                # "type": "navigate",
                # "target": "white_table_nav",
                # "reason": "Drive to the white table to place the cup."
                # },
                # {
                # "type": "manipulate",
                # "target": "white_table_place",
                # "operation": "pick",
                # "reason": "Pick up the cup from the yellow table."
                # },
                # {
                # "type": "navigate",
                # "target": "yellow_table_nav",
                # "reason": "Move to the yellow table with the cup."
                # },
                # {
                # "type": "manipulate",
                # "target": "yellow_table_pick",
        #         "operation": "place",
        #         "height": 0.78,
        #         "reason": "Place the cup onto the white table surface."
        #         }
        #     ]
        # }


        actions = plan.get('actions') or plan.get('steps')

        #log
        if not isinstance(actions, list):
            self.get_logger().error('Planner output missing actions list.')
            return False
        

        skills_map = {s['id']: s for s in skills}
        for idx, action in enumerate(actions, 1):
            action_type = str(action.get('type') or '').lower()
            target_id = action.get('target')
            self.echo_info(f'GPT step {idx}: {action}')
            
            #导航
            if action_type == 'navigate':
                pose = None
                if target_id and target_id in skills_map:
                    pose = skills_map[target_id].get('pose')  #获取导航位姿
                pose = pose or self._normalize_pose_input(action.get('pose')) #还是pose
                # if pose is None or not self.navigate_to_waypoint(pose):
                #     self.get_logger().error(f'Navigation failed at step {idx}')
                #     return False
                self.navigate_to_waypoint(pose)
            
            elif action_type == 'manipulate':
                skill = skills_map.get(target_id or '')
                operation = str(action.get('operation') or (skill or {}).get('operation') or 'pick').lower()
                
                #高度，place
                height = action.get('height')
                if height is None and skill is not None:
                    height = skill.get('height')
                height = float(height) if height is not None else default_place_height
                
                #pick
                if operation in ('pick', 'grasp'):
                    if not self._perform_pick_sequence():
                        return False
                #place
                elif operation in ('place', 'drop'):
                    if not self._perform_place_sequence(height):
                        return False
                # elif operation in ('store', 'stow'):
                #     self.store_object_in_pocket()
                else:
                    self.get_logger().error(f'Unknown manipulation op "{operation}"')
                    return False
            else:
                self.get_logger().error(f'Unknown action type "{action_type}"')
                return False
            time.sleep(2)

        return True

    def load_pose_config(self):
        """Read pose/gripper presets for pocket and elevator routines."""
        config_dir = os.path.abspath(os.path.join(script_dir, '..', 'config'))
        config_path = os.path.join(config_dir, 'coffee_positions.json')
        self.pose_config_path = config_path
        default_config = {
            "put_in_pocket": {
                "approach": {"pose": [336.10, -132.51, 153.34, -32.46, 20.78, 7.19], "gripper": 200},
                "drop": {"pose": [258.26, -32.44, 13.89, -27.63, 27.86, 27.27], "gripper": 800},
                # "mid": {"pose": [331.7,-22.9,375.962,-15.06,54.91,33.27], "gripper": 300},
                "retreat": {"pose": [302.052, -55.04, 172.80, -27.20, 26.31, 16.68], "gripper": 1000}
            },
            "elevator_observation_ready": {"pose": None, "gripper": None},
            "elevator_observation_after_press": {"pose": None, "gripper": None},
            "out_of_pocket": {
                "approach": {"pose": [278.0, 115.911, -42.634, 6.99, 70.34, 76.55], "gripper": None},
                "grasp": {"pose": [278.0, 115.911, -42.634, 6.99, 70.34, 76.55], "gripper": 300},
                # "grasp": {"pose": [279.8, 118.17, -47.48, 3.87, 69.56, 74.22], "gripper": [600]},
                "retreat": {"pose": [278.0, 115.911, 160.634, 6.99, 70.34, 76.55], "gripper": 300}
                # "retreat": {"pose": [359.72, 104.07, 214.57, -27.5, 33.16, 40.5], "gripper": 400}
            },
            "table_ready_pose": {"pose": [655.09,-156.19,135.11,-10.92,70.11,19.22], "gripper": 1000}
        }
        try:
            with open(config_path, 'r', encoding='utf-8') as cfg_file:
                data = json.load(cfg_file)
        except FileNotFoundError:
            self.get_logger().warning(f'Pose config not found at {config_path}. Using defaults.')
            data = default_config
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Failed to parse pose config ({config_path}): {exc}')
            data = default_config

        for key, value in default_config.items():
            if key not in data or data[key] is None:
                data[key] = value
            elif isinstance(value, dict):
                if not isinstance(data[key], dict):
                    data[key] = value
                else:
                    for sub_key, sub_value in value.items():
                        data[key].setdefault(sub_key, sub_value)
        return data

    def _normalize_pose_entry(self, entry, label):
        """Convert json entries into motion commands plus optional gripper target."""
        if entry is None:
            self.get_logger().error(f'Pose config missing for "{label}".')
            return None

        move_type = 'pose'
        gripper_value = None
        values_raw = entry
        if isinstance(entry, dict):
            if 'gripper' in entry and entry['gripper'] is not None:
                try:
                    gripper_value = float(entry['gripper'])
                except (TypeError, ValueError):
                    self.get_logger().error(f'Gripper target for "{label}" must be numeric.')
                    return None
            if 'pose' in entry and entry['pose'] is not None:
                values_raw = entry['pose']
                move_type = 'pose'
            elif 'joints' in entry and entry['joints'] is not None:
                values_raw = entry['joints']
                move_type = 'joint'
            else:
                self.get_logger().error(f'Pose config for "{label}" must include "pose" or "joints".')
                return None

        if values_raw is None:
            self.get_logger().error(f'Pose config missing numeric data for "{label}".')
            return None

        if not isinstance(values_raw, (list, tuple)) or len(values_raw) != 6:
            self.get_logger().error(f'Pose config for "{label}" must be a list of 6 numeric values.')
            return None

        try:
            values = [float(val) for val in values_raw]
        except (TypeError, ValueError):
            self.get_logger().error(f'Pose config for "{label}" contains non-numeric values.')
            return None

        return {'type': move_type, 'values': values, 'gripper': gripper_value}

    def _fetch_pose_entry(self, key, subkey=None):
        entry = self.pose_config.get(key)
        label = key if subkey is None else f'{key}.{subkey}'
        if subkey is not None:
            if not isinstance(entry, dict):
                self.get_logger().error(f'Pose config for "{key}" must be a dict to access "{subkey}".')
                return None
            entry = entry.get(subkey)
        return self._normalize_pose_entry(entry, label)

    def _move_pose_from_config(self, key, description, subkey=None):
        """Execute a stored pose/joint move and optionally command the gripper."""
        pose_entry = self._fetch_pose_entry(key, subkey=subkey)
        if pose_entry is None:
            return None
        self.echo_info(f'-- moving to {description}')
        try:
            if pose_entry['type'] == 'joint':
                self.robot.moveJ(pose_entry['values'])
            else:
                self.robot.moveJ_pose(pose_entry['values'])
            if pose_entry.get('gripper') is not None:
                self.gripper.move_to(int(pose_entry['gripper']))
        except Exception as exc:
            self.get_logger().error(f'Failed to move to {description}: {exc}')
            return None
        return pose_entry

    def go_to_elevator_observation(self):
        """Move the arm to the pre-defined elevator observation pose."""
        pose_entry = self._move_pose_from_config('elevator_observation_ready', 'elevator observation ready pose')
        if pose_entry is None:
            self.echo_info('-- fallback to default elevator observation joint pose')
            try:
                self.robot.moveJ(self.obs_joint_button)
            except Exception as exc:
                self.get_logger().error(f'Fallback elevator observation move failed: {exc}')
                return False
            return False
        return True

    def go_to_elevator_observation_after_press(self):
        """Return to the post-button observation pose (or fallback to the ready pose)."""
        pose_entry = self._move_pose_from_config('elevator_observation_after_press', 'post-button observation pose')
        if pose_entry is None:
            return self.go_to_elevator_observation()
        return True

    def go_to_table_ready_pose(self):
        """Bring the arm back to the table placement ready pose."""
        pose_entry = self._move_pose_from_config('table_ready_pose', 'table ready pose')
        if pose_entry is None:
            self.echo_info('-- fallback to hold_joint for table ready pose')
            try:
                self.robot.moveJ(self.hold_joint)
                rt_matrix = self.robot.get_RT_matrix()
                translation = rt_matrix[:3, 3] * 1000
                rotation = rotation_matrix_to_euler(rt_matrix[:3, :3])
                self.operate_pose = [
                    float(translation[0]),
                    float(translation[1]),
                    float(translation[2]),
                    float(rotation[0]),
                    float(rotation[1]),
                    float(rotation[2])
                ]
            except Exception as exc:
                self.get_logger().warning(f'Unable to capture TCP pose after hold_joint fallback: {exc}')
                self.operate_pose = None
            return False

        if pose_entry['type'] == 'pose':
            self.operate_pose = pose_entry['values']
        else:
            try:
                rt_matrix = self.robot.get_RT_matrix()
                translation = rt_matrix[:3, 3] * 1000
                rotation = rotation_matrix_to_euler(rt_matrix[:3, :3])
                self.operate_pose = [
                    float(translation[0]),
                    float(translation[1]),
                    float(translation[2]),
                    float(rotation[0]),
                    float(rotation[1]),
                    float(rotation[2])
                ]
            except Exception as exc:
                self.get_logger().warning(f'Unable to capture TCP pose for table_ready_pose: {exc}')
                self.operate_pose = None
        return True


    def retrieve_object_from_pocket(self):
        """Execute the three-step pocket retrieval sequence with configurable gripper."""
        self.echo_info('- Retrieving object from pocket ...')
        if self.object_state not in ('stowed', 'unknown'):
            self.get_logger().warning(f'Object state is "{self.object_state}", expected "stowed" before retrieval.')

        approach_entry = self._move_pose_from_config('out_of_pocket', 'pocket approach pose', subkey='approach')
        if approach_entry is None:
            return

        if approach_entry.get('gripper') is None:
            # Ensure the gripper is open before reaching into the pocket if not specified.
            self.gripper.move_to(1000)

        grasp_entry = self._move_pose_from_config('out_of_pocket', 'pocket grasp pose', subkey='grasp')
        if grasp_entry is None:
            return

        if grasp_entry.get('gripper') is None:
            # Default grasp action if the config does not command it.
            self.gripper.move_to(0)
            time.sleep(0.3)

        self._move_pose_from_config('out_of_pocket', 'pocket retreat pose', subkey='retreat')
        self.object_state = 'held'
        self.go_to_table_ready_pose()
        self.echo_info('- Pocket retrieval finished.')

    '''利用订阅得到的pose抓取: 转换pose到6D --> 机械臂移动到观察姿态 --> 移动到catch_ready姿态 --> 
        移动到抓取姿态,夹爪闭合 --> 移动到持握姿态
    '''
    def grasp_obj(self, x_bias=26.08, y_bias=-18.985, z_bias=10.0):   
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
                        [-62.15, -57.52, 29.53]
        catch_ready_pose = [float(translation[0]) + x_bias - 100, float(translation[1]) + y_bias, float(translation[2]) + z_bias] + \
                        [-62.15, -57.52, 29.53]

        self.operate_pose = transformed_pose

        self.echo_info('- Grasp object at:')
        self.echo_info(str(transformed_pose))

        print("catch_ready_pose:", catch_ready_pose)
        ret = self.robot.moveJ_pose(catch_ready_pose)
        ret = self.robot.moveJ_pose(transformed_pose)

        self.gripper.move_to(100) # debug

        self.robot.moveJ(self.hold_joint)
        self.object_state = 'held'
        self.echo_info('- Grasping finished!')

    def reset_grasp_pose(self, x_bias=0, y_bias=0, z_bias=0):   
        '''
        bias: transform between estimated pose and actual TCP (mm)
        '''
        self.echo_info('- Grasping begin ...')
        self.echo_info('-- go to observation pose')
        self.robot.moveJ(self.obs_joint)

        print(f"Reset to obs joint{self.obs_joint}")

        self.echo_info('-- open gripper')
        self.gripper.move_to(1000) # debug

        print(f"Gripper open to {1000}")

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

        print("pose for obs_joint:", pose)
        print("pose before reset:", transformed_pose)
    
    def press_button_up(self, x_bias=-70, y_bias=130, z_bias=-40):
        '''
        bias: transform between estimated pose and actual TCP (mm)
        '''
        self.echo_info('- Press begin ...')
        self.echo_info('-- go to observation pose')
        # self.go_to_elevator_observation()
        self.echo_info('-- close gripper')
        self.gripper.move_to(1000)

        if self.button_pose is None:
            self.get_logger().error('ERROR: button_pose is None')
            return
        
        # if self.card_pose is None:
        #     self.get_logger().error('ERROR: card_pose is None')
        #     return

        pose = self.button_pose

        translation = pose[:3, 3]*1000

        print("translation", translation)


        button_pose = [float(translation[0]) + x_bias+15, float(translation[1]) + y_bias-65, float(translation[2]) + z_bias] + \
                        [-24.,18.,-5.]


        self.echo_info('- Press button at:')
        self.echo_info(str(button_pose))

        ret = self.robot.moveJ_pose(button_pose)

        # self.go_to_elevator_observation_after_press()
        
        self.echo_info('- Press finished!')

    def press_button(self, x_bias=-70, y_bias=130, z_bias=-40):
        '''
        bias: transform between estimated pose and actual TCP (mm)
        '''
        self.echo_info('- Press begin ...')
        self.echo_info('-- go to observation pose')
        # self.go_to_elevator_observation()
        
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
        
        # self.go_to_elevator_observation_after_press()
        
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
        self.object_state = 'empty'
        self.robot.moveJ_pose(post_drop_pose)
        self.robot.moveJ(self.obs_joint)
        self.echo_info('- Object placed!')

    # 处理接收到pose的回调函数：从相机坐标系到机械臂基座坐标系的转换
    def estimate_obj_callback(self, pose_msg):
        self.get_logger().info('Subscriber: obj pose received!')
        obj2eye = np.array(pose_msg.pose).reshape(4, 4)
        # eye2tcp
        eye2tcp_path = os.path.join("/home/yofo/robotics/eye_hand_calib/data_new", "eye2tcp_matrix.txt") # debug
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
        
    def estimate_button_up_callback(self, pose_msg):
        self.get_logger().info('Subscriber: button pose received!')
        obj2eye = pose_msg
        eye2tcp_path = os.path.join("/home/yofo/robotics/eye_hand_calib/data", "eye2tcp_matrix.txt")
        eye2tcp = np.loadtxt(eye2tcp_path)
        eye2tcp[:3, 3] = eye2tcp[:3, 3] / 1000 # convert to meters
        obj2tcp = eye2tcp @ obj2eye
        button_pose = self.tcp2base @ obj2tcp
        self.button_pose = button_pose



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
            # if is_button:
            #     if not self.go_to_elevator_observation():
            #         self.echo_info('-- using default elevator observation joint pose')
            # else:
            #     self.robot.moveJ(self.obs_joint)
            if is_button:
                self.robot.moveJ(self.obs_joint_button)
            else:
                self.robot.moveJ(self.obs_joint)
            # self.echo_info('- Photo taking ...')
            # Wait for frames and align them
            # frames = self.camera.wait_for_frames()
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

    def deliver_from_front_basket_to_table(self, table_height=0.75):
        """
        送货功能：导航到桌子 → 从身前篮子中抓杯子（用预设位姿）→ 放到桌子上。
        不依赖视觉，仅使用人工标定好的抓取位姿。
        桌子高度随便填的，后续会复用 place_at_fixed_height 函数。实际应用中，请根据实际情况调整桌子高度。
        """
        self.echo_info('========== Delivery: basket -> table ==========')

        # Step 1: 导航到桌子（机器人整体移动）
        try:
            self.echo_info(f'- Step 1: Navigating to table: {self.table_nav_name} ...')
            self.mobile_base.nav_to_obj(self.table_nav_name)
            time.sleep(2.0)  # zzzzzz
        except Exception as exc:
            self.get_logger().error(f'Navigation to table failed: {exc}')
            return False

        # Step 2: 机械臂复位
        try:
            self.echo_info('- Step 2: Move arm to table ready pose ...')
            self.go_to_table_ready_pose()
        except Exception as exc:
            self.get_logger().warning(f'Failed to move to table ready pose before basket grasp: {exc}')

        # Step 3: 从身前篮子抓杯子（固定相对位姿，不用视觉）
        self.echo_info('- Step 3: Grasp cup from front basket using fixed pose ...')

        # 先确保夹爪是打开的
        self.gripper.move_to(1000)
        time.sleep(0.8)

        grasp_pose = self.fixed_basket_grasp_pose  # [x, y, z, rx, ry, rz]，人工标定好的相对位姿

        # 预抓取位姿：在 z 方向（向上）先抬高一段，避免横向撞到篮子边缘
        pre_grasp_pose = [
            grasp_pose[0],
            grasp_pose[1],
            grasp_pose[2] + 80.0,  # 上方 80mm
            grasp_pose[3],
            grasp_pose[4],
            grasp_pose[5],
        ]

        try:
            # 3.1 到预抓取点
            self.echo_info(f'-- pre-grasp pose (basket): {pre_grasp_pose}')
            self.robot.moveJ_pose(pre_grasp_pose)

            # 3.2 垂直下探到真正抓取点
            self.echo_info(f'-- grasp pose (basket): {grasp_pose}')
            self.robot.moveJ_pose(grasp_pose)

            # 3.3 闭合夹爪抓住杯子
            self.gripper.move_to(0)
            time.sleep(0.5)

            # 3.4 抬回预抓取点，避免从篮子里拖擦出来
            self.robot.moveJ_pose(pre_grasp_pose)

            # 3.5 回到持握姿态，方便后续放置
            self.robot.moveJ(self.hold_joint)

            # 记录当前“操作姿态”，后续放桌子会复用姿态，只改高度
            self.operate_pose = grasp_pose.copy()
            self.object_state = 'held'

            self.echo_info('- Grasp from basket finished.')
        except Exception as exc:
            self.get_logger().error(f'Fixed basket grasp failed: {exc}')
            return False

        # Step 4: 放到身前桌子（使用已有 place_at_fixed_height）
        self.echo_info(f'- Step 4: Place cup onto table at height {table_height} m ...')
        try:
            self.place_at_fixed_height(table_height)
        except Exception as exc:
            self.get_logger().error(f'Place on table failed: {exc}')
            return False

        self.echo_info('========== Delivery task done: basket -> table ==========')
        return True
    
    def get_odom_once(self, timeout_sec=2.0):
        future = rclpy.task.Future()
        odom_data = {}

        def odom_callback(msg):
            if future.done():
                return
            odom_data["position"] = msg.pose.pose.position
            odom_data["orientation"] = msg.pose.pose.orientation
            future.set_result(True)

        odom_sub = self.create_subscription(
            Odometry,
            '/slamware_ros_sdk_server_node/odom',
            odom_callback,
            10
        )

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        self.destroy_subscription(odom_sub)

        if not future.done():
            self.get_logger().warning('No odom received within timeout')
            return None

        return odom_data["position"], odom_data["orientation"]

    # 打印出所有支持的功能
    def instruction(self):
        print(
            "Optional choice:\n"
            "-> Mobile base:\n"
            "---> 0: Go to pick tabel; 1: Go to place table; 2: Print Pose(odm).\n"
            "---> P: Set pick-tabel pose; L: Set place-table pose.\n"
            "-> Arm:\n"
            "---> 3: Grasp white cup; 4: Place cup; 5: Print Pose.\n"
            "---> B: Put object into pocket; O: Retrieve object from pocket.\n"
            "-> Gripper:\n"
            "---> 6: Close; 7: Open; 8: Print Pose.\n"
            "---> 9: Press button; T: Reset Grasp pose.\n"
            "-> Q: Exit.\n"
            "Please enter from keyboard (NO NEED TO ENTER) >>"
        )


    # def 
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
            key_upper = key.upper()
                
            if key == '0':
                self.echo_info('Operating: Navigating to pick table ...')
                self.mobile_base.nav_to_obj('pick_table') # debug
            elif key == '1':
                self.echo_info('Operating: Navigating to place table ...')
                self.mobile_base.nav_to_obj('place_table') # debug
            elif key == '2':
                odom = self.get_odom_once()
                if odom is not None:
                    pos, ori = odom
                    print(
                        f"Mobile base odom position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})"
                    )
                    print(
                        "Mobile base odom orientation: "
                        f"({ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f})"
                    )
            elif key.upper() == 'P':
                odom = self.get_odom_once()
                if odom is not None:
                    pos, ori = odom
                    updated_pose = GeoPose(
                        position=Point(x=pos.x, y=pos.y, z=pos.z),
                        orientation=Quaternion(x=ori.x, y=ori.y, z=ori.z, w=ori.w)
                    )
                    self.mobile_base.locations['pick_table'] = updated_pose
                    self.echo_info('Updated pick_table location from current odom pose')
            elif key.upper() == 'L':
                odom = self.get_odom_once()
                if odom is not None:
                    pos, ori = odom
                    updated_pose = GeoPose(
                        position=Point(x=pos.x, y=pos.y, z=pos.z),
                        orientation=Quaternion(x=ori.x, y=ori.y, z=ori.z, w=ori.w)
                    )
                    self.mobile_base.locations['place_table'] = updated_pose
                    self.echo_info('Updated place_table location from current odom pose')


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
            elif key_upper == 'B':
                self.echo_info('Operating: Stowing object into pocket ...')
                self.store_object_in_pocket()
            elif key_upper == 'O':
                self.echo_info('Operating: Retrieving object from pocket ...')
                self.retrieve_object_from_pocket()
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
                # self.go_to_elevator_observation()
                self.robot.moveJ(self.obs_joint_button)
                # time.sleep(4)
                color_image, depth_colormap, depth_data = self.get_observation(is_button=True)
                self.tcp2base = self.robot.get_RT_matrix()
                self.tcp2base[:3, 3] = self.tcp2base[:3, 3] / 1000
            
                cv2.imwrite("rgb.png", color_image)
                np.save("dep.npy", depth_data)
                
                pose, pose_card, boxcls = self.button_yolo.run_yolo(3)
                # print(pose)
                self.estimate_button_callback(pose,pose_card)

                self.press_button()

                self.robot.moveJ(self.restore_pose)

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
                
            elif key == 'u':
                self.echo_info('Operating: Press Up button ...')
                # self.go_to_elevator_observation()
                self.robot.moveJ(self.obs_joint_button)
                # time.sleep(4)
                color_image, depth_colormap, depth_data = self.get_observation(is_button=True)
                self.tcp2base = self.robot.get_RT_matrix()
                self.tcp2base[:3, 3] = self.tcp2base[:3, 3] / 1000
                print(color_image.shape)
                cv2.imwrite("rgb.png", color_image)
                np.save("dep.npy", depth_data)
                
                pose, boxcls = self.button_yolo.run_yolo(0, need_card = False)
                self.estimate_button_up_callback(pose)

                print("ready press")
                self.press_button_up()
                self.robot.moveJ(self.restore_pose)
                # self.get_in()
                
            elif key == 'o':
                self.echo_info('Observing: Waiting to get out the elevator...')
                self.go_to_elevator_observation()
                time.sleep(0.2)
                target_floor = 2
                
                while 1:
                    color_image, depth_colormap, depth_data = self.get_observation(is_button=True)
                    self.tcp2base = self.robot.get_RT_matrix()
                    self.tcp2base[:3, 3] = self.tcp2base[:3, 3] / 1000
                
                    cv2.imwrite("rgb.png", color_image)
                    np.save("dep.npy", depth_data)
                
                    pose, boxcls = self.button_yolo.run_yolo(2, need_card = False)
                    
                    boxcls = boxcls.tolist()
                    if target_floor in boxcls:
                        break
                
                # self.get_out()
                                
            
            elif key_upper == 'T':
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
                
            elif key_upper == 'A':
                self.mobile_base.rec_loc(5.3, -5.36, 9.5, -17) # debug 
            
            elif key_upper == 'P':
                self.echo_info('Operating: Delivery from front basket to table ...')
                success = self.deliver_from_front_basket_to_table(table_height=0.75)
                if success:
                    print('>>> Delivery task SUCCESS')
                else:
                    print('>>> Delivery task FAILED, see log for details')
            
            elif key_upper == 'Q':
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
        # node.run()
        instruction="pick the cup on the white desk, and place it on the yellow desk"
        knowledge_base="poses"
        # model=""
        node.execute_instruction_with_gpt(instruction,knowledge_base)
        node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
