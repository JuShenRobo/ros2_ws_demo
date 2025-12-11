import sys
import time

from models.object_grounding import Ball
from models.model import EE_manipulator

import argparse
import json

from ultralytics import YOLO
import numpy as np


from scipy.spatial.transform import Rotation as R
import cv2
import torch
import os
import pickle
import signal


from PIL import Image

import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import torch

from PIL import Image


def depth_to_point_cloud(depth, intrinsic, scale=1000):
    """
    将深度图转换为点云。
    Args:
        depth (numpy.ndarray): 深度图。
        intrinsic (numpy.ndarray): 相机内参矩阵 (3x3)。
        scale (float): 深度缩放因子（例如，深度图的值单位为毫米，则设置为1000）。

    Returns:
        numpy.ndarray: 点云数组，形状为 (N, 3)。
    """
    # 获取深度图的高度和宽度
    height, width = depth.shape
    
    # 生成像素坐标网格
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    
    # 展平像素坐标和深度图
    u = u.flatten()
    v = v.flatten()
    depth = depth.flatten() / scale  # 将深度值转换为米

    # 过滤掉深度为0的点
    valid = depth > 0
    u, v, depth = u[valid], v[valid], depth[valid]
    
    # 将像素坐标转换为相机坐标系中的点
    fx, fy = intrinsic[0, 0], intrinsic[1, 1]
    cx, cy = intrinsic[0, 2], intrinsic[1, 2]
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth
    
    # 生成点云
    # points = np.vstack((x, y, z)).T
    points = np.vstack((x, y, z))

    return points.T


def dep2pcl_func(depth_image, intrinsic):
    depth_array = np.array(depth_image)  # 转换为numpy数组
    point_cloud = depth_to_point_cloud(depth_array, intrinsic)
    return point_cloud



class Ball():
    def __init__(self):
        pass
        
    def get_position(self, dep_image, bbox_prompt=None):
        intrinsic = np.array([[604.716, 0, 322.682],[0, 603.582, 244.672],[0,  0,  1.0]])

        # check shape with mask
        mask = np.zeros((480, 640), dtype=np.uint8)
        # get a point
        x1,y1,x2,y2 = bbox_prompt
        button_x = (x2+x1)/2.
        button_y = (y1+y2)/2.
        # mask[1,1] = 1
        mask[int(button_y), int(button_x)] = 1
        
        dep_image = mask*dep_image
        
        if dep_image.max()<=0:
            return None, None

        centroid = dep2pcl_func(dep_image, intrinsic)

        return centroid



class EE_manipulator():
    def __init__(self):
        pass
    
    def infer(self, ball, yolo_model, target, need_card=True):
        save_path = '/home/yofo/ros2_ws_demo/src/grasp_topic/scripts'

        # 读取 RGB 图像（默认是 BGR 顺序）
        color = cv2.imread(os.path.join(save_path, "rgb.png"), cv2.IMREAD_COLOR)  # dtype=uint8, shape=(H, W, 3)
        # dep = cv2.imread(os.path.join(save_path, "dep.png"), cv2.IMREAD_UNCHANGED)  # dtype=uint16, shape=(H, W)
        dep = np.load("dep.npy")
        # color = cv2.imread(os.path.join(save_path, "color.png"), cv2.IMREAD_COLOR)  # dtype=uint8, shape=(H, W, 3)
        # dep = cv2.imread(os.path.join(save_path, "dep.png"), cv2.IMREAD_UNCHANGED)  # dtype=uint16, shape=(H, W)
        
        # color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        # img_pil = Image.fromarray(color)
        # img_pil.save("saved_rgb_image.png")

        # dep = np.random.rand(480, 640)
        # names: {0: '4f', 1: '3f', 2: '2f', 3: '1f', 4: 'open', 5: 'close', 6: 'card', 7: '1', 8: '2', 9: '3', 10: '4', 11: 'up', 12: 'down'}

        # result = yolo_model(["saved_rgb_image.png"])[0]
        
        result = yolo_model(["rgb.png"])[0]
        result.save(filename="result.jpg")  # save to disk
        boxes = result.boxes  # Boxes object for bounding box outputs
        print(np.array(boxes.cls.cpu()))
        # print(boxes.xyxy)
        
        target = torch.tensor(float(target), device=boxes.cls.device).float()
        target_idx = torch.where(boxes.cls == target)[0].item()
        target_bbox = boxes.xyxy[target_idx]
        target_bbox = target_bbox.cpu().numpy()

        button_position = ball.get_position(dep,bbox_prompt = target_bbox)
        print("button_position", button_position)

        # ball_position[2] = 22
        # tcp_rot = [-177.68, -1.206, 174.829]
        # new_ball_position2 = ball_position.tolist()
        # print(new_ball_position2 + tcp_rot)

        if need_card:
            target = torch.tensor(float(6), device=boxes.cls.device).float()
            target_idx = torch.where(boxes.cls == target)[0].item()
            target_bbox = boxes.xyxy[target_idx]
            target_bbox = target_bbox.cpu().numpy()

            card_position = ball.get_position(dep, bbox_prompt = target_bbox)
            print("card_position", card_position)

            return button_position, card_position, np.array(boxes.cls.cpu())

        else:
            return button_position, np.array(boxes.cls.cpu())
        

class Button_yolo():
    def __init__(self):
        self.yolo_model = YOLO("./runs/train/weights/last.pt")  # pretrained YOLO11n model
    
    def run_yolo(self, button=8, need_card=True):
        # parser.add_argument('--button', type=int, default=9,
        #                     help="names: {0: '4f', 1: '3f', 2: '2f', 3: '1f', 4: 'open', 5: 'close', 6: 'card', 7: '1', 8: '2', 9: '3', 10: '4', 11: 'up', 12: 'down'")
        
        target = button
        ball = Ball()
        predictor = EE_manipulator()
        if need_card:
            pose, cardpose, boxcls = predictor.infer(ball, self.yolo_model, target, need_card)
            obj2eye = np.eye(4)
            obj2eye[:3, 3] = pose[0]

            obj2eye_card = np.eye(4)
            obj2eye_card[:3, 3] = cardpose[0]

            return obj2eye, obj2eye_card, boxcls
        
        else:
            pose, boxcls = predictor.infer(ball, self.yolo_model, target, need_card)
            obj2eye = np.eye(4)
            obj2eye[:3, 3] = pose[0]
            
            return obj2eye, boxcls

