import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import torch
import time
import os
import pickle
import json
import signal


from PIL import Image

class EE_manipulator():
    def __init__(self):
        pass
    
    def infer(self, ball, yolo_model, target):
        
        save_path = '/home/user/PTY/button/out'

        # 读取 RGB 图像（默认是 BGR 顺序）
        color = cv2.imread(os.path.join(save_path, "rgb.png"), cv2.IMREAD_COLOR)  # dtype=uint8, shape=(H, W, 3)
        dep = cv2.imread(os.path.join(save_path, "depth.png"), cv2.IMREAD_UNCHANGED)  # dtype=uint16, shape=(H, W)
        # color = cv2.imread(os.path.join(save_path, "color.png"), cv2.IMREAD_COLOR)  # dtype=uint8, shape=(H, W, 3)
        # dep = cv2.imread(os.path.join(save_path, "dep.png"), cv2.IMREAD_UNCHANGED)  # dtype=uint16, shape=(H, W)
        
        # color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        # img_pil = Image.fromarray(color)
        # img_pil.save("saved_rgb_image.png")

        # dep = np.random.rand(480, 640)
        # names: {0: '4f', 1: '3f', 2: '2f', 3: '1f', 4: 'open', 5: 'close', 6: 'card', 7: '1', 8: '2', 9: '3', 10: '4', 11: 'up', 12: 'down'}

        # result = yolo_model(["saved_rgb_image.png"])[0]
        result = yolo_model(["./out/rgb.png"])[0]
        result.save(filename="result.jpg")  # save to disk
        boxes = result.boxes  # Boxes object for bounding box outputs
        print(np.array(boxes.cls.cpu()))
        # print(boxes.xyxy)
        
        target = torch.tensor(float(target), device=boxes.cls.device).float()
        target_idx = torch.where(boxes.cls == target)[0].item()
        target_bbox = boxes.xyxy[target_idx]
        target_bbox = target_bbox.cpu().numpy()

        button_position = ball.get_position(dep,bbox_prompt = target_bbox)
        # print(button_position)

        # ball_position[2] = 22
        # tcp_rot = [-177.68, -1.206, 174.829]
        # new_ball_position2 = ball_position.tolist()
        # print(new_ball_position2 + tcp_rot)

        return button_position, np.array(boxes.cls.cpu())
