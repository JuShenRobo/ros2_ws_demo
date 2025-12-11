import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2

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
        # print(centroid.shape) (1, 3)
        # return centroid*1000
        return centroid






        
        
        