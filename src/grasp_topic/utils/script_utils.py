#!/home/adminpc/anaconda3/envs/py3.10/bin/python 
from math import radians, cos, sin
import re
import numpy as np
import sys
import termios
import tty
from scipy.spatial.transform import Rotation as R

def euler_to_rotation_matrix(rx, ry, rz):
    """将欧拉角 (Rx, Ry, Rz) 转换为 3x3 旋转矩阵 (ZYX 顺序)"""
    rx, ry, rz = radians(rx), radians(ry), radians(rz)
    
    # 计算旋转矩阵（ZYX 顺序）
    Rz = np.array([
        [cos(rz), -sin(rz), 0],
        [sin(rz),  cos(rz), 0],
        [0,        0,       1]
    ])
    
    Ry = np.array([
        [cos(ry),  0, sin(ry)],
        [0,        1, 0],
        [-sin(ry), 0, cos(ry)]
    ])
    
    Rx = np.array([
        [1, 0,        0],
        [0, cos(rx), -sin(rx)],
        [0, sin(rx),  cos(rx)]
    ])
    
    R = Rz @ Ry @ Rx  # 组合旋转（Z → Y → X）
    return R


def string_to_homogeneous_matrix(s):
    """将字符串 '[X,Y,Z] [Rx,Ry,Rz]' 转换为 4x4 齐次矩阵"""
    # 提取数字部分
    matches = re.findall(r"[-+]?\d*\.\d+|\d+", s)
    if len(matches) != 6:
        raise ValueError("输入格式错误！应为 '[X,Y,Z] [Rx,Ry,Rz]'")
    
    # 解析 XYZ 和 Rx,Ry,Rz
    x, y, z = map(float, matches[:3])
    rx, ry, rz = map(float, matches[3:6])
    
    # 计算旋转矩阵
    R = euler_to_rotation_matrix(rx, ry, rz)
    
    # 构建齐次矩阵
    T = np.eye(4)
    T[:3, :3] = R      # 旋转部分
    T[:3, 3] = [x, y, z]  # 平移部分
    return T


def rotation_matrix_to_euler(R_matrix):
    """
    将旋转矩阵转换为欧拉角
    """
    rotation = R.from_matrix(R_matrix)
    euler_angles = rotation.as_euler('xyz', degrees=True)  # 返回的是 [roll, pitch, yaw]
    return euler_angles


def getch():
    """获取单个字符输入（不需要按回车）"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch