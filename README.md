# ros2_ws_demo

## 安装与编译
- ros2安装：鱼香ros一键安装 `wget http://fishros.com/install -O fishros && . fishros`
- 控制主机工作空间编译
  ```bash
  export PATH=/usr/bin:$PATH
  source /opt/ros/humble/setup.bash
  cd ros2_ws # 替换成自己的工作空间
  rosdepc update && rosdepc install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  source install/setup.bash
  # 安装python库
  /usr/bin/python3 -m pip install pyserial
  /usr/bin/python3 -m pip install transforms3d
  /usr/bin/python3 -m pip install pyrealsense2
  
## 上传代码
推荐用 SSH 上传，以添加了新氦4号机的鉴权
