export PATH=/usr/bin:$PATH
source /opt/ros/humble/setup.bash
source /home/yofo/ros2_ws_demo/install/setup.bash  # 绝对路径（核心修改）
# ros2 run slamware_ros_sdk multi_floor.py
python3 /home/yofo/ros2_ws_demo/install/slamware_ros_sdk/lib/slamware_ros_sdk/multi_floor.py