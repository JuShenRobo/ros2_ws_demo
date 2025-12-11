#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        # 创建订阅者
        self.subscription = self.create_subscription(
            Odometry,
            '/slamware_ros_sdk_server_node/odom',
            self.odom_callback,
            10  # QoS queue size
        )
        self.subscription  # prevent unused variable warning

    def odom_callback(self, data):
        # 打印接收到的位姿信息
        self.get_logger().info(f"Position: ({data.pose.pose.position.x:.6f}, "
                              f"{data.pose.pose.position.y:.6f}, "
                              f"{data.pose.pose.position.z:.6f})")
        self.get_logger().info(f"Orientation: ({data.pose.pose.orientation.x:.6f}, "
                              f"{data.pose.pose.orientation.y:.6f}, "
                              f"{data.pose.pose.orientation.z:.6f}, "
                              f"{data.pose.pose.orientation.w:.6f})")

def main(args=None):
    rclpy.init(args=args)
    odom_listener = OdomListener()
    
    try:
        rclpy.spin(odom_listener)
    except KeyboardInterrupt:
        pass
    finally:
        odom_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()