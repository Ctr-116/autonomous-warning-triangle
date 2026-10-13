import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


def main():
    rclpy.init()
    node = Node('test_cli')
    pub = node.create_publisher(String, '/test_cmd', 10)
    time.sleep(0.3)  # 等待连接

    if len(sys.argv) < 2:
        print("Usage: ros2 run motion_control_pkg test_cli <command>")
        print("Commands:")
        print("  forward_1m    - 前进1米")
        print("  forward_2m    - 前进2米")
        print("  back_1m       - 后退1米")
        print("  turn_90       - 原地左转90度")
        print("  turn_180      - 原地转180度")
        print("  turnr_90      - 原地右转90度")
        print("  stop          - 急停")
        return

    cmd = sys.argv[1]
    msg = String(data=cmd)
    pub.publish(msg)
    node.get_logger().info(f"Sent: {cmd}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()