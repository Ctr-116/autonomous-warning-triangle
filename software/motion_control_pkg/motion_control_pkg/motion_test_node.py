import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
import math
import time
import csv
import os
from datetime import datetime
from tf_transformations import euler_from_quaternion
from .pid_controller import PID


class MotionTestNode(Node):
    def __init__(self):
        super().__init__('motion_test_node')

        # ========== 参数声明 ==========
        self.declare_parameter('wheel_base', 0.25)  # 轮距(m)，X3 PLUS实测约0.25
        self.declare_parameter('test_speed', 0.2)  # 测试速度 m/s
        self.declare_parameter('turn_speed', 0.15)  # 转向速度 m/s
        self.declare_parameter('kp_v', 0.0)  # 上层速度补偿PID（如底层已有则保持0）
        self.declare_parameter('ki_v', 0.0)
        self.declare_parameter('kd_v', 0.0)
        self.declare_parameter('kp_pos', 1.0)  # 位置环PID
        self.declare_parameter('ki_pos', 0.0)
        self.declare_parameter('kd_pos', 0.1)
        self.declare_parameter('data_dir', '/home/jetson/motion_test_data')

        # ========== 物理参数 ==========
        self.wheel_base = self.get_parameter('wheel_base').value
        self.test_speed = self.get_parameter('test_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        # ========== PID初始化 ==========
        # 位置环（走指定距离/角度用）
        self.pid_pos = PID(
            self.get_parameter('kp_pos').value,
            self.get_parameter('ki_pos').value,
            self.get_parameter('kd_pos').value,
            output_limit=self.test_speed
        )

        # ========== 状态变量 ==========
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.v_yaw = 0.0

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        # 测试状态机
        self.state = 'IDLE'  # IDLE, RUNNING, FINISHED
        self.test_type = ''  # forward, backward, turn_left, turn_right
        self.target_value = 0.0  # 目标距离(m)或角度(rad)
        self.run_start_time = None

        # 数据记录
        self.data_dir = self.get_parameter('data_dir').value
        os.makedirs(self.data_dir, exist_ok=True)
        self.csv_file = None
        self.csv_writer = None
        self.record_buffer = []

        # ========== ROS接口 ==========
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(String, '/test_cmd', self.test_cmd_cb, 10)

        # 状态发布（方便调试）
        self.state_pub = self.create_publisher(String, '/test_state', 10)
        self.dist_pub = self.create_publisher(Float64, '/test_distance', 10)
        self.yaw_err_pub = self.create_publisher(Float64, '/test_yaw_error', 10)

        # 控制循环 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Motion test node started. Waiting for /odom...")
        self.get_logger().info("Available tests: forward_1m, forward_2m, back_1m, turn_90, turn_180, stop")

    def odom_cb(self, msg):
        """接收官方底盘发布的/odom"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.vx = msg.twist.twist.linear.x
        self.v_yaw = msg.twist.twist.angular.z

    def test_cmd_cb(self, msg):
        """解析测试指令"""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"Received test command: {cmd}")

        if cmd == 'stop':
            self.emergency_stop()
            return

        # 解析指令，如 "forward_1m", "turn_90"
        if '_' not in cmd:
            self.get_logger().warn(f"Invalid command format: {cmd}")
            return

        action, val_str = cmd.rsplit('_', 1)
        val_str = val_str.replace('m', '').replace('deg', '')

        try:
            value = float(val_str)
        except ValueError:
            self.get_logger().warn(f"Invalid value in command: {cmd}")
            return

        # 启动测试
        self.start_test(action, value)

    def start_test(self, action, value):
        """启动一次测试"""
        # 重置状态
        self.state = 'RUNNING'
        self.test_type = action
        self.start_x = self.x
        self.start_y = self.y
        self.start_yaw = self.yaw
        self.run_start_time = time.time()
        self.pid_pos.reset()

        # 确定目标值
        if action in ['forward', 'back']:
            self.target_value = abs(value)  # 距离，单位m
            if action == 'back':
                self.target_value = -self.target_value  # 后退用负距离
        elif action in ['turn_left', 'turn_right', 'turn']:
            self.target_value = math.radians(value)  # 角度转弧度
            if action == 'turn_right':
                self.target_value = -self.target_value

        self.get_logger().info(
            f"Test START: {action} | "
            f"target={'%.2fm' % self.target_value if 'turn' not in action else '%.1f°' % math.degrees(self.target_value)} | "
            f"start=({self.start_x:.3f}, {self.start_y:.3f}, {math.degrees(self.start_yaw):.1f}°)"
        )

        # 创建CSV文件
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.data_dir}/{action}_{value}_{timestamp}.csv"
        self.csv_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time', 'target_dist', 'actual_dist', 'target_yaw', 'actual_yaw',
            'vx', 'v_yaw', 'cmd_vx', 'cmd_v_yaw', 'x', 'y', 'yaw_deg'
        ])

    def control_loop(self):
        """50Hz控制循环"""
        # 发布当前状态
        state_msg = String(data=f"{self.state}|{self.test_type}")
        self.state_pub.publish(state_msg)

        if self.state != 'RUNNING':
            # IDLE或FINISHED时保持零速
            self.send_cmd_vel(0.0, 0.0)
            return

        # 计算当前进度
        dx = self.x - self.start_x
        dy = self.y - self.start_y
        traveled = math.sqrt(dx ** 2 + dy ** 2)

        dyaw = self.normalize_angle(self.yaw - self.start_yaw)

        self.dist_pub.publish(Float64(data=traveled))
        self.yaw_err_pub.publish(Float64(data=dyaw))

        # 根据测试类型生成控制指令
        cmd = Twist()

        if self.test_type in ['forward', 'back']:
            # 位置环：根据剩余距离算目标速度
            remaining = abs(self.target_value) - traveled

            if remaining <= 0.02:  # 2cm容差
                self.finish_test(traveled, dyaw)
                return

            # 位置环PID输出目标速度
            target_v = self.pid_pos.update(remaining)
            # 限制最小速度，防止最后蠕行太久
            if target_v < 0.05:
                target_v = 0.05

            # 方向
            if self.test_type == 'back':
                target_v = -target_v

            cmd.linear.x = target_v
            cmd.angular.z = 0.0

        elif self.test_type in ['turn_left', 'turn_right', 'turn']:
            # 角度环
            remaining_yaw = self.target_value - dyaw

            if abs(remaining_yaw) <= math.radians(3):  # 3度容差
                self.finish_test(traveled, dyaw)
                return

            # 简单P控制转向
            k_p_turn = 2.0  # 转向增益
            cmd.angular.z = max(-1.0, min(1.0, k_p_turn * remaining_yaw))
            cmd.linear.x = 0.0  # 原地转向

        # 记录数据
        if self.csv_writer:
            self.csv_writer.writerow([
                f"{time.time() - self.run_start_time:.3f}",
                f"{self.target_value:.3f}",
                f"{traveled:.3f}",
                f"{math.degrees(self.target_value):.1f}",
                f"{math.degrees(dyaw):.1f}",
                f"{self.vx:.3f}",
                f"{self.v_yaw:.3f}",
                f"{cmd.linear.x:.3f}",
                f"{cmd.angular.z:.3f}",
                f"{self.x:.3f}",
                f"{self.y:.3f}",
                f"{math.degrees(self.yaw):.1f}"
            ])

        self.cmd_vel_pub.publish(cmd)

    def finish_test(self, traveled, dyaw):
        """完成测试"""
        duration = time.time() - self.run_start_time

        self.send_cmd_vel(0.0, 0.0)
        self.state = 'FINISHED'

        # 关闭CSV
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None

        self.get_logger().info(
            f"Test FINISHED: {self.test_type} | "
            f"duration={duration:.2f}s | "
            f"traveled={traveled:.3f}m | "
            f"dyaw={math.degrees(dyaw):.1f}° | "
            f"final_pos=({self.x:.3f}, {self.y:.3f})"
        )

    def emergency_stop(self):
        """急停"""
        self.send_cmd_vel(0.0, 0.0)
        self.state = 'IDLE'
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        self.get_logger().warn("EMERGENCY STOP")

    def send_cmd_vel(self, vx, v_yaw):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = v_yaw
        self.cmd_vel_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        """归一化角度到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = MotionTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()