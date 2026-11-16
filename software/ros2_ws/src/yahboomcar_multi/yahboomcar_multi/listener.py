#!/usr/bin/env python3
# coding: utf-8
"""RobotListener - ROS2 Humble port of ROS1 yahboomcar_multi/scripts/listener.py.

Original: yahboomcar_multi (ROS1). License unknown (TODO).

Native ROS2 parameters replace RobotListener.cfg dynamic_reconfigure.
All cmd_vel/goal/scan subscriptions are relative (resolve into the node namespace).
Frames are explicit (leader base_footprint, source formation frame, map).
Bounded outputs: |x|,|y| <= 0.2 m/s, |angular.z| <= 0.6 rad/s.
A steady wall-clock watchdog (time.monotonic) zeroes velocity if the loop stalls,
even if the simulated clock stops.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from yahboomcar_multi.single_pid import SinglePID

RAD2DEG = 180.0 / math.pi

def clamp(value, low, high):
    return max(low, min(high, value))

def compute_pid_twist(x, y, theta, lin_pid, ang_pid, robot_model, distance, x_limit=0.2,
                      angular_limit=0.6):
    """Compute a Twist from relative target pose using the original PID equations.

    Returns (Twist, distance). Bounded outputs applied.
    """
    twist = Twist()
    if robot_model == 'omni':
        raw_x = lin_pid.pid_compute(x, 0) / 4.0
        raw_y = lin_pid.pid_compute(y, 0) / 4.0
        raw_w = lin_pid.pid_compute(
            np.interp(theta * RAD2DEG, [-180, 180], [-5, 5]), 0) / 4.0
        twist.linear.x = clamp(raw_x, -x_limit, x_limit)
        twist.linear.y = clamp(raw_y, -x_limit, x_limit)
        twist.angular.z = clamp(raw_w, -angular_limit, angular_limit)
    elif robot_model == 'diff':
        direction = 1.0 if x >= 0 else -1.0
        angular = np.interp(math.atan2(y, x) * RAD2DEG, [-180, 180], [-5, 5])
        if abs(distance) > 0.1:
            lin = lin_pid.pid_compute(distance, 0)
            if lin > 0.6:
                twist.linear.x = 0.6 * direction
            else:
                twist.linear.x = lin * direction
        if abs(y) > 0.1:
            twist.angular.z = ang_pid.pid_compute(angular, 0)
        else:
            twist.angular.z = theta
        twist.linear.x = clamp(twist.linear.x, -x_limit, x_limit)
        twist.angular.z = clamp(twist.angular.z, -angular_limit, angular_limit)
    return twist


def scan_warning(scan_data, laser_angle_deg, response_dist):
    """Return warning count (starts at 1) for obstacles in the rear sectors."""
    warning = 1
    if scan_data is None:
        return warning
    ranges = np.asarray(scan_data.ranges, dtype=float)
    for i in range(len(ranges)):
        angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
        if abs(angle) > (180.0 - laser_angle_deg):
            if ranges[i] < response_dist:
                warning += 1
    return warning


class RobotListener(Node):
    def __init__(self):
        super().__init__('RobotListener')

        # Native ROS2 replacement for RobotListener.cfg (defaults preserved).
        self.declare_parameter('robot_name', 'robot2')
        self.declare_parameter('source_frame', 'point1')
        self.declare_parameter('leader_frame', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lin_Kp', 1.0)
        self.declare_parameter('lin_Ki', 0.0)
        self.declare_parameter('lin_Kd', 1.0)
        self.declare_parameter('ang_Kp', 0.8)
        self.declare_parameter('ang_Ki', 0.0)
        self.declare_parameter('ang_Kd', 1.0)
        self.declare_parameter('teams', 1)  # 0 convoyl, 1 vertical, 2 horizonta
        self.declare_parameter('robot_model', 0)  # 0 omni, 1 diff
        self.declare_parameter('navigate', False)
        self.declare_parameter('switch', True)
        self.declare_parameter('dist', 0.7)
        self.declare_parameter('linPIDparam', [1.0, 0.0, 1.0])
        self.declare_parameter('angPIDparam', [0.8, 0.0, 1.0])
        self.declare_parameter('laser_angle', 30.0)
        self.declare_parameter('response_dist', 0.3)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('stale_transform_sec', 0.5)
        self.declare_parameter('watchdog_sec', 0.5)
        self.declare_parameter('max_linear', 0.2)
        self.declare_parameter('max_angular', 0.6)

        self.robot_name = self.get_parameter('robot_name').value
        self.source_frame = self.get_parameter('source_frame').value
        leader_frame = self.get_parameter('leader_frame').value
        self.robot_pose = leader_frame if leader_frame else (self.robot_name + '/base_footprint')
        self.map_frame = self.get_parameter('map_frame').value

        self.switch = bool(self.get_parameter('switch').value)
        self.navigate = bool(self.get_parameter('navigate').value)
        self.Joy_active = False
        self.pause = False
        self.warning = 1
        self.LaserAngle = float(self.get_parameter('laser_angle').value)
        self.ResponseDist = float(self.get_parameter('response_dist').value)
        self.min_dist = 100.0
        self.minX = 0.0
        self.minY = 0.0
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.stale_sec = float(self.get_parameter('stale_transform_sec').value)
        self.watchdog_sec = float(self.get_parameter('watchdog_sec').value)

        self.robot_model = 'omni' if int(self.get_parameter('robot_model').value) == 0 else 'diff'
        teams = int(self.get_parameter('teams').value)
        if teams == 0:
            self.order = 'convoyl'
        elif teams == 1:
            self.order = 'vertical'
        else:
            self.order = 'horizonta'
        self.dist = float(self.get_parameter('dist').value)

        self.lin_pid = SinglePID()
        self.ang_pid = SinglePID()
        lin = self.get_parameter('linPIDparam').value
        ang = self.get_parameter('angPIDparam').value
        if lin is None or len(lin) != 3:
            lin = [self.get_parameter('lin_Kp').value, self.get_parameter('lin_Ki').value,
                   self.get_parameter('lin_Kd').value]
        if ang is None or len(ang) != 3:
            ang = [self.get_parameter('ang_Kp').value, self.get_parameter('ang_Ki').value,
                   self.get_parameter('ang_Kd').value]
        self.lin_pid.Set_pid(lin[0], lin[1], lin[2])
        self.ang_pid.Set_pid(ang[0], ang[1], ang[2])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        self.pub_order = self.create_publisher(String, 'order', 10)
        self.pub_dist = self.create_publisher(Float32, 'dist', 10)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_goal = self.create_publisher(PoseStamped, 'move_base_simple/goal', 1)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.registerScan, 1)
        self.sub_JoyState = self.create_subscription(Bool, 'JoyState', self.JoyStateCallback, 10)

        self.last_loop_wall = time.monotonic()
        self.last_vel_wall = time.monotonic()
        rate_hz = float(self.get_parameter('rate_hz').value)
        if rate_hz <= 0.0:
            rate_hz = 10.0
        self.timer = self.create_timer(1.0 / rate_hz, self.run_loop)
        self.get_logger().info(
            'RobotListener started robot=%s source=%s leader=%s model=%s' %
            (self.robot_name, self.source_frame, self.robot_pose, self.robot_model))

    # ---- callbacks ----
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool):
            return
        self.Joy_active = msg.data

    def registerScan(self, scan_data):
        if self.Joy_active:
            return
        self.warning = scan_warning(scan_data, self.LaserAngle, self.ResponseDist)

    # ---- control loop ----
    def run_loop(self):
        now_wall = time.monotonic()
        if (now_wall - self.last_loop_wall) > self.watchdog_sec:
            self.get_logger().warn('Control loop stalled > %.2fs; zeroing velocity' % self.watchdog_sec)
            self.pub_vel.publish(Twist())
            self.last_loop_wall = now_wall
            return
        self.last_loop_wall = now_wall

        start_status = True
        if self.Joy_active or self.switch:
            self.min_dist = 100.0
            if self.pause:
                for _ in range(3):
                    self.pub_vel.publish(Twist())
                self.pause = False
        else:
            self.pause = True
            if self.navigate:
                transform = self.lookup(self.map_frame, self.source_frame)
                if transform is not None:
                    self.PubTargetPoint(transform)
            else:
                transform = self.lookup(self.robot_pose, self.source_frame)
                if transform is not None:
                    self.apply_control(transform, start_status)

    def lookup(self, target_frame, source_frame):
        try:
            tf_msg = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except (TransformException, Exception) as e:
            self.get_logger().error('lookup_transform(%s, %s) failed: %s' %
                                    (target_frame, source_frame, str(e)))
            return None
        age = time.monotonic() - self.last_loop_wall
        if age > self.stale_sec:
            self.get_logger().warn('Stale transform age %.2fs > threshold; zeroing' % age)
            self.pub_vel.publish(Twist())
            return None
        return tf_msg

    def apply_control(self, transform, start_status):
        t = transform.transform
        x = t.translation.x
        y = t.translation.y
        q = t.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)
        distance = math.sqrt(x * x + y * y)

        if start_status:
            if (abs(self.min_dist - distance) < 0.01 and abs(self.minX - x) < 0.1 and abs(self.minY - y) < 0.1):
                start_status = False
            elif self.min_dist > distance:
                self.min_dist = distance
                self.minX = x
                self.minY = y
        else:
            twist = compute_pid_twist(x, y, theta, self.lin_pid, self.ang_pid,
                                      self.robot_model, distance,
                                      self.max_linear, self.max_angular)
            self.pub_vel.publish(twist)

    def PubTargetPoint(self, transform):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation.x = transform.transform.rotation.x
        pose.pose.orientation.y = transform.transform.rotation.y
        pose.pose.orientation.z = transform.transform.rotation.z
        pose.pose.orientation.w = transform.transform.rotation.w
        self.pub_goal.publish(pose)

    def publish_params_to_formation(self):
        msg = String()
        msg.data = self.order
        for _ in range(3):
            self.pub_order.publish(msg)
        f = Float32()
        f.data = self.dist
        for _ in range(3):
            self.pub_dist.publish(f)

    def destroy_node(self):
        try:
            self.pub_vel.publish(Twist())
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
