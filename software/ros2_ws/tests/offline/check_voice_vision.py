"""Synthetic camera fixtures over DDS for actual voice vision nodes."""

from support import ARTIFACT_DIR, WORKSPACE_ROOT
import time
import numpy as np
import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool
from yahboomcar_msgs.msg import Position
from yahboomcar_voice_ctrl.voice_line import VoiceLine
from yahboomcar_voice_ctrl.voice_color_identify import VoiceColorIdentify
from yahboomcar_voice_ctrl.voice_color_tracker import VoiceColorHSV, VoiceDepthTracker
from check_voice import spin, call


def stream(executor, seconds, publishers):
    end = time.monotonic() + seconds
    next_frame = 0.0
    while time.monotonic() < end:
        if time.monotonic() >= next_frame:
            for publisher, message in publishers:
                publisher.publish(message)
            next_frame = time.monotonic() + 0.08
        executor.spin_once(timeout_sec=0.01)


def main():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/vision_check',
                     '-r', '/cmd_vel:=vel_raw', '-r', 'cmd_vel:=vel_raw',
                     '-r', '/camera/rgb/image_raw:=camera/rgb/image_raw',
                     '-r', '/camera/depth/image_raw:=camera/depth/image_raw',
                     '-r', '/camera/depth/camera_info:=camera/depth/camera_info',
                     '-r', '/Current_point:=Current_point', '-r', '/JoyState:=JoyState'])
    executor = SingleThreadedExecutor()
    probe = rclpy.create_node('vision_probe')
    executor.add_node(probe)
    nodes = []
    bridge = CvBridge()
    playback, commands = [], []
    probe.create_subscription(Int32, 'voice/playback', lambda msg: playback.append(msg.data), 100)
    probe.create_subscription(Twist, 'vel_raw', lambda msg: commands.append((msg.linear.x, msg.angular.z)), 100)
    voice = probe.create_publisher(Int32, 'voice/command', 10)
    status = probe.create_publisher(String, 'voice/status', 10)
    rgb = probe.create_publisher(Image, 'camera/rgb/image_raw', qos_profile_sensor_data)
    usb = probe.create_publisher(Image, '/usb_cam/image_raw', qos_profile_sensor_data)
    depth = probe.create_publisher(Image, 'camera/depth/image_raw', qos_profile_sensor_data)
    position = probe.create_publisher(Position, 'Current_point', 10)
    try:
        identify = VoiceColorIdentify()
        nodes.append(identify)
        executor.add_node(identify)
        spin(executor, 0.6)
        red = bridge.cv2_to_imgmsg(np.full((480, 640, 3), (5, 5, 255), np.uint8), 'bgr8')
        stream(executor, 0.2, [(rgb, red)])
        voice.publish(Int32(data=60))
        spin(executor, 0.15)
        assert not playback, 'disabled identification answered'
        enable = probe.create_client(SetBool, 'voice_color_identify/set_enabled')
        assert call(executor, enable, SetBool.Request(data=True)).success
        stream(executor, 0.2, [(rgb, red)])
        voice.publish(Int32(data=60))
        spin(executor, 0.2)
        assert playback == [61], playback
        spin(executor, 0.6)
        voice.publish(Int32(data=60))
        spin(executor, 0.2)
        assert playback == [61], 'stale image answered'
        print('PASS voice image identification: explicit enable + actual RGB image -> red playback; stale/disabled rejected', flush=True)
        executor.remove_node(identify)
        identify.destroy_node()
        nodes.remove(identify)

        line = VoiceLine()
        nodes.append(line)
        executor.add_node(line)
        spin(executor, 0.6)
        line_enable = probe.create_client(SetBool, 'LineDetect/set_enabled')
        assert call(executor, line_enable, SetBool.Request(data=True)).success
        voice.publish(Int32(data=23))
        spin(executor, 0.1)
        stripe = np.zeros((480, 640, 3), np.uint8)
        stripe[240:, 270:370] = (5, 5, 255)
        frame = bridge.cv2_to_imgmsg(stripe, 'bgr8')
        commands.clear()
        stream(executor, 0.5, [(usb, frame)])
        assert any(x > 0 for x, _ in commands), commands
        assert all(abs(x) <= 0.2 and abs(z) <= 0.6 for x, z in commands)
        spin(executor, 0.75)
        assert commands[-1] == (0.0, 0.0), commands[-1]
        status.publish(String(data='disconnect'))
        spin(executor, 0.15)
        voice.publish(Int32(data=23))
        commands.clear()
        stream(executor, 0.25, [(usb, frame)])
        assert not any(x != 0 or z != 0 for x, z in commands)
        print('PASS voice line command -> synthetic stripe -> bounded motion; image loss stops; serial fault latches disabled', flush=True)
        executor.remove_node(line)
        line.destroy_node()
        nodes.remove(line)

        hsv = VoiceColorHSV()
        tracker = VoiceDepthTracker()
        nodes.extend([hsv, tracker])
        hsv.set_tracker_enable_callback(tracker.set_enabled)
        hsv.set_tracker_command_callback(tracker.on_speech_command)
        hsv.set_tracker_reset_callback(tracker.on_speech_reset)
        executor.add_node(hsv)
        executor.add_node(tracker)
        spin(executor, 0.65)
        assert probe.count_publishers('vel_raw') == 1, 'multiple final velocity publishers'
        enable_tracker = probe.create_client(SetBool, 'colorHSV/set_enabled')
        assert call(executor, enable_tracker, SetBool.Request(data=True)).success
        position.publish(Position(angle_x=320.0, angle_y=240.0, distance=60.0))
        depth_frame = bridge.cv2_to_imgmsg(np.full((480, 640), 2.0, np.float32), '32FC1')
        commands.clear()
        stream(executor, 0.25, [(depth, depth_frame)])
        assert not any(x != 0 or z != 0 for x, z in commands), 'enable alone moved tracker'
        voice.publish(Int32(data=74))
        spin(executor, 0.1)
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.circle(image, (320, 240), 60, (10, 200, 10), -1)
        green = bridge.cv2_to_imgmsg(image, 'bgr8')
        stream(executor, 0.8, [(rgb, green), (depth, depth_frame)])
        assert any(x > 0 for x, _ in commands), commands
        assert all(abs(x) <= 0.2 and abs(z) <= 0.6 for x, z in commands)
        stream(executor, 0.8, [(depth, depth_frame)])
        assert commands[-1] == (0.0, 0.0), 'missing RGB did not stop tracker'
        status.publish(String(data='serial read error'))
        spin(executor, 0.2)
        assert not tracker.is_tracker_active
        print('PASS voice RGB + depth tracking: one velocity owner, speech gate, real image/Position/depth chain, watchdog and fault stop', flush=True)
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
