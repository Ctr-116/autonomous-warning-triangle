
from support import ARTIFACT_DIR, WORKSPACE_ROOT
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_share_directory
from yahboomcar_laser.laser_avoidance import LaserAvoidanceNode
from yahboomcar_laser.laser_tracker import LaserTrackerNode
from yahboomcar_laser.laser_warning import LaserWarningNode


def spin_for(executor, seconds, publish=None):
    end = time.monotonic() + seconds
    next_publish = 0.0
    while time.monotonic() < end:
        if publish and time.monotonic() >= next_publish:
            publish()
            next_publish = time.monotonic() + 0.04
        executor.spin_once(timeout_sec=0.02)


def main():
    share = Path(get_package_share_directory('yahboomcar_description'))
    urdf = share / 'urdf/yahboomcar_X3plus.urdf'
    root = ET.parse(urdf).getroot()
    assert len(root.findall('link')) == 17
    assert len(root.findall('joint')) == 16
    for mesh in root.findall('.//mesh'):
        filename = mesh.attrib['filename']
        if filename.startswith('package://'):
            package, relative_path = filename[10:].split('/', 1)
            assert (Path(get_package_share_directory(package)) / relative_path).is_file(), filename
    checked = subprocess.run(['check_urdf', str(urdf)], capture_output=True, text=True)
    assert checked.returncode == 0, checked.stderr
    print('PASS URDF tree + all installed mesh references')

    rclpy.init()
    probe = rclpy.create_node('model_laser_validation')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(probe)
    description, transforms = [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    desc_sub = probe.create_subscription(String, '/model_check/robot_description', lambda msg: description.append(msg.data), qos)
    tf_sub = probe.create_subscription(TFMessage, '/tf_static', lambda msg: transforms.extend(msg.transforms), qos)
    log = open(str(ARTIFACT_DIR) + '/model_launch.log', 'w')
    child = subprocess.Popen(['ros2', 'launch', 'yahboomcar_description', 'display.launch.py',
                              'namespace:=model_check', 'frame_prefix:=model_check/', 'rviz:=false'],
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        spin_for(executor, 3.0)
        assert child.poll() is None, 'robot_state_publisher exited'
        assert description and '<robot' in description[-1]
        assert transforms and all(t.child_frame_id.startswith('model_check/') for t in transforms)
        assert probe.count_publishers('/model_check/joint_states') == 0, 'Unexpected synthetic joint state publisher'
        print('PASS headless RSP launch + namespaced description + prefixed static TF + no fake joints')
    finally:
        child.send_signal(signal.SIGINT)
        try: child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait(timeout=3)
        log.close()
        probe.destroy_subscription(desc_sub)
        probe.destroy_subscription(tf_sub)

    commands = []
    sub = probe.create_subscription(Twist, 'vel_raw', lambda msg: commands.append((msg.linear.x, msg.angular.z)), 10)
    pub = probe.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
    scan = LaserScan()
    scan.header.frame_id = 'laser'
    scan.angle_min, scan.angle_max = -math.pi, math.pi
    scan.angle_increment = 2.0 * math.pi / 360.0
    scan.range_min, scan.range_max = 0.05, 10.0
    scan.ranges = [2.0] * 361
    for klass in (LaserAvoidanceNode, LaserTrackerNode, LaserWarningNode):
        node = klass()
        executor.add_node(node)
        try:
            commands.clear()
            scan.ranges = [2.0] * 361
            # An off-axis closer target exercises tracker/warning steering.
            scan.ranges[200] = 0.8
            spin_for(executor, 1.0, lambda: pub.publish(scan))
            assert commands, f'{klass.__name__}: no output from BEST_EFFORT scan'
            assert any(abs(x) + abs(w) > 1e-5 for x, w in commands), f'{klass.__name__}: no active response'
            commands.clear()
            spin_for(executor, 0.8)
            assert commands and commands[-1] == (0.0, 0.0), f'{klass.__name__}: missing-scan stop failed'
            scan.ranges = [float('nan')] * 361
            commands.clear()
            spin_for(executor, 0.3, lambda: pub.publish(scan))
            assert commands and commands[-1] == (0.0, 0.0), f'{klass.__name__}: invalid-scan stop failed'
            print(f'PASS {klass.__name__}: BEST_EFFORT scan -> control, missing/NaN scan -> zero')
        finally:
            node.safe_shutdown()
            executor.remove_node(node)
            node.destroy_node()
    executor.shutdown()
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
