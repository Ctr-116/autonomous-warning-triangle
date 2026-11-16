"""Exercise the real ROS2 base/accessory interfaces in an explicit mock namespace."""

from support import ARTIFACT_DIR, WORKSPACE_ROOT
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32, Bool, Float32
from std_srvs.srv import Trigger
from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray
from x3plus_base.base_node import BaseNode


def spin(executor, seconds, publish=None):
    end = time.monotonic() + seconds
    next_publish = 0.0
    while time.monotonic() < end:
        if publish and time.monotonic() >= next_publish:
            publish()
            next_publish = time.monotonic() + 0.05
        executor.spin_once(timeout_sec=0.02)


def call(executor, client, request):
    deadline = time.monotonic() + 3
    while not client.service_is_ready() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert client.service_is_ready(), 'Service discovery timeout'
    future = client.call_async(request)
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert future.done(), 'Service response timeout'
    return future.result()


def main():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/accessory_test', '-p', 'enable_arm:=true'])
    probe = rclpy.create_node('accessory_validation')
    editions, feedback = [], []
    sub = probe.create_subscription(Float32, 'edition', lambda msg: editions.append(msg.data), 5)
    arm_sub = probe.create_subscription(ArmJoint, 'ArmAngleUpdate', lambda msg: feedback.append(list(msg.joints)), 10)
    base = BaseNode()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    executor.add_node(base)
    rgb = probe.create_publisher(Int32, 'RGBLight', 5)
    buzzer = probe.create_publisher(Bool, 'Buzzer', 5)
    arm = probe.create_publisher(ArmJoint, 'TargetAngle', 5)
    angles = probe.create_client(RobotArmArray, 'CurrentAngle')
    stop = probe.create_client(Trigger, 'base/emergency_stop')
    reset = probe.create_client(Trigger, 'base/reset_stop')
    try:
        spin(executor, 0.7)
        initial = call(executor, angles, RobotArmArray.Request(apply='read'))
        assert len(initial.angles) == 6
        assert feedback and len(feedback[-1]) == 6
        service_names = dict(probe.get_service_names_and_types())
        assert '/base/emergency_stop' not in service_names
        assert '/accessory_test/base/emergency_stop' in service_names
        print('PASS namespace isolation + exact RobotArmArray service + measured mock full-array feedback')
        rgb.publish(Int32(data=4))
        buzzer.publish(Bool(data=True))
        arm.publish(ArmJoint(joints=[100.0, 90.0, 90.0, 90.0, 90.0, 120.0], run_time=500))
        spin(executor, 0.4)
        response = call(executor, angles, RobotArmArray.Request(apply='read'))
        assert abs(response.angles[0] - 100.0) < 0.01
        assert base.transport.rgb_effect == 4 and base.transport.buzzer is True
        print('PASS RGBLight/Buzzer/TargetAngle full-array command -> explicit mock state')
        arm.publish(ArmJoint(id=2, angle=110.0, run_time=250))
        spin(executor, 0.3)
        response = call(executor, angles, RobotArmArray.Request(apply='read'))
        assert abs(response.angles[1] - 110.0) < 0.01
        arm.publish(ArmJoint(id=6, angle=-10.0, run_time=250))
        spin(executor, 0.2)
        assert base.transport.arm_angles[5] == 120.0
        print('PASS single-servo command + invalid-angle rejection')
        stopped = call(executor, stop, Trigger.Request())
        assert stopped.success
        arm.publish(ArmJoint(id=1, angle=130.0, run_time=250))
        spin(executor, 0.3)
        assert base.transport.arm_angles[0] == 100.0
        assert call(executor, reset, Trigger.Request()).success
        arm.publish(ArmJoint(id=1, angle=105.0, run_time=250))
        spin(executor, 0.3)
        assert base.transport.arm_angles[0] == 105.0
        print('PASS stop latch blocks new arm motion; explicit reset + new command restores')
        if not editions: spin(executor, 5.0)
        assert editions and abs(editions[-1] - 3.3) < 1e-4
        print('PASS explicitly simulated firmware-version reply publication')
    finally:
        base.close()
        executor.shutdown()
        probe.destroy_node()
        base.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
