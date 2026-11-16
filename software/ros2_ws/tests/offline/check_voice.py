"""Actual DDS + real pyserial/PTY integration, entirely without a vehicle."""

from support import ARTIFACT_DIR, WORKSPACE_ROOT
import os
import pty
import tempfile
import time
from pathlib import Path
import sys

sys.path.insert(0, str(WORKSPACE_ROOT) + '/src/yahboomcar_voice_ctrl')
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger
from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_voice_ctrl.speech_transport import SpeechDecoder, encode_playback, SpeechTransportNode
from yahboomcar_voice_ctrl.voice_base import VoiceBase
from yahboomcar_voice_ctrl.voice_arm import VoiceArm
from yahboomcar_voice_ctrl.voice_arm_study import VoiceArmStudy
from x3plus_base.base_node import BaseNode


def spin(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def call(executor, client, request):
    deadline = time.monotonic() + 2
    while not client.service_is_ready() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert client.service_is_ready(), client.srv_name
    future = client.call_async(request)
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert future.done(), client.srv_name + ' callback blocked'
    return future.result()


def main():
    decoder = SpeechDecoder()
    assert decoder.feed(b'noise$B0', now=0) == []
    assert decoder.feed(b'04#$B999#$?123#$B005#', now=0.1) == [4, 999, 5]
    assert decoder.feed(b'$B00', now=0.2) == []
    assert decoder.feed(b'6#', now=3.0) == []
    assert decoder.feed(b'x' * 1024 + b'$B007#', now=3.1) == [7]
    assert len(decoder._buffer) <= decoder.max_buffer
    assert encode_playback(7) == b'$A007#'
    for invalid in (-1, 1000, True, float('nan')):
        try:
            encode_playback(invalid)
        except (TypeError, ValueError):
            pass
        else:
            raise AssertionError('invalid playback code accepted')
    print('PASS speech parser fragmented/corrupt/multiple/expired/bounded frames + exact wire encoding', flush=True)

    master, slave = pty.openpty()
    port = os.ttyname(slave)
    os.set_blocking(master, False)
    with tempfile.TemporaryDirectory(prefix='voice-check-') as directory:
        pose_file = str(Path(directory) / 'recorded.yaml')
        args = ['--ros-args', '-r', '__ns:=/voice_test', '-p', 'enable_arm:=true',
                '-p', 'enable_serial:=true', '-p', 'port:=' + port,
                '-p', 'motion_duration:=0.7', '-p', 'pose_file:=' + pose_file]
        rclpy.init(args=args)
        nodes = []
        executor = SingleThreadedExecutor()
        try:
            probe = rclpy.create_node('voice_probe')
            nodes.append(probe)
            velocities, angles, codes = [], [], []
            probe.create_subscription(Twist, 'vel_raw', lambda msg: velocities.append((time.monotonic(), msg.linear.x, msg.angular.z)), 100)
            probe.create_subscription(ArmJoint, 'TargetAngle', lambda msg: angles.append((msg.id, list(msg.joints))), 100)
            probe.create_subscription(Int32, 'voice/command', lambda msg: codes.append(msg.data), 20)
            command = probe.create_publisher(Int32, 'voice/command', 10)
            playback = probe.create_publisher(Int32, 'voice/playback', 10)
            status = probe.create_publisher(String, 'voice/status', 10)
            base = BaseNode()
            voice = VoiceBase()
            arm = VoiceArm()
            study = VoiceArmStudy()
            transport = SpeechTransportNode()
            nodes.extend([base, voice, arm, study, transport])
            for node in nodes:
                executor.add_node(node)
            spin(executor, 0.8)
            assert not Path(pose_file).exists(), 'startup wrote pose file'
            assert not angles, 'startup commanded arm'
            assert all(x == 0 and yaw == 0 for _, x, yaw in velocities), 'startup moved base'
            services = probe.get_service_names_and_types_by_node('voice_arm', '/voice_test')
            assert all(name != '/voice_test/CurrentAngle' for name, _ in services), 'duplicate CurrentAngle server'
            print('PASS all voice constructors disabled + sole base arm-service owner + no startup pose writes', flush=True)

            os.write(master, b'$B0')
            spin(executor, 0.1)
            os.write(master, b'04#$B999#')
            spin(executor, 0.2)
            assert codes == [4], codes
            assert all(x == 0 and yaw == 0 for _, x, yaw in velocities), 'disabled speech moved base'
            playback.publish(Int32(data=7))
            spin(executor, 0.15)
            assert os.read(master, 4096) == b'$A007#'
            print('PASS actual pyserial PTY -> recognized DDS command, idle filtered, DDS playback -> exact TX bytes', flush=True)

            enable = probe.create_client(SetBool, 'voice_base/enable')
            assert call(executor, enable, SetBool.Request(data=True)).success
            start = len(velocities)
            os.write(master, b'$B004#')
            spin(executor, 0.45)
            active = velocities[start:]
            assert len(active) >= 5 and active[-1][1] > 0, 'command refresh absent before deadline'
            assert all(abs(x) <= 0.2 and abs(yaw) <= 0.6 for _, x, yaw in active)
            spin(executor, 0.4)
            assert velocities[-1][1:] == (0.0, 0.0), 'motion did not expire'
            print('PASS recognized speech -> continuously refreshed bounded base motion -> deadline zero', flush=True)

            command.publish(Int32(data=11))
            spin(executor, 0.15)
            assert base.transport.rgb_lamps == (255, 255, 0, 0), base.transport.rgb_lamps
            command.publish(Int32(data=15))
            spin(executor, 0.5)
            assert base.transport.rgb_effect == 1, base.transport.rgb_effect
            print('PASS static RGBColor and RGBLight effect arrive at the actual base interfaces without Int32 overflow', flush=True)

            os.write(master, b'$B004#')
            spin(executor, 0.15)
            status.publish(String(data='serial read error: simulated disconnect'))
            spin(executor, 0.12)
            assert velocities[-1][1:] == (0.0, 0.0)
            os.write(master, b'$B004#')
            spin(executor, 0.15)
            assert velocities[-1][1:] == (0.0, 0.0), 'fault did not require explicit reenable'
            print('PASS serial-status fault stops and latches voice base disabled', flush=True)

            arm_enable = probe.create_client(SetBool, 'voice_arm/set_enabled')
            arm_cancel = probe.create_client(Trigger, 'voice_arm/cancel')
            assert call(executor, arm_enable, SetBool.Request(data=True)).success
            command.publish(Int32(data=49))
            spin(executor, 0.35)
            assert len(angles) == 1 and angles[-1][0] == 10 and len(angles[-1][1]) == 6, angles
            spin(executor, 0.6)
            assert len(angles) == 1, 'single step repeated'
            command.publish(Int32(data=50))
            spin(executor, 0.25)
            assert len(angles) == 2
            assert call(executor, arm_cancel, Trigger.Request()).success
            count = len(angles)
            spin(executor, 0.8)
            assert len(angles) == count, 'canceled sequence resumed'
            print('PASS fresh CurrentAngle client -> one full arm step + multi-step cancellation stays canceled', flush=True)

            study_enable = probe.create_client(SetBool, 'voice_arm_study/set_enabled')
            assert call(executor, study_enable, SetBool.Request(data=True)).success
            command.publish(Int32(data=53))
            spin(executor, 0.08)
            command.publish(Int32(data=54))
            spin(executor, 0.3)
            assert len(study._study_joints) == 1, study._study_joints
            assert not Path(pose_file).exists(), 'record implicitly persisted'
            save = probe.create_client(Trigger, 'voice_arm_study/save_poses')
            assert call(executor, save, Trigger.Request()).success
            assert Path(pose_file).is_file()
            replay = probe.create_client(Trigger, 'voice_arm_study/replay_poses')
            cancel = probe.create_client(Trigger, 'voice_arm_study/cancel_replay')
            assert call(executor, replay, Trigger.Request()).success
            spin(executor, 0.2)
            assert call(executor, cancel, Trigger.Request()).success
            count = len(angles)
            spin(executor, 0.8)
            assert len(angles) == count, 'canceled replay resumed'
            print('PASS asynchronous measured-pose record + explicit persistence + responsive replay cancellation', flush=True)
        finally:
            if 'base' in locals():
                base.close()
            executor.shutdown()
            for node in reversed(nodes):
                node.destroy_node()
            rclpy.shutdown()
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    main()
