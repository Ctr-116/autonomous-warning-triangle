# -*- coding: utf-8 -*-
"""DDS waypoint integration regression with native ROS 2 Humble.

Tests native DDS interfaces with explicit mock Nav2 NavigateToPose ActionServer.
Does NOT run on real vehicle hardware or launch actual Nav2 stack.
"""

import math
import os
import sys
import threading
import time

# Ensure sibling directory is in sys.path to import helpers
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from voicebiz_runtime import MockNavNode, call_service, spin_until

import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger

from yahboomcar_voice_ctrl.voice_waypoints import VoiceWaypoints, WAYPOINTS


class DetailedMockNavNode(MockNavNode):
    """Subclass of MockNavNode recording full PoseStamped requests."""

    def __init__(self):
        super().__init__()
        self.request_poses = []

    def goal_callback(self, goal_request):
        pose = goal_request.pose
        with self._lock:
            self.request_poses.append(pose)
        return super().goal_callback(goal_request)


def main():
    args = [
        '--ros-args',
        '-r', '__ns:=/waypoint_test',
        '-p', 'enabled:=false',
        '-p', 'goal_timeout:=0.5',
        '-p', 'sending_timeout:=0.2',
    ]
    rclpy.init(args=args)

    mock_nav = None
    multi_exec = None
    multi_thread = None
    waypoint_node = None
    probe = None
    single_exec = None

    try:
        mock_nav = DetailedMockNavNode()

        multi_exec = MultiThreadedExecutor(num_threads=4)
        multi_exec.add_node(mock_nav)
        multi_thread = threading.Thread(target=multi_exec.spin, daemon=True)
        multi_thread.start()

        waypoint_node = VoiceWaypoints()
        probe = rclpy.create_node('waypoint_probe')

        single_exec = SingleThreadedExecutor()
        single_exec.add_node(waypoint_node)
        single_exec.add_node(probe)

        # Probe communication interfaces
        cmd_pub = probe.create_publisher(Int32, 'voice/command', 10)
        serial_status_pub = probe.create_publisher(String, 'voice/status', 10)

        status_msgs = []
        playback_msgs = []

        probe.create_subscription(
            String, 'voice_waypoints/status', lambda m: status_msgs.append(m.data), 100)
        probe.create_subscription(
            Int32, 'voice/playback', lambda m: playback_msgs.append(m.data), 100)

        enable_cli = probe.create_client(SetBool, 'voice_waypoints/set_enabled')
        cancel_cli = probe.create_client(Trigger, 'voice_waypoints/cancel')

        # ----------------------------------------------------------------------
        # Case 1: Disabled at startup, no goals sent, readonly parameter rejection
        # ----------------------------------------------------------------------
        cmd_pub.publish(Int32(data=19))
        spin_until(single_exec, lambda: False, timeout=0.2)
        assert len(mock_nav.request_x_history) == 0, "No goals should send while disabled"
        assert len(playback_msgs) == 0, "No playback while disabled"

        # Check atomic parameter rejection for read-only parameters
        bad_res = waypoint_node.set_parameters_atomically([
            Parameter('enabled', Parameter.Type.BOOL, True),
            Parameter('goal_timeout', Parameter.Type.DOUBLE, 10.0),
        ])
        assert not bad_res.successful, "Atomic parameter update with read-only parameter must fail"
        assert not waypoint_node._enabled, "Node must not enable when atomic parameter set fails"

        print("PASS Case 1: Disabled startup and parameter validation (honest mock Nav2)", flush=True)

        # ----------------------------------------------------------------------
        # Case 2: Explicit enable and sequential sending of all 5 waypoint IDs
        # ----------------------------------------------------------------------
        resp = call_service(single_exec, enable_cli, SetBool.Request(data=True))
        assert resp.success, "Failed to enable waypoints node"
        assert waypoint_node._enabled, "Node internal state should be enabled"

        all_ids = [19, 20, 21, 32, 33]
        for wp_id in all_ids:
            cur_req_count = len(mock_nav.request_x_history)
            cmd_pub.publish(Int32(data=wp_id))

            # Spin until completed before next
            ok = spin_until(
                single_exec,
                lambda: any(f'goal {wp_id} completed' in s for s in status_msgs),
                timeout=4.0
            )
            assert ok, f"Goal {wp_id} did not complete"
            assert len(mock_nav.request_x_history) == cur_req_count + 1

            # Check recorded PoseStamped
            recorded_pose = mock_nav.request_poses[-1]
            expected = WAYPOINTS[wp_id]
            assert math.isclose(recorded_pose.pose.position.x, expected['x'], rel_tol=1e-5)
            assert math.isclose(recorded_pose.pose.position.y, expected['y'], rel_tol=1e-5)
            assert recorded_pose.pose.position.z == 0.0

            qz = recorded_pose.pose.orientation.z
            qw = recorded_pose.pose.orientation.w
            q_norm = math.sqrt(qz * qz + qw * qw)
            assert math.isclose(q_norm, 1.0, rel_tol=1e-5), f"Quaternion norm not unit: {q_norm}"

        assert [p.pose.position.x for p in mock_nav.request_poses] == [WAYPOINTS[i]['x'] for i in all_ids]
        assert playback_msgs == all_ids, f"Expected playback {all_ids}, got {playback_msgs}"

        print("PASS Case 2: All 5 waypoints sequential execution, coordinates and quaternion norm verified (honest mock Nav2)", flush=True)

        # ----------------------------------------------------------------------
        # Case 3: Repeated same ID after completion and busy rejection
        # ----------------------------------------------------------------------
        # Repeated same ID (33)
        mock_nav.queue_plan(outcome='hold')
        cmd_pub.publish(Int32(data=33))
        ok = spin_until(single_exec, lambda: len(mock_nav.active_goals) > 0, timeout=2.0)
        assert ok, "Goal 33 not active"

        # While busy, send another goal -> must be rejected as busy
        status_msgs.clear()
        cmd_pub.publish(Int32(data=19))
        spin_until(single_exec, lambda: False, timeout=0.1)
        assert 'busy' in status_msgs, "Should report busy when goal is currently active"

        # Cancel holding goal via service Trigger
        resp = call_service(single_exec, cancel_cli, Trigger.Request())
        assert resp.success
        ok = spin_until(single_exec, lambda: len(mock_nav.active_goals) == 0, timeout=2.0)
        assert ok

        # Send ID 33 again after completion/cancel -> must send again
        req_count = len(mock_nav.request_x_history)
        cmd_pub.publish(Int32(data=33))
        ok = spin_until(single_exec, lambda: len(mock_nav.request_x_history) > req_count, timeout=2.0)
        assert ok, "Repeated command ID 33 failed to send again after session ended"
        ok = spin_until(single_exec, lambda: any('goal 33 completed' in s for s in status_msgs), timeout=3.0)
        assert ok

        print("PASS Case 3: Repeated ID after completion and busy rejection while held (honest mock Nav2)", flush=True)

        # ----------------------------------------------------------------------
        # Case 4: Unknown command 999
        # ----------------------------------------------------------------------
        status_msgs.clear()
        req_count = len(mock_nav.request_x_history)
        cmd_pub.publish(Int32(data=999))
        spin_until(single_exec, lambda: False, timeout=0.1)
        assert len(mock_nav.request_x_history) == req_count, "Unknown command 999 must not send goal"
        assert any('unknown command 999' in s for s in status_msgs)

        print("PASS Case 4: Unknown command 999 generates no goal (honest mock Nav2)", flush=True)

        # ----------------------------------------------------------------------
        # Case 5: Action rejection latches fault and disables node
        # ----------------------------------------------------------------------
        mock_nav.queue_plan(outcome='reject')
        cmd_pub.publish(Int32(data=19))
        ok = spin_until(single_exec, lambda: waypoint_node._faulted and not waypoint_node._enabled, timeout=2.0)
        assert ok, "Goal rejection should latch fault and disable node"

        # Re-enable node
        resp = call_service(single_exec, enable_cli, SetBool.Request(data=True))
        assert resp.success

        print("PASS Case 5: Goal rejection latches fault (honest mock Nav2)", flush=True)

        # ----------------------------------------------------------------------
        # Case 6: Action abort latches fault and disables node
        # ----------------------------------------------------------------------
        mock_nav.queue_plan(outcome='abort')
        cmd_pub.publish(Int32(data=19))
        ok = spin_until(single_exec, lambda: waypoint_node._faulted and not waypoint_node._enabled, timeout=2.0)
        assert ok, "Goal abort should latch fault and disable node"

        # Re-enable node
        resp = call_service(single_exec, enable_cli, SetBool.Request(data=True))
        assert resp.success

        print("PASS Case 6: Goal abort latches fault (honest mock Nav2)", flush=True)

        # ----------------------------------------------------------------------
        # Case 7: Goal result timeout latches fault
        # ----------------------------------------------------------------------
        # Node goal_timeout is 0.5s; hold mock server longer
        mock_nav.queue_plan(outcome='hold')
        cmd_pub.publish(Int32(data=19))
        ok = spin_until(single_exec, lambda: waypoint_node._faulted and not waypoint_node._enabled, timeout=2.0)
        assert ok, "Goal timeout must latch fault and disable node"

        # Re-enable node
        resp = call_service(single_exec, enable_cli, SetBool.Request(data=True))
        assert resp.success

        print("PASS Case 7: Goal timeout latches fault (honest mock Nav2)", flush=True)

        # ----------------------------------------------------------------------
        # Case 8: Late accept cancel (accept_delay > sending_timeout=0.2)
        # ----------------------------------------------------------------------
        mock_nav.queue_plan(outcome='hold', accept_delay=0.35)
        before_cancels = mock_nav.cancel_count
        cmd_pub.publish(Int32(data=19))

        # Wait for sending timeout fault to latch (sending_timeout is 0.2)
        ok = spin_until(single_exec, lambda: waypoint_node._faulted, timeout=2.0)
        assert ok, "Sending timeout should trigger fault"

        # After accept_delay elapses, late acceptance callback cancels the leaked handle
        ok = spin_until(single_exec, lambda: mock_nav.cancel_count > before_cancels, timeout=2.0)
        assert ok, "Late accepted handle must be canceled by late cancel mechanism"

        print("PASS Case 8: Late accept cancel prevents handle leak (honest mock Nav2)", flush=True)

        # ----------------------------------------------------------------------
        # Case 9: Command 0 cancels even while disabled
        # ----------------------------------------------------------------------
        # Ensure node is disabled
        resp = call_service(single_exec, enable_cli, SetBool.Request(data=False))
        assert resp.success
        assert not waypoint_node._enabled

        status_msgs.clear()
        cmd_pub.publish(Int32(data=0))
        spin_until(single_exec, lambda: False, timeout=0.1)
        assert any('cancel: stop id 0' in s for s in status_msgs), "Command 0 must publish stop cancel even when disabled"

        print("PASS Case 9: Command 0 cancels even when disabled (honest mock Nav2)", flush=True)

        print("\nALL WAYPOINT RUNTIME TESTS PASSED (Honest Mock Nav2 ActionServer)", flush=True)

    finally:
        if multi_exec is not None:
            multi_exec.shutdown()
        if multi_thread is not None and multi_thread.is_alive():
            multi_thread.join(timeout=1.0)
        if waypoint_node is not None:
            try:
                waypoint_node.destroy_node()
            except Exception:
                pass
        if probe is not None:
            try:
                probe.destroy_node()
            except Exception:
                pass
        if single_exec is not None:
            try:
                single_exec.shutdown()
            except Exception:
                pass
        if mock_nav is not None:
            try:
                mock_nav.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            try:
                rclpy.try_shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        sys.exit(1)
