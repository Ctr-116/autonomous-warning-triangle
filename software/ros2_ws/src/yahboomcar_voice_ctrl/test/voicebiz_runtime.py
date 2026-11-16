# -*- coding: utf-8 -*-
"""DDS business integration regression with native ROS 2 Humble.

Tests native DDS interfaces with mock Nav2 NavigateToPose ActionServer and
X3plus mock BaseNode. Does NOT run on real vehicle hardware or launch actual
Nav2 stack.
"""

import math
import sys
import threading
import time

import numpy as np
import cv2

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray
from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray

from x3plus_base.base_node import BaseNode
from yahboomcar_voice_ctrl.voice_business import VoiceColorTransport, VoiceTakeThings


class MockNavNode(Node):
    """Explicitly mock Nav2 NavigateToPose ActionServer with ReentrantCallbackGroup."""

    def __init__(self):
        super().__init__('mock_nav2_server')
        self._cb_group = ReentrantCallbackGroup()
        self.request_x_history = []
        self.cancel_count = 0
        self.active_goals = []
        self._lock = threading.Lock()
        self.plan_queue = []
        self.accepted_plans = []

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

    def queue_plan(self, outcome='success', accept_delay=0.0):
        with self._lock:
            self.plan_queue.append({'outcome': outcome, 'accept_delay': accept_delay})

    def goal_callback(self, goal_request):
        pos_x = goal_request.pose.pose.position.x
        with self._lock:
            self.request_x_history.append(pos_x)
            plan = self.plan_queue.pop(0) if self.plan_queue else {'outcome': 'success', 'accept_delay': 0.0}

        delay = plan.get('accept_delay', 0.0)
        if delay > 0:
            time.sleep(delay)

        if plan.get('outcome') == 'reject':
            return GoalResponse.REJECT
        with self._lock:
            self.accepted_plans.append(plan)
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        with self._lock:
            self.cancel_count += 1
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        with self._lock:
            self.active_goals.append(goal_handle)

        with self._lock:
            outcome = self.accepted_plans.pop(0)['outcome']

        # Small fixture wait
        time.sleep(0.04)

        start_time = time.monotonic()
        while time.monotonic() - start_time < 3.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with self._lock:
                    if goal_handle in self.active_goals:
                        self.active_goals.remove(goal_handle)
                return NavigateToPose.Result()
            time.sleep(0.02)
            if outcome == 'abort':
                break
            if outcome == 'success':
                break

        with self._lock:
            if goal_handle in self.active_goals:
                self.active_goals.remove(goal_handle)

        if outcome == 'abort':
            goal_handle.abort()
        elif outcome == 'success':
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return NavigateToPose.Result()

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()


def spin_until(executor, predicate, timeout=10.0, step=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=step)
        if predicate():
            return True
    return False


def call_service(executor, client, request, timeout=4.0):
    deadline = time.monotonic() + timeout
    while not client.service_is_ready() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert client.service_is_ready(), f'{client.srv_name} not ready'
    future = client.call_async(request)
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert future.done(), f'{client.srv_name} call timeout'
    return future.result()


def main():
    args = [
        '--ros-args',
        '-r', '__ns:=/voicebiz_test',
        '-p', 'time_scale:=0.02',
        '-p', 'service_timeout:=0.4',
        '-p', 'nav_response_timeout:=0.5',
        '-p', 'nav_result_timeout:=1.0',
        '-p', 'enabled:=false',
        '-p', 'enable_arm:=true',
        '-p', 'use_mock:=true',
    ]
    rclpy.init(args=args)

    base = None
    mock_nav = None
    multi_exec = None
    multi_thread = None
    business = None
    probe = None
    single_exec = None

    try:
        # BaseNode & MockNav in MultiThreadedExecutor
        base = BaseNode()
        mock_nav = MockNavNode()

        # Wrap BaseNode.current_angle_service.callback
        original_arm_cb = base.current_angle_service.callback
        arm_service_wrapper = {
            'count': 0,
            'delay': 0.0,
            'invalid': False,
            'last_req_apply': None,
        }

        def wrapped_arm_cb(request, response):
            arm_service_wrapper['count'] += 1
            arm_service_wrapper['last_req_apply'] = request.apply
            if arm_service_wrapper['delay'] > 0:
                time.sleep(arm_service_wrapper['delay'])
            if arm_service_wrapper['invalid']:
                response.angles = []
                return response
            return original_arm_cb(request, response)

        base.current_angle_service.callback = wrapped_arm_cb

        multi_exec = MultiThreadedExecutor(num_threads=4)
        multi_exec.add_node(base)
        multi_exec.add_node(mock_nav)
        multi_thread = threading.Thread(target=multi_exec.spin, daemon=True)
        multi_thread.start()

        # DDS Probe Node
        probe = rclpy.create_node('voicebiz_probe')

        cmd_pub = probe.create_publisher(Int32, 'voice/command', 10)
        status_pub = probe.create_publisher(String, 'voice/status', 10)

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_pub = probe.create_publisher(MarkerArray, 'color_end_pose', marker_qos)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_pub = probe.create_publisher(Image, 'camera/rgb/image_raw', image_qos)

        arm_targets = []
        buzzer_msgs = []
        playback_msgs = []
        twist_msgs = []

        probe.create_subscription(ArmJoint, 'TargetAngle', lambda m: arm_targets.append(m), 100)
        probe.create_subscription(Bool, 'Buzzer', lambda m: buzzer_msgs.append(m.data), 100)
        probe.create_subscription(Int32, 'voice/playback', lambda m: playback_msgs.append(m.data), 100)
        probe.create_subscription(Twist, 'cmd_vel', lambda m: twist_msgs.append(m), 100)

        # Helpers for probe publishers
        bridge = CvBridge()

        def send_markers():
            markers = []
            specs = [
                (0, 1.0, 0.0),  # 0: red -> X=1
                (1, 2.0, 0.0),  # 1: green -> X=2
                (2, 3.0, 0.0),  # 2: blue -> X=3
                (3, 4.0, 0.0),  # 3: yellow -> X=4
                (5, 0.0, 0.0),  # 5: origin -> X=0
            ]
            for mid, x, y in specs:
                m = Marker()
                m.header.frame_id = 'map'
                m.id = mid
                m.action = Marker.ADD
                m.pose.position.x = float(x)
                m.pose.position.y = float(y)
                m.pose.position.z = 0.0
                m.pose.orientation.w = 1.0
                markers.append(m)
            ma = MarkerArray()
            ma.markers = markers
            marker_pub.publish(ma)

        def send_fresh_red_image():
            img = np.full((480, 640, 3), (5, 5, 255), dtype=np.uint8)
            msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
            image_pub.publish(msg)

        # ----------------------------------------------------------------------
        # Case 1: Disabled startup check
        # ----------------------------------------------------------------------
        business = VoiceColorTransport()
        single_exec = SingleThreadedExecutor()
        single_exec.add_node(business)
        single_exec.add_node(probe)

        # Publish initial map markers and frame
        deadline = time.monotonic() + 2
        while len(business._markers) < 5 and time.monotonic() < deadline:
            send_markers()
            send_fresh_red_image()
            spin_until(single_exec, lambda: False, timeout=0.1)
        assert len(business._markers) == 5

        assert business.state == 'idle', f"Initial state should be idle, got {business.state}"
        assert len(arm_targets) == 0, "No arm targets should be commanded at startup"
        assert len(buzzer_msgs) == 0, "No buzzer messages at startup"
        assert len(twist_msgs) == 0, "No twist messages at startup"

        cmd_pub.publish(Int32(data=61))
        spin_until(single_exec, lambda: False, timeout=0.2)
        assert len(arm_targets) == 0, "Command 61 while disabled must not cause motion"

        svc_names_types = business.get_service_names_and_types_by_node(business.get_name(), business.get_namespace())
        assert not any(name.endswith('/CurrentAngle') for name, _ in svc_names_types), \
            "CurrentAngle service must not belong to business"
        assert probe.count_publishers('vel_raw') == 0, "Business must ONLY publish cmd_vel, never vel_raw"

        print("PASS Case 1: Disabled startup, no motion, proper service ownership and QoS subscription", flush=True)

        # ----------------------------------------------------------------------
        # Case 2: Enable transport, command 61, execute pick and drop
        # ----------------------------------------------------------------------
        enable_cli = probe.create_client(SetBool, 'voice_color_transport/set_enabled')
        resp = call_service(single_exec, enable_cli, SetBool.Request(data=True))
        assert resp.success, "Failed to enable transport"

        send_markers()
        send_fresh_red_image()
        spin_until(single_exec, lambda: False, timeout=0.1)

        arm_targets.clear()
        buzzer_msgs.clear()
        playback_msgs.clear()
        twist_msgs.clear()
        arm_service_wrapper['count'] = 0
        mock_nav.request_x_history.clear()

        cmd_pub.publish(Int32(data=61))
        # Immediate duplicate command while busy
        cmd_pub.publish(Int32(data=61))

        # Wait until full sequence completes and returns to idle
        ok = spin_until(single_exec, lambda: business.state == 'idle' and len(mock_nav.request_x_history) >= 2, timeout=10.0)
        assert ok, f"Transport did not finish back to idle, state={business.state}"

        # Assert exactly 5 arm joint targets: full PICK, servo 143, full DROP, servo 30, full PICK
        assert len(arm_targets) == 5, f"Expected 5 arm targets, got {len(arm_targets)}"
        expected_full_pick = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
        expected_full_drop = [90.0, 2.0, 60.0, 40.0, 90.0, 140.0]

        assert arm_targets[0].id == 10 and list(arm_targets[0].joints) == expected_full_pick
        assert arm_targets[1].id == 6 and int(arm_targets[1].angle) == 143
        assert arm_targets[2].id == 10 and list(arm_targets[2].joints) == expected_full_drop
        assert arm_targets[3].id == 6 and int(arm_targets[3].angle) == 30
        assert arm_targets[4].id == 10 and list(arm_targets[4].joints) == expected_full_pick

        # Check physical run_times
        run_times = [m.run_time for m in arm_targets]
        assert run_times == [1000, 500, 1000, 500, 1000], f"Run times altered by time_scale: {run_times}"
        assert list(base.transport.arm_angles) == expected_full_pick, 'Mock base did not apply the actual DDS arm targets'

        # Nav goals: red (1.0) and origin (0.0)
        assert mock_nav.request_x_history == [1.0, 0.0], f"Expected nav goals [1.0, 0.0], got {mock_nav.request_x_history}"

        # Playback: 69 (red) and 65 (drop)
        assert playback_msgs == [69, 65], f"Expected playback [69, 65], got {playback_msgs}"

        # CurrentAngle requests exactly 5, all apply='read'
        assert arm_service_wrapper['count'] == 5, f"Expected 5 CurrentAngle calls, got {arm_service_wrapper['count']}"
        assert arm_service_wrapper['last_req_apply'] == 'read'

        # Twist zero only
        assert all(tw.linear.x == 0.0 and tw.angular.z == 0.0 for tw in twist_msgs)

        # Beeps finite ON/OFF and final False
        assert len(buzzer_msgs) > 0
        assert buzzer_msgs[-1] is False

        print("PASS Case 2: Full color transport flow, arm presets, nav goals, and latch checks", flush=True)

        # ----------------------------------------------------------------------
        # Case 3: Stale image rejection
        # ----------------------------------------------------------------------
        # Wait > 0.5s for stale image
        time.sleep(0.55)
        arm_targets.clear()
        cmd_pub.publish(Int32(data=61))
        spin_until(single_exec, lambda: False, timeout=0.2)
        assert len(arm_targets) == 0, "Stale image must reject transport command"

        # Now send fresh image without new command; ensure no queued pickup starts
        send_fresh_red_image()
        spin_until(single_exec, lambda: False, timeout=0.2)
        assert len(arm_targets) == 0, "No queued pickup should start upon frame arrival without command"

        print("PASS Case 3: Stale image rejection and no latent queue execution", flush=True)

        # ----------------------------------------------------------------------
        # Case 4: Measured invalid pose injection & cancel during service delay
        # ----------------------------------------------------------------------
        arm_service_wrapper['invalid'] = True
        send_fresh_red_image()
        spin_until(single_exec, lambda: False, timeout=0.1)
        cmd_pub.publish(Int32(data=61))
        ok = spin_until(single_exec, lambda: business.state == 'fault', timeout=2.0)
        assert ok, f"State should latch to fault on invalid arm response, got {business.state}"
        assert len(arm_targets) == 0, "No arm commands on fault"

        # Restore fixture & re-enable
        arm_service_wrapper['invalid'] = False
        resp = call_service(single_exec, enable_cli, SetBool.Request(data=True))
        assert resp.success
        assert business.state == 'idle'

        # Delay CurrentAngle by 0.25s
        arm_service_wrapper['delay'] = 0.25
        arm_service_wrapper['count'] = 0
        cancel_cli = probe.create_client(Trigger, 'voice_color_transport/cancel')
        send_fresh_red_image()
        spin_until(single_exec, lambda: False, timeout=0.1)
        cmd_pub.publish(Int32(data=61))

        # Wait until wrapper count increments (service call in progress)
        ok = spin_until(single_exec, lambda: arm_service_wrapper['count'] > 0, timeout=3.0)
        assert ok, "CurrentAngle service call did not start"

        # Cancel via Trigger
        resp = call_service(single_exec, cancel_cli, Trigger.Request())
        assert resp.success

        # Wait until delayed reply arrives and assert no arm command resumes
        spin_until(single_exec, lambda: False, timeout=0.5)
        assert len(arm_targets) == 0, "Arm command executed despite pending cancellation"
        arm_service_wrapper['delay'] = 0.0

        print("PASS Case 4: Fault injection latches disabled and cancel while arm future is pending", flush=True)

        # Exercise the actual finite service deadline rather than a user cancel.
        arm_service_wrapper['delay'] = 0.7
        arm_service_wrapper['count'] = 0
        send_fresh_red_image()
        spin_until(single_exec, lambda: False, timeout=0.1)
        cmd_pub.publish(Int32(data=61))
        assert spin_until(single_exec, lambda: business.state == 'fault', timeout=3.0)
        spin_until(single_exec, lambda: False, timeout=0.8)
        assert not arm_targets, 'A timed out arm response resumed movement'
        arm_service_wrapper['delay'] = 0.0
        print('PASS service deadline: delayed CurrentAngle faults and its late reply cannot publish an arm command', flush=True)

        # Cleanly destroy transport before testing VoiceTakeThings
        single_exec.remove_node(business)
        business.shutdown_cleanup()
        business.destroy_node()
        business = None

        # ----------------------------------------------------------------------
        # Case 5: VoiceTakeThings disabled, route resolution, speech 53 flow
        # ----------------------------------------------------------------------
        business = VoiceTakeThings()
        single_exec.add_node(business)
        deadline = time.monotonic() + 2
        while len(business._markers) < 5 and time.monotonic() < deadline:
            send_markers()
            spin_until(single_exec, lambda: False, timeout=0.1)
        assert len(business._markers) == 5

        arm_targets.clear()
        playback_msgs.clear()
        mock_nav.request_x_history.clear()
        arm_service_wrapper['count'] = 0

        # Commands 53 and 19 while disabled
        cmd_pub.publish(Int32(data=53))
        cmd_pub.publish(Int32(data=19))
        spin_until(single_exec, lambda: False, timeout=0.1)
        assert len(arm_targets) == 0 and len(mock_nav.request_x_history) == 0

        # Enable take things
        enable_take_cli = probe.create_client(SetBool, 'voice_take_things/set_enabled')
        resp = call_service(single_exec, enable_take_cli, SetBool.Request(data=True))
        assert resp.success

        # Speech 53 before nav must not pick, remains idle
        cmd_pub.publish(Int32(data=53))
        spin_until(single_exec, lambda: False, timeout=0.1)
        assert business.state == 'idle'
        assert len(arm_targets) == 0

        # Command 20 initial -> green (X=2)
        cmd_pub.publish(Int32(data=20))
        ok = spin_until(single_exec, lambda: business.state == 'wait_pick', timeout=3.0)
        assert ok, f"Expected state wait_pick, got {business.state}"
        assert mock_nav.request_x_history == [2.0]
        assert len(arm_targets) == 0

        # Command 53 -> PICK + servo 146, playback 34 -> state wait_dest
        cmd_pub.publish(Int32(data=53))
        ok = spin_until(single_exec, lambda: business.state == 'wait_dest' and playback_msgs == [34], timeout=3.0)
        assert ok, f"Expected state wait_dest, got {business.state}"
        assert len(arm_targets) == 2
        assert arm_targets[0].id == 10 and list(arm_targets[0].joints) == expected_full_pick
        assert arm_targets[1].id == 6 and int(arm_targets[1].angle) == 146
        assert playback_msgs == [34]

        # Command 20 post mapping -> yellow (X=4)
        cmd_pub.publish(Int32(data=20))
        # Executes drop: full DROP, servo 30, full PICK, playback 65 -> terminal wait_pick
        ok = spin_until(single_exec, lambda: business.state == 'wait_pick' and len(arm_targets) == 5 and playback_msgs == [34, 65], timeout=5.0)
        assert ok, f"Expected state wait_pick with 5 arm targets, got {business.state}, count={len(arm_targets)}"
        assert mock_nav.request_x_history == [2.0, 4.0]
        assert playback_msgs == [34, 65]
        assert arm_service_wrapper['count'] == 5

        print("PASS Case 5: VoiceTakeThings speech 53 grip, staged route mapping, and drop sequence", flush=True)

        # ----------------------------------------------------------------------
        # Case 6: MockNav reject and abort outcomes
        # ----------------------------------------------------------------------
        # Cancel / reset to idle
        cancel_take_cli = probe.create_client(Trigger, 'voice_take_things/cancel')
        resp = call_service(single_exec, cancel_take_cli, Trigger.Request())
        assert resp.success
        assert business.state == 'idle'

        # Nav reject outcome
        mock_nav.queue_plan(outcome='reject')
        cmd_pub.publish(Int32(data=19))
        ok = spin_until(single_exec, lambda: business.state == 'fault', timeout=2.0)
        assert ok, f"Goal reject should latch fault, state={business.state}"

        # Re-enable
        resp = call_service(single_exec, enable_take_cli, SetBool.Request(data=True))
        assert resp.success

        # Nav abort outcome
        mock_nav.queue_plan(outcome='abort')
        cmd_pub.publish(Int32(data=19))
        ok = spin_until(single_exec, lambda: business.state == 'fault', timeout=2.0)
        assert ok, f"Goal abort should latch fault, state={business.state}"

        print("PASS Case 6: MockNav reject and abort trigger fault and never reach wait_pick", flush=True)

        # ----------------------------------------------------------------------
        # Case 7: Late accepted goal cancel
        # ----------------------------------------------------------------------
        resp = call_service(single_exec, enable_take_cli, SetBool.Request(data=True))
        assert resp.success

        mock_nav.queue_plan(outcome='hold', accept_delay=0.3)
        init_cancels = mock_nav.cancel_count
        initial_requests = len(mock_nav.request_x_history)
        arm_targets.clear()

        cmd_pub.publish(Int32(data=19))
        # Wait for goal callback to begin
        ok = spin_until(single_exec, lambda: len(mock_nav.request_x_history) > initial_requests, timeout=1.0)
        assert ok

        # Immediately trigger cancel
        resp = call_service(single_exec, cancel_take_cli, Trigger.Request())
        assert resp.success

        # Wait for mock cancel count to increment
        ok = spin_until(single_exec, lambda: mock_nav.cancel_count > init_cancels, timeout=3.0)
        assert ok, f"Mock cancel count did not increment: {mock_nav.cancel_count} vs {init_cancels}"
        assert business.state == 'idle'
        assert len(arm_targets) == 0

        print("PASS Case 7: Late accepted goal cancellation without leaking active handle", flush=True)

        # ----------------------------------------------------------------------
        # Case 8: Status fault while holding navigation
        # ----------------------------------------------------------------------
        resp = call_service(single_exec, enable_take_cli, SetBool.Request(data=True))
        assert resp.success
        assert spin_until(single_exec, lambda: not mock_nav.active_goals, timeout=1.0)

        mock_nav.queue_plan(outcome='hold')
        cmd_pub.publish(Int32(data=19))

        # Wait until accepted
        ok = spin_until(single_exec, lambda: len(mock_nav.active_goals) > 0, timeout=2.0)
        assert ok

        init_cancels = mock_nav.cancel_count
        status_pub.publish(String(data='disconnect'))

        ok = spin_until(single_exec, lambda: business.state == 'fault', timeout=2.0)
        assert ok, f"Status disconnect did not latch fault, state={business.state}"
        assert spin_until(single_exec, lambda: mock_nav.cancel_count > init_cancels, timeout=1.0), "Disconnect did not cancel active nav goal"

        # Ensure new command ignored while disabled
        cmd_pub.publish(Int32(data=20))
        spin_until(single_exec, lambda: False, timeout=0.2)
        assert business.state == 'fault'

        print("PASS Case 8: Voice status disconnect triggers latched fault, goal cancel, and stops", flush=True)

        # Held accepted navigation must fault at the configured result deadline.
        assert spin_until(single_exec, lambda: not mock_nav.active_goals, timeout=1.0)
        assert call_service(single_exec, enable_take_cli, SetBool.Request(data=True)).success
        before_cancel = mock_nav.cancel_count
        mock_nav.queue_plan(outcome='hold')
        cmd_pub.publish(Int32(data=19))
        assert spin_until(single_exec, lambda: business.state == 'fault', timeout=3.0)
        assert spin_until(single_exec, lambda: mock_nav.cancel_count > before_cancel, timeout=1.0)
        assert not arm_targets
        print('PASS Nav2 result deadline: held mock goal canceled, fault latched, no arm continuation', flush=True)

        assert spin_until(single_exec, lambda: not mock_nav.active_goals, timeout=1.0)
        assert call_service(single_exec, enable_take_cli, SetBool.Request(data=True)).success
        before_cancel = mock_nav.cancel_count
        mock_nav.queue_plan(outcome='hold', accept_delay=0.8)
        cmd_pub.publish(Int32(data=19))
        assert spin_until(single_exec, lambda: business.state == 'fault', timeout=2.0)
        assert spin_until(single_exec, lambda: mock_nav.cancel_count > before_cancel, timeout=2.0)
        assert not arm_targets
        print('PASS Nav2 acceptance deadline: late accepted mock handle canceled after fault, no stale work', flush=True)

    finally:
        # Cleanup
        if multi_exec is not None:
            multi_exec.shutdown(timeout_sec=4.0)
        if multi_thread is not None and multi_thread.is_alive():
            multi_thread.join(timeout=1.0)
        if business is not None:
            try:
                business.shutdown_cleanup()
            except Exception:
                pass
            try:
                business.destroy_node()
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

        if base is not None:
            try:
                base.close()
            except Exception:
                pass
            try:
                base.destroy_node()
            except Exception:
                pass

        if multi_exec is not None:
            try:
                multi_exec.shutdown()
            except Exception:
                pass

        if multi_thread is not None and multi_thread.is_alive():
            multi_thread.join(timeout=1.0)

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
