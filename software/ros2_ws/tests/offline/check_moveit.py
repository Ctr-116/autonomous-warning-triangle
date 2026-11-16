"""Real Humble MoveIt planning/execution against ros2_control mock hardware."""

from support import ARTIFACT_DIR, WORKSPACE_ROOT
import os
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def wait_future(node, future, seconds=15):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert future.done(), 'Future deadline expired'
    return future.result()


def main():
    log = open(str(ARTIFACT_DIR) + '/moveit_behavior_launch.log', 'w')
    child = subprocess.Popen(['ros2', 'launch', 'x3plus_moveit_config', 'demo.launch.py', 'rviz:=false'],
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('moveit_behavior_validation')
    states = {}
    sub = node.create_subscription(JointState, 'joint_states',
                                   lambda msg: states.update(zip(msg.name, msg.position)), 10)
    arm_names = ['arm_joint%d' % index for index in range(1, 6)]
    plan_client = node.create_client(GetMotionPlan, 'plan_kinematic_path')
    execute_client = ActionClient(node, ExecuteTrajectory, 'execute_trajectory')
    arm_client = ActionClient(node, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
    grip_client = ActionClient(node, FollowJointTrajectory, 'gripper_controller/follow_joint_trajectory')
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (plan_client.service_is_ready() and arm_client.server_is_ready()
                    and grip_client.server_is_ready() and execute_client.server_is_ready()
                    and all(name in states for name in arm_names)):
                break
            assert child.poll() is None, 'MoveIt launch exited'
        assert plan_client.service_is_ready(), 'Planning service unavailable'
        assert all(name in states for name in arm_names), 'Joint feedback missing'
        assert node.count_publishers('/joint_states') == 1, 'Multiple joint state publishers'
        print('PASS MoveIt service/actions + active arm/gripper controllers + single measured mock state source', flush=True)

        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = 'arm_group'
        motion.num_planning_attempts = 3
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.2
        motion.max_acceleration_scaling_factor = 0.2
        motion.start_state.is_diff = True
        goal_positions = [0.15, 0.15, 0.0, -0.15, 0.0]
        constraints = Constraints()
        for name, position in zip(arm_names, goal_positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        motion.goal_constraints = [constraints]
        response = wait_future(node, plan_client.call_async(request))
        assert response.motion_plan_response.error_code.val == 1, response.motion_plan_response.error_code.val
        trajectory = response.motion_plan_response.trajectory
        assert len(trajectory.joint_trajectory.points) >= 2, 'Empty planned trajectory'
        print('PASS actual OMPL GetMotionPlan joint-space planning', flush=True)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle = wait_future(node, execute_client.send_goal_async(goal))
        assert handle.accepted
        result = wait_future(node, handle.get_result_async(), 20)
        assert result.status == 4 and result.result.error_code.val == 1, result
        for _ in range(5): rclpy.spin_once(node, timeout_sec=0.05)
        assert all(abs(states[name] - target) < 0.04 for name, target in zip(arm_names, goal_positions)), states
        print('PASS ExecuteTrajectory -> actual ros2_control mock joint feedback reaches goal', flush=True)

        grip_goal = FollowJointTrajectory.Goal()
        grip_goal.trajectory.joint_names = ['grip_joint']
        point = JointTrajectoryPoint()
        point.positions = [-0.2]
        point.time_from_start.sec = 1
        grip_goal.trajectory.points = [point]
        grip_handle = wait_future(node, grip_client.send_goal_async(grip_goal))
        assert grip_handle.accepted
        grip_result = wait_future(node, grip_handle.get_result_async(), 5)
        assert grip_result.status == 4 and grip_result.result.error_code == 0, grip_result
        print('PASS gripper FollowJointTrajectory execution', flush=True)

        cancel_goal = FollowJointTrajectory.Goal()
        cancel_goal.trajectory.joint_names = arm_names
        point = JointTrajectoryPoint()
        point.positions = [0.0] * 5
        point.time_from_start.sec = 4
        cancel_goal.trajectory.points = [point]
        cancel_handle = wait_future(node, arm_client.send_goal_async(cancel_goal))
        assert cancel_handle.accepted
        for _ in range(3): rclpy.spin_once(node, timeout_sec=0.05)
        canceled = wait_future(node, cancel_handle.cancel_goal_async())
        assert canceled.goals_canceling, 'Cancel request rejected'
        cancel_result = wait_future(node, cancel_handle.get_result_async(), 5)
        assert cancel_result.status == 5, cancel_result.status
        print('PASS actual arm trajectory action cancellation', flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        child.send_signal(signal.SIGINT)
        try: child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait(timeout=3)
        log.close()


if __name__ == '__main__':
    main()
