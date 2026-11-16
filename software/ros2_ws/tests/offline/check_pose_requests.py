"""Regression for pose xyz ignored and invalid BoundingVolume in Python demo."""

from support import ARTIFACT_DIR, WORKSPACE_ROOT
from types import SimpleNamespace
import copy
import math
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetMotionPlan
from rclpy.serialization import serialize_message, deserialize_message
from arm_moveit_demo.demo import MoveItDemoNode

captured = []
response = GetMotionPlan.Response()
response.motion_plan_response.error_code.val = 1
client = SimpleNamespace(call_async=lambda request: captured.append(request))
node = SimpleNamespace(_plan_client=client, timeout=2,
                       _wait_service=lambda *args: True,
                       _spin_until=lambda *args: response,
                       get_logger=lambda: SimpleNamespace(error=lambda *args: None))
pose = Pose()
pose.position.x, pose.position.y, pose.position.z = 0.1, -0.2, 0.3
pose.orientation.w = 2.0
MoveItDemoNode.plan_pose(node, pose)
request = deserialize_message(serialize_message(captured[-1]), GetMotionPlan.Request)
goal = request.motion_plan_request.goal_constraints[0]
assert len(goal.position_constraints) == 1 and len(goal.orientation_constraints) == 1
pc = goal.position_constraints[0]
region = pc.constraint_region.primitive_poses[0]
assert (region.position.x, region.position.y, region.position.z) == (0.1, -0.2, 0.3)
assert region.orientation.w == 1.0 and pc.constraint_region.primitives[0].dimensions[0] == 0.005
assert math.isclose(goal.orientation_constraints[0].orientation.w, 1.0)
print('PASS requested xyz + normalized quaternion preserved in actual Humble serialized GetMotionPlan request')
for kind in ('nan_position', 'inf_orientation', 'zero_quaternion', 'bad_tolerance'):
    invalid = copy.deepcopy(pose)
    tol = 0.005
    if kind == 'nan_position': invalid.position.x = float('nan')
    if kind == 'inf_orientation': invalid.orientation.z = float('inf')
    if kind == 'zero_quaternion': invalid.orientation.w = 0.0
    if kind == 'bad_tolerance': tol = -1.0
    before = len(captured)
    assert MoveItDemoNode.plan_pose(node, invalid, tolerance=tol) is None
    assert len(captured) == before
print('PASS invalid pose/tolerance rejected before any service request')
