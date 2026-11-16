"""Exercise generated ROS2 interfaces with real serialization and a DDS service."""

from support import ARTIFACT_DIR, WORKSPACE_ROOT
import time
import rclpy
from rclpy.serialization import serialize_message, deserialize_message
from geometry_msgs.msg import Point
from yahboomcar_msgs.msg import ArmJoint, ImageMsg, PointArray, Position, Target, TargetArray
from yahboomcar_msgs.srv import Kinematics, RobotArmArray


def main():
    messages = [
        Position(angle_x=1.25, angle_y=-2.5, distance=3.0),
        PointArray(points=[Point(x=1.0, y=2.0, z=3.0)]),
        ImageMsg(height=1, width=2, channels=1, data=[12, 34]),
        Target(frame_id='camera', ptx=12.0, pty=34.0),
        TargetArray(data=[Target(frame_id='camera')]),
        ArmJoint(id=1, angle=90.0, run_time=500, joints=[90.0] * 6),
        RobotArmArray.Request(apply='read'),
        RobotArmArray.Response(angles=[90.0] * 6),
        Kinematics.Request(kin_name='ik', src_pos=[0.1, 0.2, 0.3]),
        Kinematics.Response(target_joints=[0.1] * 5),
    ]
    messages[3].stamp.sec, messages[3].stamp.nanosec = 12, 345
    for msg in messages:
        decoded = deserialize_message(serialize_message(msg), type(msg))
        assert msg == decoded, type(msg).__name__
    print('PASS serialization roundtrip: 6 messages + 4 service request/response types')
    rclpy.init()
    server = rclpy.create_node('interface_service_server')
    client_node = rclpy.create_node('interface_service_client')
    service = server.create_service(RobotArmArray, 'validation/current_angle',
                                    lambda req, res: serve_angles(req, res))
    client = client_node.create_client(RobotArmArray, 'validation/current_angle')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(client_node)
    try:
        deadline = time.monotonic() + 10.0
        while not client.service_is_ready() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert client.service_is_ready(), 'DDS service discovery timeout'
        future = client.call_async(RobotArmArray.Request(apply='read'))
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert future.done(), 'DDS service response timeout'
        assert list(future.result().angles) == [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
        print('PASS actual DDS RobotArmArray request/response')
    finally:
        executor.shutdown()
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def serve_angles(request, response):
    assert request.apply == 'read'
    response.angles = [90.0, 145.0, 0.0, 45.0, 90.0, 30.0]
    return response


if __name__ == '__main__':
    main()
