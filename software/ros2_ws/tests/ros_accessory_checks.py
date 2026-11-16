"""Actual ROS2 -> sole real SDK/pyserial owner -> explicit PTY firmware fixture."""
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'/'x3plus_base'))
sys.path.insert(0,str(ROOT/'vendor'/'py_install'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import UInt32, Int32, Float32
from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray
from x3plus_base.base_node import BaseNode
from serial_accessory_emulator import AccessoryFirmware


def spin(executor, seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        executor.spin_once(timeout_sec=0.01)


def call(executor, client, request):
    end=time.monotonic()+2
    while not client.service_is_ready() and time.monotonic()<end:
        executor.spin_once(timeout_sec=0.01)
    assert client.service_is_ready(),client.srv_name
    future=client.call_async(request)
    while not future.done() and time.monotonic()<end:
        executor.spin_once(timeout_sec=0.01)
    assert future.done(),client.srv_name
    return future.result()


def main():
    firmware=AccessoryFirmware().start()
    rclpy.init(args=['--ros-args','-r','__ns:=/real_accessory_test',
                     '-p','use_mock:=false','-p','port:='+firmware.port])
    executor=SingleThreadedExecutor()
    base=None
    probe=rclpy.create_node('real_accessory_probe')
    executor.add_node(probe)
    try:
        base=BaseNode()
        executor.add_node(base)
        measured,editions=[],[]
        probe.create_subscription(ArmJoint,'ArmAngleUpdate',lambda msg:measured.append(list(msg.joints)),100)
        probe.create_subscription(Float32,'edition',lambda msg:editions.append(msg.data),10)
        arm=probe.create_publisher(ArmJoint,'TargetAngle',10)
        rgb=probe.create_publisher(UInt32,'RGBColor',10)
        effect=probe.create_publisher(Int32,'RGBLight',10)
        angles=probe.create_client(RobotArmArray,'CurrentAngle')
        params=probe.create_client(SetParameters,'x3plus_base/set_parameters')
        spin(executor,0.8)
        response=call(executor,angles,RobotArmArray.Request(apply='read'))
        assert list(response.angles)==[90.0]*6 and measured
        assert editions and abs(editions[-1]-3.3)<1e-4
        assert base.cfg['use_mock'] is False
        print('PASS real serial ROS driver: CurrentAngle/ArmAngleUpdate and firmware version derive from actual PTY replies',flush=True)
        before=len([f for f,_,_ in firmware.commands if f in (0x20,0x23)])
        arm.publish(ArmJoint(id=10,joints=[100.0,90.0,90.0,90.0,90.0,120.0],run_time=500))
        spin(executor,0.15)
        assert len([f for f,_,_ in firmware.commands if f in (0x20,0x23)])==before
        request=SetParameters.Request(parameters=[Parameter(name='enable_arm',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=True))])
        assert call(executor,params,request).results[0].successful
        assert base.cfg['enable_arm'] is True
        arm.publish(ArmJoint(id=10,joints=[100.0,90.0,90.0,90.0,90.0,120.0],run_time=500))
        spin(executor,0.4)
        response=call(executor,angles,RobotArmArray.Request(apply='read'))
        assert abs(response.angles[0]-100)<1 and abs(response.angles[5]-120)<1,list(response.angles)
        print('PASS disabled arm blocks bytes; actual ROS parameter enable opens gate; command/query returns measured replies',flush=True)
        rgb.publish(UInt32(data=0x123456))
        effect.publish(Int32(data=1))
        spin(executor,0.3)
        assert firmware.lamps==(255,18,52,86) and firmware.effect==(1,6,1)
        print('PASS ROS RGBColor/RGBLight -> actual SDK serial payloads',flush=True)
        request.parameters[0].value.bool_value=False
        assert call(executor,params,request).results[0].successful
        before=len([f for f,_,_ in firmware.commands if f in (0x20,0x23)])
        arm.publish(ArmJoint(id=1,angle=110.0,run_time=250))
        spin(executor,0.15)
        assert len([f for f,_,_ in firmware.commands if f in (0x20,0x23)])==before
        firmware.answer_arm_queries=False
        spin(executor,0.45)
        assert not call(executor,angles,RobotArmArray.Request(apply='read')).angles
        assert base.transport.snapshot().motion_time is not None
        assert firmware.error is None and base.transport.error is None
        print('PASS runtime disable blocks new arm bytes; fresh drive feedback cannot conceal stale measured joint feedback',flush=True)
    finally:
        if base is not None:
            base.close()
        executor.shutdown()
        if base is not None:
            base.destroy_node()
        probe.destroy_node()
        rclpy.try_shutdown()
        firmware.close()


if __name__=='__main__':
    main()
