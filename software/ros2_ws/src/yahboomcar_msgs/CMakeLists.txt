cmake_minimum_required(VERSION 3.8)
project(yahboomcar_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Position.msg"
        "msg/PointArray.msg"
        "msg/ImageMsg.msg"
        "msg/Target.msg"
        "msg/TargetArray.msg"
        "msg/ArmJoint.msg"
        "srv/RobotArmArray.srv"
        "srv/Kinematics.srv"
        DEPENDENCIES
        builtin_interfaces
        geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
