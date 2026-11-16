from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    img_flip = LaunchConfiguration('img_flip')
    gui = LaunchConfiguration('gui')
    image_topic = LaunchConfiguration('image_topic')
    kinematics_service = LaunchConfiguration('kinematics_service')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('img_flip', default_value='false',
                              description='Flip incoming image horizontally'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Enable OpenCV GUI (keyboard/ROI mouse)'),
        DeclareLaunchArgument('image_topic', default_value='image_raw',
                              description='Image topic to subscribe (BEST_EFFORT)'),
        DeclareLaunchArgument('kinematics_service', default_value='kinematics',
                              description='External Kinematics service name'),
        DeclareLaunchArgument('config_file', default_value='',
                              description='Optional parameter YAML override'),
        Node(
            package='arm_autopilot',
            executable='autopilot_main',
            name='arm_autopilot',
            output='screen',
            parameters=[
                {'img_flip': img_flip},
                {'gui': gui},
                {'image_topic': image_topic},
                {'kinematics_service': kinematics_service},
            ] if config_file == '' else [
                config_file,
                {'img_flip': img_flip},
                {'gui': gui},
                {'image_topic': image_topic},
                {'kinematics_service': kinematics_service},
            ],
        ),
    ])
