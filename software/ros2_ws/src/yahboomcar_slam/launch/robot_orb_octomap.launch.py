# Robot + native PCL mapper + optional OctoMap server (official octomap_server).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    run_octomap = LaunchConfiguration('run_octomap').perform(context).lower() in ('1', 'true', 'yes')
    actions = [Node(
        package='yahboomcar_slam', executable='pointcloud_mapping',
        name='pointcloud_mapping', output='screen',
        parameters=[LaunchConfiguration('mapping_params')],
        remappings=[
            ('/RGBD/RGB/Image', LaunchConfiguration('color_topic')),
            ('/RGBD/Depth/Image', LaunchConfiguration('depth_topic')),
            ('/RGBD/CameraPose', LaunchConfiguration('pose_topic')),
        ])]
    if run_octomap:
        actions.append(Node(
            package='octomap_server', executable='octomap_server_node',
            name='octomap_server', output='screen',
            parameters=[{'frame_id': LaunchConfiguration('map_frame'),
                         'resolution': LaunchConfiguration('octomap_resolution')}],
            remappings=[('cloud_in', '/pointcloud_mapping/Global/PointCloudOutput')]))
    else:
        actions.append(LogInfo(msg='OctoMap server not requested (run_octomap:=false).'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mapping_params', default_value=''),
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('pose_topic', default_value='/RGBD/CameraPose'),
        DeclareLaunchArgument('run_octomap', default_value='false'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('octomap_resolution', default_value='0.05'),
        OpaqueFunction(function=_launch),
    ])
