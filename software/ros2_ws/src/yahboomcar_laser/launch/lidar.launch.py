"""Optional upstream lidar driver selector.

The ROS 1 package included ``ydlidar_ros_driver/launch/TG.launch``.  The
actual ROSMASTER X3plus sensor model, serial port and baud rate are not known
from the source tree, so this launch file does not hard-code them as if they
were confirmed hardware.  It starts one of the two common ROS 2 drivers only
when explicitly requested, and leaves ``driver:=none`` as the default.

The application launch files work with ``driver:=none``: they subscribe to
``scan`` and publish a zero command until some other process (a real driver or
a test publisher) supplies fresh, valid LaserScan data.
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_DEFAULTS = {
    "ydlidar": ("ydlidar_ros2_driver", "ydlidar_launch.py"),
    "sllidar": ("sllidar_ros2", "sllidar_launch.py"),
}


def _text(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _launch_driver(context, *_args, **_kwargs):
    driver = _text(context, "driver").lower()
    if driver == "none":
        return [
            LogInfo(
                msg=(
                    "lidar.launch.py: no lidar driver started (driver:=none). "
                    "The laser application will publish zero until a fresh, "
                    "valid LaserScan arrives on its scan_topic. Use a real ROS "
                    "2 driver or a clearly-labelled synthetic test publisher."
                )
            )
        ]

    if driver not in _DEFAULTS:
        raise RuntimeError(
            f"unknown lidar driver '{driver}' (expected none, ydlidar or "
            "sllidar)"
        )

    default_package, default_file = _DEFAULTS[driver]
    package_name = (
        _text(context, "driver_launch_package") or default_package
    )
    launch_file = _text(context, "driver_launch_file") or default_file
    port = _text(context, "port")
    frame_id = _text(context, "frame_id")
    params_file = _text(context, "params_file")

    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"the selected {driver} driver package '{package_name}' is not "
            "installed. Install it separately or set "
            "driver_launch_package to the installed package name."
        ) from exc

    launch_path = launch_file
    if not os.path.isabs(launch_path):
        launch_path = os.path.join(package_share, "launch", launch_file)
    if not os.path.isfile(launch_path):
        raise RuntimeError(
            f"driver launch file not found: {launch_path}. Set "
            "driver_launch_file to a model-specific launch file shipped by "
            "the selected driver package."
        )

    launch_arguments = {}
    if frame_id:
        # This frame must match the robot URDF/TF; 'laser' is only a launch
        # default, not a claim about the installed hardware.
        launch_arguments["frame_id"] = frame_id

    if driver == "ydlidar":
        if port:
            launch_arguments["port"] = port
        if params_file:
            launch_arguments["params_file"] = params_file
    else:  # sllidar
        if port:
            launch_arguments["serial_port"] = port

    notes = [
        f"lidar.launch.py: starting upstream {driver} driver from "
        f"{package_name}/{launch_file}."
    ]
    if driver == "ydlidar" and not params_file:
        notes.append(
            "No params_file supplied: the upstream driver's own defaults "
            "(including device path and model) will apply; verify them for "
            "your hardware."
        )
    if driver == "sllidar" and not port:
        notes.append(
            "No serial port supplied: the upstream driver's default port "
            "will apply; override 'port' for your hardware."
        )
    notes.append(
        "The driver is expected to publish a LaserScan topic named 'scan'. "
        "If it publishes another name, set the application's scan_topic "
        "accordingly."
    )

    return [
        LogInfo(msg=" ".join(notes)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments=launch_arguments.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "driver",
                default_value="none",
                description=(
                    "Upstream lidar driver to start: none, ydlidar "
                    "(YDLIDAR ROS 2 driver) or sllidar (sllidar_ros2)."
                ),
            ),
            DeclareLaunchArgument(
                "driver_launch_package",
                default_value="",
                description=(
                    "Override the upstream package name. Defaults to "
                    "ydlidar_ros2_driver or sllidar_ros2."
                ),
            ),
            DeclareLaunchArgument(
                "driver_launch_file",
                default_value="",
                description=(
                    "Override the upstream launch file name/path. Defaults "
                    "to ydlidar_launch.py or sllidar_launch.py."
                ),
            ),
            DeclareLaunchArgument(
                "port",
                default_value="",
                description=(
                    "Optional serial device. Empty means use the upstream "
                    "driver's default, which this package does not claim to "
                    "know."
                ),
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="laser",
                description=(
                    "Laser frame id passed to the upstream driver. Must match "
                    "the robot URDF/TF."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description=(
                    "Optional YDLIDAR parameter YAML. Empty means use the "
                    "upstream driver's defaults."
                ),
            ),
            OpaqueFunction(function=_launch_driver),
        ]
    )
