import os
from glob import glob

from setuptools import setup

package_name = "yahboomcar_laser"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description=(
        "ROS 2 Humble lidar avoidance, tracker and warning applications for "
        "the Yahboom ROSMASTER X3plus"
    ),
    license='TODO',
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "laser_avoidance = yahboomcar_laser.laser_avoidance:main",
            "laser_tracker = yahboomcar_laser.laser_tracker:main",
            "laser_warning = yahboomcar_laser.laser_warning:main",
        ],
    },
)
