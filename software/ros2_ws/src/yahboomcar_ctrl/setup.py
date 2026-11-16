from glob import glob
import os

from setuptools import setup

package_name = 'yahboomcar_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), ['README.md', 'MIGRATION.md']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='ROS 2 Humble Yahboom teleoperation controllers',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yahboom_keyboard = yahboomcar_ctrl.yahboom_keyboard:main',
            'yahboom_joy = yahboomcar_ctrl.yahboom_joy:main',
            'turtlebot_joy = yahboomcar_ctrl.turtlebot_joy:main',
            'twist_joy = yahboomcar_ctrl.twist_joy:main',
        ],
    },
)
