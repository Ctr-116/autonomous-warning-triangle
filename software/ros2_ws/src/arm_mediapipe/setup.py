from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_mediapipe'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/README.md', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='ROS2 Humble port of arm_mediapipe.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_ctrl = arm_mediapipe.hand_ctrl:main',
            'finger_ctrl = arm_mediapipe.finger_ctrl:main',
            'pose_arm = arm_mediapipe.pose_arm:main',
            'arm_ctrl = arm_mediapipe.arm_ctrl:main',
            'robot_ctrl = arm_mediapipe.robot_ctrl:main',
            'msg_to_img = arm_mediapipe.msg_to_img:main',
            '01_HandCtrlArm = arm_mediapipe.hand_ctrl:main',
            '02_PoseCtrlArm = arm_mediapipe.pose_arm:main',
            'ArmCtrl = arm_mediapipe.arm_ctrl:main',
            'FingerCtrl = arm_mediapipe.finger_ctrl:main',
            'HandCtrl = arm_mediapipe.hand_ctrl:main',
            'PoseArm = arm_mediapipe.pose_arm:main',
            'RobotCtrl = arm_mediapipe.robot_ctrl:main',
            'msgToimg = arm_mediapipe.msg_to_img:main',
        ],
    },
)
