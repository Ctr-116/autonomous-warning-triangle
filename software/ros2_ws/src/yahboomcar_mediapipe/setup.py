from glob import glob
import os

from setuptools import setup

package_name = 'yahboomcar_mediapipe'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'file'), glob('file/*.xml') + glob('file/*.dat')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='ROS2 Humble port of yahboomcar_mediapipe',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'HandDetector = yahboomcar_mediapipe.detectors:HandDetector_main',
            'PoseDetector = yahboomcar_mediapipe.detectors:PoseDetector_main',
            'Holistic = yahboomcar_mediapipe.detectors:Holistic_main',
            'FaceMesh = yahboomcar_mediapipe.detectors:FaceMesh_main',
            'FaceEyeDetection = yahboomcar_mediapipe.detectors:FaceEyeDetection_main',
            'FaceLandmarks = yahboomcar_mediapipe.detectors:FaceLandmarks_main',
            'FaceDetection = yahboomcar_mediapipe.detectors:FaceDetection_main',
            'Objectron = yahboomcar_mediapipe.detectors:Objectron_main',
            'VirtualPaint = yahboomcar_mediapipe.detectors:VirtualPaint_main',
            'HandCtrl = yahboomcar_mediapipe.detectors:HandCtrl_main',
            'GestureRecognition = yahboomcar_mediapipe.detectors:GestureRecognition_main',
            'mediapipe_point = yahboomcar_mediapipe.mediapipe_point:main',
        ],
    },
)
