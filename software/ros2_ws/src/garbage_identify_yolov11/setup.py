from glob import glob
from setuptools import setup

package_name = 'garbage_identify_yolov11'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements-onnx.txt']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.onnx') + glob('models/*.pt') + glob('models/*.yaml') + glob('models/*.json') + glob('models/*.md')),
        ('share/' + package_name + '/models/samples', glob('models/samples/*.jpg') + glob('models/samples/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chen',
    maintainer_email='chen@todo.todo',
    description='Native ROS2 Humble migration of garbage_identify_yolov11.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'garbage_identify_yolov11 = garbage_identify_yolov11.garbage_identify_yolov11:main',
            'voice_ctrl_take_garbage = garbage_identify_yolov11.voice_ctrl_take_garbage:main',
            'detector = garbage_identify_yolov11.detector:main',
        ],
    },
)
