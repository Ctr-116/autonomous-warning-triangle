from setuptools import setup
from glob import glob
import os

package_name = 'yahboomcar_linefollw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='ROS2 Humble port of Yahboom color line follower.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_line = yahboomcar_linefollw.follow_line:main',
        ],
    },
)
