import os
from glob import glob

from setuptools import setup

package_name = 'yahboomcar_multi'

setup(
    name=package_name,
    version='0.0.1',
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
    description='Multi-robot formation control ported from ROS1 yahboomcar_multi',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broad_queue = yahboomcar_multi.broad_queue:main',
            'broad_round = yahboomcar_multi.broad_round:main',
            'listener = yahboomcar_multi.listener:main',
        ],
    },
)
