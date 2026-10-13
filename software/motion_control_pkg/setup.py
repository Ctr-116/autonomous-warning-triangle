from setuptools import setup
from glob import glob
import os

package_name = 'motion_control_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cuiyang',
    maintainer_email='your@email.com',
    description='Motion test and PID tuning for ROSMASTER X3 PLUS',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motion_test_node = motion_control_pkg.motion_test_node:main',
            'test_cli = motion_control_pkg.test_cli:main',
        ],
    },
)