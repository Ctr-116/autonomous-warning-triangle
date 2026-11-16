from glob import glob
from setuptools import find_packages, setup

setup(
    name='x3plus_base', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/x3plus_base']),
        ('share/x3plus_base', ['package.xml']),
        ('share/x3plus_base/config', glob('config/*.yaml')),
        ('share/x3plus_base/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Project maintainers', maintainer_email='maintainer@example.invalid',
    description='ROS 2 X3plus chassis adapter', license='MIT',
    entry_points={'console_scripts': ['base_node = x3plus_base.base_node:main']},
)
