from setuptools import setup
from glob import glob

package_name = 'yahboomcar_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='ROS 2 Humble bringup/calibration/patrol/velocity mux for X3plus.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_linear = yahboomcar_bringup.calibration:calibrate_linear_main',
            'calibrate_angular = yahboomcar_bringup.calibration:calibrate_angular_main',
            'patrol = yahboomcar_bringup.patrol:main',
            'velocity_mux = yahboomcar_bringup.velocity_mux:main',
        ],
    },
)
