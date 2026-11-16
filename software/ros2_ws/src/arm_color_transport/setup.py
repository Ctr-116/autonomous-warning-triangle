from setuptools import setup

package_name = 'arm_color_transport'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/transport_base.launch.py',
          'launch/transport_rviz.launch.py']),
        ('share/' + package_name + '/config', ['config/defaults.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='Color-based arm transport (ROS1 to ROS2 Humble port)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_marker = arm_color_transport.draw_marker:main',
            'transport_main = arm_color_transport.transport_main:main',
            'transport_common = arm_color_transport.transport_common:main',
        ],
    },
)
