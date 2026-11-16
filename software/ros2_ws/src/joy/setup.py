from glob import glob
import os

from setuptools import setup

package_name = 'joy'

setup(
    name=package_name,
    version='1.15.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name), [
            'README.md',
            'MIGRATION.md',
            'LICENSE',
        ]),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Bohren',
    maintainer_email='jbo@jhu.edu',
    author='Morgan Quigley, Brian Gerkey, Kevin Watts, Blaise Gassend, furushchev',
    description=(
        'ROS 2 Humble PS4 remapping helper for the upstream joy_linux driver'
    ),
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_remap = joy.joy_remap:main',
        ],
    },
)
