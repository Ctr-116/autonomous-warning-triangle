from glob import glob
from setuptools import find_packages, setup
setup(name='yahboomcar_voice_ctrl', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/yahboomcar_voice_ctrl']),
                  ('share/yahboomcar_voice_ctrl', ['package.xml']),
                  ('share/yahboomcar_voice_ctrl/launch', glob('launch/*.launch.py')),
                  ('share/yahboomcar_voice_ctrl/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'speech_transport = yahboomcar_voice_ctrl.speech_transport:main',
          'voice_base = yahboomcar_voice_ctrl.voice_base:main',
          'voice_arm = yahboomcar_voice_ctrl.voice_arm:main',
          'voice_arm_study = yahboomcar_voice_ctrl.voice_arm_study:main',
          'voice_line = yahboomcar_voice_ctrl.voice_line:main',
          'voice_color_tracker = yahboomcar_voice_ctrl.voice_color_tracker:main',
          'voice_color_identify = yahboomcar_voice_ctrl.voice_color_identify:main',
          'voice_color_transport = yahboomcar_voice_ctrl.voice_business:main_transport',
          'voice_take_things = yahboomcar_voice_ctrl.voice_business:main_take_things',
          'voice_waypoints = yahboomcar_voice_ctrl.voice_waypoints:main']})
