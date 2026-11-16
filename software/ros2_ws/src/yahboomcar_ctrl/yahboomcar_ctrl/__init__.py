"""ROS 2 Humble teleoperation controllers for Yahboom chassis.

The four original ROS 1 scripts are preserved as native rclpy console
entrypoints:

* ``yahboom_keyboard.py`` -> ``yahboom_keyboard``
* ``yahboom_joy.py``      -> ``yahboom_joy``
* ``turtlebot_joy.py``    -> ``turtlebot_joy``
* ``twist_joy.py``        -> ``twist_joy``

Pure state machines live in ``joy_core``, ``arm_core`` and ``keyboard_core`` so
edge-triggering, timeout, bounds, and evaluator behavior can be unit tested
without ROS.
"""

__version__ = '0.0.0'
