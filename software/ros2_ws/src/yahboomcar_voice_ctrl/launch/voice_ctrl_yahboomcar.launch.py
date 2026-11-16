"""ROS 2 replacement for original voice_ctrl_yahboomcar.launch."""
from yahboomcar_voice_ctrl.compat_launch import compatibility_description

def generate_launch_description():
    return compatibility_description('base', navigation=False)
