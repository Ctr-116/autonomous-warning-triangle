"""ROS 2 replacement for original voice_transport_base.launch."""
from yahboomcar_voice_ctrl.compat_launch import compatibility_description

def generate_launch_description():
    return compatibility_description('transport', navigation=True)
