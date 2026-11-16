"""Validate installed business launches and all seven legacy entry names."""

from support import ARTIFACT_DIR, WORKSPACE_ROOT
from pathlib import Path
import os
import signal
import subprocess
import time
import rclpy


def main():
    compatibility = ['voice_ctrl_arm_study', 'voice_ctrl_arm', 'voice_ctrl_colorTracker',
                     'voice_ctrl_followline', 'voice_ctrl_takethings', 'voice_ctrl_yahboomcar',
                     'voice_transport_base']
    for name in compatibility:
        result = subprocess.run(['ros2', 'launch', 'yahboomcar_voice_ctrl', name + '.launch.py', '--show-args'],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=15)
        assert result.returncode == 0, (name, result.stdout[-3000:])
        assert 'namespace' in result.stdout and 'enable_serial' in result.stdout
        print('PASS installed original launch replacement: ' + name, flush=True)

    rclpy.init()
    node = rclpy.create_node('voicebiz_launch_probe')
    try:
        for mode in ('transport', 'takethings'):
            namespace = '/voicebiz_launch_' + mode
            path = Path(str(ARTIFACT_DIR) + '/voicebiz_launch_' + mode + '.log')
            log = path.open('w')
            child = subprocess.Popen(['ros2', 'launch', 'yahboomcar_voice_ctrl', 'voice.launch.py',
                                      'mode:=' + mode, 'namespace:=' + namespace,
                                      'start_base:=true', 'use_mock:=true', 'start_navigation:=false',
                                      'start_markers:=false', 'enable_serial:=false'],
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                end = time.monotonic() + 10
                names = []
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    text = path.read_text()
                    names = [name for name, ns in node.get_node_names_and_namespaces() if ns == namespace]
                    ready = 'base started: MOCK' in text and 'ready (mode=' in text
                    if len(names) == 3 and ready:
                        break
                    assert child.poll() is None, text[-3000:]
                assert len(names) == 3 and len(set(names)) == 3, (mode, names)
                assert node.count_publishers(namespace + '/vel_raw') == 0, 'business created a final velocity owner'
                assert node.count_publishers(namespace + '/cmd_vel') == 1, 'stop input to Nav2 relay missing'
                text = path.read_text()
                assert 'Traceback' not in text and 'process has died' not in text, text[-3000:]
                assert 'Speech serial opened' not in text
                print('PASS installed business launch %s: unique namespace, no serial, mock base, no second final command owner' % mode, flush=True)
            finally:
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=6)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGTERM)
                        child.wait(timeout=3)
                log.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
