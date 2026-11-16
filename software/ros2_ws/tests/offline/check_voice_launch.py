"""Launch the installed ROS2 entrypoint for every implemented voice mode."""

from support import ARTIFACT_DIR, WORKSPACE_ROOT
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy


def main():
    rclpy.init()
    node = rclpy.create_node('voice_launch_validation')
    try:
        for mode in ('base', 'arm', 'study', 'line', 'color_tracker', 'color_identify'):
            namespace = '/launch_voice_' + mode
            path = Path(str(ARTIFACT_DIR) + '/voice_launch_' + mode + '.log')
            log = path.open('w')
            process = subprocess.Popen([
                'ros2', 'launch', 'yahboomcar_voice_ctrl', 'voice.launch.py',
                'mode:=' + mode, 'namespace:=' + namespace,
                'start_base:=true', 'use_mock:=true', 'enable_serial:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                expected_nodes = 4 if mode == 'color_tracker' else (2 if mode == 'color_identify' else 3)
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    names = [name for name, ns in node.get_node_names_and_namespaces() if ns == namespace]
                    ready = mode == 'color_identify' or 'base started: MOCK' in path.read_text()
                    owner_ready = node.count_publishers(namespace + '/vel_raw') == int(mode in ('base', 'line', 'color_tracker'))
                    if len(names) >= expected_nodes and ready and owner_ready:
                        break
                    assert process.poll() is None, path.read_text()[-3000:]
                assert len(names) == expected_nodes and len(set(names)) == expected_nodes, (mode, names, path.read_text()[-3000:])
                active_owner = mode in ('base', 'line', 'color_tracker')
                assert node.count_publishers(namespace + '/vel_raw') == int(active_owner), (mode, 'velocity owner count')
                assert process.poll() is None
                text = path.read_text()
                assert 'Traceback' not in text and 'process has died' not in text, text[-3000:]
                assert 'Speech serial opened' not in text, 'default launch opened serial'
                if mode != 'color_identify':
                    assert 'base started: MOCK' in text, text[-3000:]
                print('PASS installed voice launch %s: unique nodes, namespaced interfaces, expected velocity owner, default hardware disabled' % mode, flush=True)
            finally:
                if process.poll() is None:
                    # ros2 launch forwards SIGINT to its children itself.
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=3)
                log.close()
                for _ in range(3):
                    rclpy.spin_once(node, timeout_sec=0.03)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
