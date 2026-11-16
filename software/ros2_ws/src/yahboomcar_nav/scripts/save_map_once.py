#!/usr/bin/env python3
"""One-shot nav2_msgs/srv/SaveMap CLI client.

Calls the native nav2 map_saver service asynchronously with a finite
monotonic timeout.  On expiry the client future is cancelled and removed
from the pending list, and the process exits non-zero.  map_url is a STEM
(nav2 map_saver appends .yaml/.pgm itself).

Usage:
    ros2 run yahboomcar_nav save_map_once.py my_map \
        [--service map_saver/save_map] [--map-topic map] \
        [--output-directory ~/.ros/yahboomcar_maps] [--timeout 15]
"""

import argparse
import math
import os
import re
import sys
import time

import rclpy
from rclpy.node import Node

from nav2_msgs.srv import SaveMap

_NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')


def sanitize_map_stem(raw):
    if raw is None:
        raise ValueError('map name is required')
    name = str(raw).strip()
    if not name:
        raise ValueError('map name must not be empty')
    if name != os.path.basename(name):
        raise ValueError('map name must not contain a directory component')
    if '/' in name or '\\' in name:
        raise ValueError('map name must not contain path separators')
    if name in ('.', '..'):
        raise ValueError('map name must not be . or ..')
    lower = name.lower()
    if lower.endswith('.yaml'):
        name = name[:-5]
    elif lower.endswith('.yml'):
        name = name[:-4]
    if not name:
        raise ValueError('map name stem must not be empty')
    if name in ('.', '..'):
        raise ValueError('map name stem must not be . or ..')
    if not _NAME_RE.match(name):
        raise ValueError('map name has illegal characters')
    return name


def resolve_map_stem(map_name, output_directory):
    stem = sanitize_map_stem(map_name)
    if not output_directory:
        raise ValueError('output_directory is required')
    directory = os.path.abspath(os.path.expanduser(str(output_directory)))
    os.makedirs(directory, exist_ok=True)
    target = os.path.abspath(os.path.join(directory, stem))
    try:
        inside = os.path.commonpath([directory, target]) == directory
    except ValueError:
        inside = False
    if not inside or target == directory:
        raise ValueError('resolved map path escapes output_directory')
    return target


class SaveMapOnceClient(Node):
    def __init__(self, service, map_topic, mode, timeout):
        super().__init__('save_map_once')
        self._timeout = float(timeout)
        self._client = self.create_client(SaveMap, service)
        self._service = service
        self._map_topic = map_topic
        self._mode = mode

    def save(self, target_stem):
        if not math.isfinite(self._timeout) or self._timeout <= 0.0:
            self.get_logger().error('timeout must be finite and positive.')
            return None
        if not self._client.wait_for_service(timeout_sec=self._timeout):
            self.get_logger().error(
                'SaveMap service %s not available (timeout %.1f s).' % (
                    self._service, self._timeout))
            return None

        req = SaveMap.Request()
        req.map_url = target_stem
        req.map_topic = self._map_topic
        req.map_mode = self._mode
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65
        req.image_format = 'pgm'

        future = self._client.call_async(req)
        deadline = time.monotonic() + self._timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    return future.result()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error('SaveMap call failed: %s' % exc)
                    return None
        self.get_logger().error(
            'SaveMap call did not complete within %.1f s; cancelling.' %
            self._timeout)
        try:
            future.cancel()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('future.cancel failed: %s' % exc)
        self._client.remove_pending_request(future)
        return None


def _parse(argv):
    parser = argparse.ArgumentParser(
        description='Save the current nav2 map to disk once.')
    parser.add_argument('map_name', help='Bare map stem (no dir, suffix stripped).')
    parser.add_argument('--service', default='map_saver/save_map')
    parser.add_argument('--map-topic', default='map')
    parser.add_argument('--mode', default='trinary',
                        choices=['trinary', 'scale', 'raw'])
    parser.add_argument(
        '--output-directory',
        default=os.path.join(os.path.expanduser('~'), '.ros', 'yahboomcar_maps'))
    parser.add_argument('--timeout', type=float, default=15.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse(argv if argv is not None else sys.argv[1:])
    try:
        target_stem = resolve_map_stem(args.map_name, args.output_directory)
    except ValueError as exc:
        print('save_map_once: %s' % exc, file=sys.stderr)
        return 2

    rclpy.init(args=[])
    node = SaveMapOnceClient(
        args.service, args.map_topic, args.mode, args.timeout)
    try:
        response = node.save(target_stem)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if response is None:
        print('save_map_once: no successful SaveMap response', file=sys.stderr)
        return 1
    if not response.result:
        print('save_map_once: SaveMap returned failure (%s)' % response.result,
              file=sys.stderr)
        return 1
    print('save_map_once: saved %s' % target_stem)
    return 0


if __name__ == '__main__':
    sys.exit(main())
