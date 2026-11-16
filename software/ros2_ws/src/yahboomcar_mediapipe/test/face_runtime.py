#!/usr/bin/env python3
"""Actual Haar/dlib inference and DDS checks; requires a positive face photo.

Run after building and sourcing the workspace:
  python3 src/yahboomcar_mediapipe/test/face_runtime.py --image face.jpg
Optional --sdk-site (or X3PLUS_DLIB_SITE) selects an isolated dlib install.
This fixture uses real SDKs and the original installed predictor, no camera.
"""
import argparse
import os
from pathlib import Path
import sys
import time


def wait_for(executor, predicate, seconds=15.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        executor.spin_once(timeout_sec=0.05)
    raise AssertionError('Timed out waiting for ROS graph/image output')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True, type=Path)
    parser.add_argument('--image-scale', type=float, default=1.0,
                        help='Resize an actual photo for small faces, e.g. 2.0 (default 1)')
    parser.add_argument('--sdk-site', default=os.environ.get('X3PLUS_DLIB_SITE', ''))
    args = parser.parse_args()
    if args.sdk_site:
        site = Path(args.sdk_site).expanduser().resolve(strict=True)
        if not site.is_dir():
            parser.error('--sdk-site must name a directory')
        sys.path.insert(0, str(site))

    import cv2 as cv
    import dlib
    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from yahboomcar_mediapipe.detectors import FaceEyeDetectionNode, FaceLandmarksNode

    image = cv.imread(str(args.image))
    if image is None:
        parser.error('Cannot decode positive face photo: ' + str(args.image))
    if not 0.1 <= args.image_scale <= 4.0:
        parser.error('--image-scale must be between 0.1 and 4')
    if args.image_scale != 1.0:
        image = cv.resize(image, None, fx=args.image_scale, fy=args.image_scale)
    blank = np.zeros_like(image)
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    bridge = CvBridge()
    print('Real SDK: dlib', dlib.__version__, dlib.__file__, 'OpenCV', cv.__version__)
    rclpy.init()
    try:
        for index, cls in enumerate((FaceEyeDetectionNode, FaceLandmarksNode)):
            detector = None
            probe = None
            executor = SingleThreadedExecutor()
            try:
                detector = cls()
                assert detector.get_parameter('gui').value is False
                if cls is FaceEyeDetectionNode:
                    assert not detector.faceDetect.empty() and not detector.eyeDetect.empty()
                    faces = detector.faceDetect.detectMultiScale(gray, 1.3)
                    assert len(faces) >= 1, 'Positive photo must contain an actual Haar-detected face'
                    assert len(detector.faceDetect.detectMultiScale(
                        cv.cvtColor(blank, cv.COLOR_BGR2GRAY), 1.3)) == 0
                    print('PASS 1: installed Haar models, positive/blank inference, faces=', len(faces))
                else:
                    detector.get_face(image.copy(), draw=False)
                    assert len(detector.faces) >= 1, 'Positive photo must contain an actual HOG face'
                    assert detector.face_landmarks.num_parts == 68
                    eye = detector.get_lmList(image.copy(), 36, 42, draw=False)
                    lip = detector.get_lipList(image.copy(), [48, 49, 50, 51, 62], draw=False)
                    assert len(eye) == 6 and len(lip) == 5
                    assert all(0 <= x < image.shape[1] and 0 <= y < image.shape[0]
                               for x, y in eye + lip)
                    detector.get_face(blank.copy(), draw=False)
                    assert len(detector.faces) == 0 and detector.face_landmarks is None
                    assert detector.get_lmList(blank.copy(), 36, 42, draw=False) == []
                    print('PASS 3: actual HOG/68-point predictor, eye/lip coordinates, stale-state clearing')

                probe = Node('face_runtime_probe_' + str(index))
                pub = probe.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
                received = []
                sub = probe.create_subscription(
                    Image, detector.get_parameter('image_output_topic').value,
                    received.append, qos_profile_sensor_data)
                executor.add_node(detector)
                executor.add_node(probe)
                wait_for(executor, lambda: pub.get_subscription_count() >= 1
                         and probe.count_publishers(sub.topic_name) >= 1)

                for sequence, frame in enumerate((image, blank), 1):
                    request = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    request.header.frame_id = 'face_test_optical'
                    request.header.stamp.sec = 1700000000 + index * 10 + sequence
                    request.header.stamp.nanosec = 123456789
                    received.clear()
                    deadline = time.monotonic() + 20.0
                    next_publish = 0.0
                    response = None
                    while time.monotonic() < deadline:
                        if time.monotonic() >= next_publish:
                            pub.publish(request)
                            next_publish = time.monotonic() + 0.4
                        executor.spin_once(timeout_sec=0.05)
                        matching = [m for m in received if m.header.stamp == request.header.stamp]
                        if matching:
                            response = matching[-1]
                            break
                    assert response is not None, 'No matching DDS image from ' + detector.get_name()
                    assert response.encoding == 'bgr8'
                    assert response.header == request.header
                    assert bridge.imgmsg_to_cv2(response, 'bgr8').shape == frame.shape
                    if cls is FaceLandmarksNode:
                        if sequence == 1:
                            assert len(detector.faces) >= 1 and detector.face_landmarks.num_parts == 68
                        else:
                            assert len(detector.faces) == 0 and detector.face_landmarks is None
                print('PASS %d: %s real DDS positive/blank images, dimensions and exact header'
                      % (2 if index == 0 else 4, detector.get_name()))
            finally:
                for node in (probe, detector):
                    if node is not None:
                        executor.remove_node(node)
                        node.destroy_node()
                executor.shutdown()
    finally:
        rclpy.try_shutdown()
    print('RESULT: PASS - all 4 actual face runtime groups')


if __name__ == '__main__':
    main()
