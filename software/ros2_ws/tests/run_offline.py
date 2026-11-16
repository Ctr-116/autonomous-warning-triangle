#!/usr/bin/env python3
"""Portable standalone offline integration test suite runner for ROS 2 Humble."""

import argparse
import os
from pathlib import Path
import subprocess
import sys


def build_pythonpath(root: Path, extra_prefix: str = "") -> str:
    paths = []
    if extra_prefix:
        paths.append(extra_prefix)
    # Offline integration validates the sourced installation. Adding every
    # source directory can shadow it and makes Python imports very slow on
    # Windows-mounted workspaces. Unit source tests use their separate runner.
    vendor_py = root / "vendor" / "py_install"
    if vendor_py.is_dir():
        paths.append(str(vendor_py))
    tests_dir = root / "tests"
    if tests_dir.is_dir():
        paths.append(str(tests_dir))
    existing = os.environ.get("PYTHONPATH", "")
    if existing:
        paths.append(existing)
    return os.pathsep.join(paths)


def get_core_suites():
    return [
        ("bringup", [["tests/offline/check_bringup.py"]]),
        (
            "model",
            [
                ["tests/offline/check_interfaces.py"],
                ["tests/offline/check_base_accessories.py"],
                ["tests/offline/check_model_laser.py"],
            ],
        ),
        (
            "voice",
            [
                ["tests/offline/check_voice.py"],
                ["tests/offline/check_voice_vision.py"],
                ["tests/offline/check_voice_launch.py"],
                ["tests/offline/voicebiz_launch_check.py"],
                ["src/yahboomcar_voice_ctrl/test/voicebiz_runtime.py"],
                ["src/yahboomcar_voice_ctrl/test/waypoints_runtime.py"],
            ],
        ),
        (
            "moveit",
            [
                ["tests/offline/check_pose_requests.py"],
                ["tests/offline/check_moveit.py"],
            ],
        ),
        (
            "nav2",
            [
                ["src/yahboomcar_nav/scripts/nav2_offline_check.py"],
                ["src/yahboomcar_nav/scripts/nav2_offline_check.py", "--amcl"],
                ["src/yahboomcar_nav/scripts/nav2_offline_check.py", "--namespace", "test_robot"],
                ["src/yahboomcar_nav/scripts/nav2_offline_check.py", "--amcl", "--namespace", "test_robot"],
                ["src/yahboomcar_nav/scripts/nav2_map_save_check.py"],
            ],
        ),
        (
            "perception",
            [
                ["src/yahboomcar_visual/scripts/perception_native_check.py"],
                ["src/yahboomcar_visual/scripts/perception_dnn_check.py"],
                ["src/yahboomcar_slam/scripts/perception_mapper_check.py"],
            ],
        ),
    ]


def build_optional_commands(suite_name: str):
    if suite_name == "mediapipe":
        site = os.environ.get("X3PLUS_MEDIAPIPE_SITE")
        data = os.environ.get("X3PLUS_MEDIAPIPE_TESTDATA")
        if not site or not Path(site).is_dir():
            print(f"Error: X3PLUS_MEDIAPIPE_SITE='{site}' invalid/missing", file=sys.stderr, flush=True)
            sys.exit(1)
        if not data or not Path(data).is_dir() or not (Path(data) / "hands.jpg").is_file() or not (Path(data) / "pose.jpg").is_file():
            print(f"Error: X3PLUS_MEDIAPIPE_TESTDATA='{data}' missing hands.jpg/pose.jpg", file=sys.stderr, flush=True)
            sys.exit(1)
        return [(["src/yahboomcar_mediapipe/scripts/mediapipe_offline_check.py", "--testdata", data], site)]
    if suite_name == "face":
        site = os.environ.get("X3PLUS_DLIB_SITE")
        photo = os.environ.get("X3PLUS_FACE_IMAGE")
        if not site or not Path(site).is_dir():
            print(f"Error: X3PLUS_DLIB_SITE='{site}' invalid/missing", file=sys.stderr, flush=True)
            sys.exit(1)
        if not photo or not Path(photo).is_file():
            print(f"Error: X3PLUS_FACE_IMAGE='{photo}' invalid/missing", file=sys.stderr, flush=True)
            sys.exit(1)
        scale = os.environ.get("X3PLUS_FACE_IMAGE_SCALE", "1")
        try:
            float(scale)
        except ValueError:
            print(f"Error: X3PLUS_FACE_IMAGE_SCALE='{scale}' not numeric", file=sys.stderr, flush=True)
            sys.exit(1)
        return [(["src/yahboomcar_mediapipe/test/face_runtime.py", "--sdk-site", site, "--image", photo, "--image-scale", scale], "")]
    if suite_name == "yolo":
        site = os.environ.get("X3PLUS_ONNX_SITE")
        if not site or not Path(site).is_dir():
            print(f"Error: X3PLUS_ONNX_SITE='{site}' invalid/missing", file=sys.stderr, flush=True)
            sys.exit(1)
        return [(["src/garbage_identify_yolov11/scripts/perception_yolo_check.py"], site)]
    return []


def main():
    parser = argparse.ArgumentParser(description="Standalone offline integration suite runner.")
    parser.add_argument(
        "--suite",
        choices=["core", "bringup", "model", "voice", "moveit", "nav2", "perception", "slam", "mediapipe", "face", "yolo"],
        default="core",
        help="Test suite to run (default: core)",
    )
    args = parser.parse_args()

    ros_distro = os.environ.get("ROS_DISTRO")
    if ros_distro != "humble":
        print(f"Error: ROS_DISTRO is '{ros_distro}', require sourced ROS 2 'humble'.", file=sys.stderr, flush=True)
        sys.exit(1)

    root = Path(__file__).resolve().parents[1]
    core_suites = get_core_suites()
    all_cmds = []

    if args.suite == "core":
        print("Coverage: core suites (bringup, model, voice, moveit, nav2, perception).", flush=True)
        print("Exclusion: optional SDK checks (mediapipe, face, yolo) and unit tests (tests/run_unit.py).", flush=True)
        for sname, cmds in core_suites:
            for c in cmds:
                all_cmds.append((sname, c, ""))
    elif args.suite == "slam":
        all_cmds.append(("slam", ["src/yahboomcar_nav/scripts/slam_toolbox_offline_check.py"], ""))
    elif args.suite in dict(core_suites):
        for sname, cmds in core_suites:
            if sname == args.suite:
                for c in cmds:
                    all_cmds.append((sname, c, ""))
    else:
        for c, extra_prefix in build_optional_commands(args.suite):
            all_cmds.append((args.suite, c, extra_prefix))

    env = os.environ.copy()
    env["ROS_LOCALHOST_ONLY"] = "1"
    env.setdefault("ROS_DOMAIN_ID", "197")

    print(f"Starting offline suite runner. Root: {root}. Total commands: {len(all_cmds)}", flush=True)
    for idx, (sname, cmd_args, extra_prefix) in enumerate(all_cmds, 1):
        full_cmd = [sys.executable, str(root / cmd_args[0])] + cmd_args[1:]
        cmd_env = env.copy()
        cmd_env["PYTHONPATH"] = build_pythonpath(root, extra_prefix=extra_prefix)
        print(f"[{idx}/{len(all_cmds)}] START ({sname}): {' '.join(full_cmd)}", flush=True)
        try:
            res = subprocess.run(full_cmd, cwd=str(root), env=cmd_env)
            print(f"[{idx}/{len(all_cmds)}] FINISHED ({sname}) Returncode: {res.returncode}", flush=True)
            if res.returncode != 0:
                print(f"FAILED ({sname}) with returncode {res.returncode}. Failfast stopping.", file=sys.stderr, flush=True)
                sys.exit(res.returncode)
        except KeyboardInterrupt:
            print("\nExecution interrupted by user (SIGINT).", file=sys.stderr, flush=True)
            sys.exit(130)

    print("SUCCESS: All integration checks passed with returncode 0.", flush=True)
    sys.exit(0)


if __name__ == "__main__":
    main()
