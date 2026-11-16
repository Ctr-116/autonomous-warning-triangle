#!/usr/bin/env python3
"""Run unit tests for the ROS 2 workspace in isolated subprocesses."""

from pathlib import Path
import argparse
import os
import subprocess
import sys


def check_pytest_available() -> None:
    """Verify that pytest is installed and importable."""
    try:
        import pytest  # noqa: F401
    except ImportError:
        print(
            "Error: 'pytest' is not installed or not available in the current environment.",
            file=sys.stderr,
            flush=True,
        )
        print("Please install pytest (e.g. via pip or rosdep) to run unit tests.", file=sys.stderr, flush=True)
        sys.exit(1)


def build_pythonpath(root: Path) -> str:
    """Construct PYTHONPATH containing src/* package roots, vendor/py_install, and tests."""
    paths = []
    src_dir = root / "src"
    if src_dir.is_dir():
        for pkg in sorted(src_dir.iterdir()):
            if pkg.is_dir():
                paths.append(str(pkg))

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


def has_test_files(test_dir: Path) -> bool:
    """Check if test_dir contains at least one test_*.py file, excluding __pycache__."""
    for p in test_dir.rglob("test_*.py"):
        if "__pycache__" not in p.parts and p.is_file():
            return True
    return False


def collect_targets(root: Path):
    """Collect test targets: package-level test directories and top-level tests/test_*.py."""
    groups = []

    # 1. Package test directories under src/*/test (only if containing test_*.py)
    src_dir = root / "src"
    if src_dir.is_dir():
        for test_path in sorted(src_dir.glob("*/test")):
            if test_path.is_dir() and has_test_files(test_path):
                pkg_name = test_path.parent.name
                groups.append((f"Package [{pkg_name}]", [str(test_path)]))

    # 2. Existing tests/test_*.py files
    tests_dir = root / "tests"
    if tests_dir.is_dir():
        test_files = [
            str(f)
            for f in sorted(tests_dir.glob("test_*.py"))
            if f.is_file() and "__pycache__" not in f.parts
        ]
        if test_files:
            groups.append(("Top-level tests", test_files))

    return groups


def main() -> None:
    parser = argparse.ArgumentParser(description="Run unit tests in isolated subprocesses.")
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose test output (passes -v to pytest)",
    )
    args = parser.parse_args()

    check_pytest_available()

    root = Path(__file__).resolve().parents[1]

    targets = collect_targets(root)
    if not targets:
        print(f"Error: No test targets found under {root / 'src'} or {root / 'tests'}.", file=sys.stderr, flush=True)
        sys.exit(1)

    pythonpath = build_pythonpath(root)
    env = os.environ.copy()
    env["PYTHONPATH"] = pythonpath
    env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"

    base_pytest_cmd = [
        sys.executable,
        "-m",
        "pytest",
        "-v" if args.verbose else "-q",
        "--import-mode=importlib",
        "-p",
        "no:cacheprovider",
    ]

    failed_suites = []

    print(f"Starting unit test runner. Root: {root}", flush=True)
    print(f"Discovered {len(targets)} test suite(s).\n", flush=True)

    for label, paths in targets:
        print(f"=== Running: {label} ===", flush=True)
        cmd = base_pytest_cmd + paths
        try:
            res = subprocess.run(cmd, cwd=str(root), env=env)
            if res.returncode == 0:
                print(f"=== Passed: {label} (exit status: 0) ===\n", flush=True)
            else:
                print(f"=== FAILED: {label} (exit status: {res.returncode}) ===\n", file=sys.stderr, flush=True)
                failed_suites.append((label, res.returncode))
        except KeyboardInterrupt:
            print(f"\nExecution interrupted by user while running: {label}", file=sys.stderr, flush=True)
            sys.exit(130)

    print("================== Summary ==================", flush=True)
    if failed_suites:
        print(f"FAIL: {len(failed_suites)} suite(s) failed:", file=sys.stderr, flush=True)
        for label, code in failed_suites:
            print(f"  - {label}: exit code {code}", file=sys.stderr, flush=True)
        sys.exit(1)
    else:
        print(f"SUCCESS: All {len(targets)} suite(s) passed successfully.", flush=True)
        sys.exit(0)


if __name__ == "__main__":
    main()
