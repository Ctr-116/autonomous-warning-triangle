#!/usr/bin/env python3
"""Verify the committed source manifest, Python syntax and ROS package layout."""
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / 'FILE_MANIFEST.json').read_text(encoding='utf-8'))
    for entry in manifest['files']:
        path = (root / entry['path']).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise RuntimeError('Missing/invalid manifest path: ' + entry['path'])
        hashed = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                hashed.update(block)
        if path.stat().st_size != entry['bytes'] or hashed.hexdigest() != entry['sha256']:
            raise RuntimeError('Source bytes differ from manifest: ' + entry['path'])
    python_paths = list((root / 'src').rglob('*.py'))
    python_paths += list((root / 'tests').rglob('*.py'))
    python_paths += list((root / 'scripts').rglob('*.py'))
    for path in python_paths:
        ast.parse(path.read_bytes(), filename=str(path.relative_to(root)))
    packages = [ET.parse(path).getroot().findtext('name')
                for path in (root / 'src').glob('*/package.xml')]
    if len(packages) != len(set(packages)) or len(packages) != 22:
        raise RuntimeError('Expected 22 distinct ROS package names')
    if (root.parent / 'motion_control_pkg').exists():
        raise RuntimeError('Legacy duplicate motion_control_pkg remains outside the workspace')
    if (root / 'src/yahboomcar_slam/param/ORBvoc.txt').exists():
        raise RuntimeError('Unused legacy ORB vocabulary must remain excluded')
    print('PASS: %d source hashes, %d Python files, 22 distinct ROS packages'
          % (len(manifest['files']), len(python_paths)))


if __name__ == '__main__':
    main()
