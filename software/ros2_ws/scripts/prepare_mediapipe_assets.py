#!/usr/bin/env python3
"""Download/check the exact official MediaPipe models and photos used in validation.

SDK wheels are installed separately. No system Python packages are changed.
--check-only performs no downloads or file writes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import tempfile
from urllib.parse import urlparse
from urllib.request import urlopen


def valid_file(path, item):
    if not path.is_file() or path.stat().st_size != item['bytes']:
        return False
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest() == item['sha256']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache-dir', required=True, type=Path)
    parser.add_argument('--mediapipe-site', type=Path,
                        help='Private site-packages containing mediapipe/__init__.py')
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    manifest = json.loads(Path(__file__).with_name('optional_vision_assets.json').read_text())
    cache = args.cache_dir.expanduser().resolve()
    sdk = None
    if args.mediapipe_site:
        site = args.mediapipe_site.expanduser().resolve()
        if not (site / 'mediapipe' / '__init__.py').is_file():
            parser.error('--mediapipe-site does not contain an installed MediaPipe SDK')
        sdk = site / 'mediapipe' / 'modules' / 'objectron'
    if not args.check_only:
        cache.mkdir(parents=True, exist_ok=True)
        if sdk is not None:
            sdk.mkdir(parents=True, exist_ok=True)
    for item in manifest['files']:
        name = item['name']
        url = urlparse(item['url'])
        if (Path(name).name != name or url.scheme != 'https'
                or url.netloc != 'storage.googleapis.com'
                or not url.path.startswith('/mediapipe-assets/')
                or not re.fullmatch('[0-9a-f]{64}', item['sha256'])
                or not 0 < item['bytes'] <= 16 * 1024 * 1024):
            raise ValueError('Invalid asset manifest entry: ' + name)
        path = cache / name
        if not valid_file(path, item):
            if args.check_only:
                raise RuntimeError('Missing or mismatched cached asset: ' + str(path))
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(dir=cache, delete=False) as output:
                    temporary = Path(output.name)
                    with urlopen(item['url'], timeout=30) as response:
                        total = 0
                        while True:
                            block = response.read(1024 * 1024)
                            if not block:
                                break
                            total += len(block)
                            if total > item['bytes']:
                                raise RuntimeError('Asset exceeds recorded length: ' + name)
                            output.write(block)
                if not valid_file(temporary, item):
                    raise RuntimeError('Downloaded asset failed SHA256/size: ' + name)
                temporary.replace(path)
            finally:
                if temporary is not None and temporary.exists():
                    temporary.unlink()
        if sdk is not None and name.endswith('.tflite'):
            target = sdk / name
            if args.check_only:
                if not valid_file(target, item):
                    raise RuntimeError('Missing/mismatched SDK model: ' + str(target))
            elif not valid_file(target, item):
                shutil.copyfile(path, target)
                if not valid_file(target, item):
                    raise RuntimeError('Installed SDK model failed verification: ' + name)
        print('PASS SHA256/size:', name, flush=True)
    print('Assets ready. Positive image testdata directory:', cache)


if __name__ == '__main__':
    main()
