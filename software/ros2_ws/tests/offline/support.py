"""Paths shared by portable offline checks. No files outside the chosen artifacts."""
import os
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
ARTIFACT_DIR = Path(os.environ.get('X3PLUS_TEST_ARTIFACTS',
                                 WORKSPACE_ROOT / 'test_results' / 'offline')).expanduser().resolve()
ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
