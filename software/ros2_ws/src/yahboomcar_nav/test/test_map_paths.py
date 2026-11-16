import os
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from map_paths import sanitize_map_destination, ensure_parent_directory


@pytest.mark.parametrize(
    'input_name,expected_filename',
    [
        ('my_map', 'my_map.yaml'),
        ('warehouse_2026', 'warehouse_2026.yaml'),
        ('map.test', 'map.test.yaml'),
        ('map..name', 'map..name.yaml'),
        ('Map-One', 'Map-One.yaml'),
        ('campus.YAML', 'campus.YAML'),
        ('floor.yml', 'floor.yml'),
        ('Zone.Yml', 'Zone.Yml'),
        ('office.yaml', 'office.yaml'),
        ('  padded_map  ', 'padded_map.yaml'),
    ],
)
def test_sanitize_valid_names_and_extension_handling(tmp_path, input_name, expected_filename):
    res = sanitize_map_destination(input_name, str(tmp_path))
    assert res == os.path.abspath(os.path.join(str(tmp_path), expected_filename))
    assert os.path.commonpath([str(tmp_path), res]) == str(tmp_path)


@pytest.mark.parametrize(
    'invalid_name',
    [
        None,
        '',
        '   ',
        '.',
        '..',
        '-leading_dash',
        '_leading_underscore',
        '.leading_dot',
        'map;rm -rf',
        'map$VAR',
        'map`whoami`',
        'map&bg',
        'sub/map',
        r'sub\map',
        '../escaped',
        r'..\escaped',
        '/absolute/path/map',
        'C:\\absolute\\path',
    ],
)
def test_sanitize_invalid_map_name(tmp_path, invalid_name):
    with pytest.raises(ValueError):
        sanitize_map_destination(invalid_name, str(tmp_path))


@pytest.mark.parametrize('invalid_dir', [None, '', False])
def test_sanitize_invalid_map_directory(invalid_dir):
    with pytest.raises(ValueError, match='map_directory is required'):
        sanitize_map_destination('valid_map', invalid_dir)


def test_sanitize_containment_and_expansion(tmp_path):
    dest = sanitize_map_destination('nav_map', str(tmp_path))
    assert os.path.isabs(dest)
    assert os.path.dirname(dest) == os.path.abspath(str(tmp_path))
    assert os.path.basename(dest) == 'nav_map.yaml'


def test_ensure_parent_directory_nested_creation(tmp_path):
    nested_file = tmp_path / 'level1' / 'level2' / 'maps' / 'target.yaml'
    assert not nested_file.parent.exists()
    parent_returned = ensure_parent_directory(str(nested_file))
    assert parent_returned == str(nested_file.parent)
    assert os.path.isdir(parent_returned)


def test_ensure_parent_directory_preserves_existing_file(tmp_path):
    parent_dir = tmp_path / 'existing_dir'
    parent_dir.mkdir(parents=True)
    existing_file = parent_dir / 'map.yaml'
    existing_file.write_text('content_data: 123')

    parent_returned = ensure_parent_directory(str(existing_file))
    assert parent_returned == str(parent_dir)
    assert existing_file.exists()
    assert existing_file.read_text() == 'content_data: 123'
