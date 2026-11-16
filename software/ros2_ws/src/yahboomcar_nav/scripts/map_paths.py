"""Map destination sanitising shared by the SaveMap clients.

The ROS 1 save-map helper concatenated a parameter directly into a shell
command.  ROS 2 never invokes a shell: callers pass a sanitised absolute YAML
path to nav2_msgs/srv/SaveMap.
"""

import os
import re

_NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')


def sanitize_map_destination(map_name, map_directory):
    """Return an absolute, contained ``*.yaml`` destination path.

    ``map_name`` must be a bare filename (no directories, no ``..``).  The
    result is guaranteed to live directly inside ``map_directory``.
    """
    if map_name is None:
        raise ValueError('map_name is required')

    raw_name = str(map_name).strip()
    if not raw_name:
        raise ValueError('map_name must not be empty')
    if raw_name != os.path.basename(raw_name):
        raise ValueError('map_name must not contain a directory component')
    if raw_name in ('.', '..'):
        raise ValueError('map_name must not be . or ..')
    if not _NAME_RE.match(raw_name):
        raise ValueError(
            'map_name may contain only letters, digits, dot, dash and '
            'underscore, and must start with an alphanumeric character')

    if not (raw_name.lower().endswith('.yaml') or
            raw_name.lower().endswith('.yml')):
        raw_name += '.yaml'

    if not map_directory:
        raise ValueError('map_directory is required')

    directory = os.path.abspath(os.path.expanduser(str(map_directory)))
    target = os.path.abspath(os.path.join(directory, raw_name))

    # Defensive containment check (basename validation above should already
    # make this impossible).
    try:
        inside = os.path.commonpath([directory, target]) == directory
    except ValueError:
        inside = False
    if not inside or target == directory:
        raise ValueError('resolved map path escapes map_directory')

    return target


def ensure_parent_directory(path):
    """Create the parent directory of ``path`` without invoking a shell."""
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    return parent
