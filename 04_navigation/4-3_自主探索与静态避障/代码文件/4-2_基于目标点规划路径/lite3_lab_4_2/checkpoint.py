"""Immutable, hashed SLAM checkpoints. No ROS imports: usable in preflight/tests."""
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite_pose(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('start_pose must be [map_x, map_y, yaw]')
    result = [float(v) for v in value]
    if not all(math.isfinite(v) for v in result):
        raise ValueError('start_pose must be finite')
    return result


def load_checkpoint(filename, next_stage=None, configuration=None):
    path = Path(filename).expanduser().resolve(strict=True)
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('schema') != 1 or data.get('complete') is not True:
        raise ValueError('Incomplete SLAM checkpoint')
    if next_stage is not None and data['stage'] != next_stage - 1:
        raise ValueError('5-3 requires a 5-2 checkpoint; 5-4 requires 5-3')
    files = data['files']
    required = {'map.yaml', 'map.pgm', 'map.pcd', 'slam.posegraph', 'slam.data'}
    if set(files) != required:
        raise ValueError('Missing map or serialized SLAM state')
    for name, sha in files.items():
        file = path.parent / name
        if not file.is_file() or file.stat().st_size == 0 or digest(file) != sha:
            raise ValueError('Checkpoint file missing or changed: ' + name)
    if configuration is not None and data['configuration'] != configuration:
        raise ValueError('Sensor installation, geometry or motion configuration differs from parent')
    finite_pose(data['saved_pose_map'])
    return data


def register_audit(directory, stage):
    directory = Path(directory).resolve(strict=True)
    run = directory.parent
    session = json.loads((run/'session.json').read_text(encoding='utf-8'))
    if session.get('stage') != stage or not directory.name.startswith('exploration'):
        raise ValueError('Exploration evidence must belong to the matching mapping session')
    temporary = run/'exploration_current.txt.tmp'
    temporary.write_text(str(directory)+'\n', encoding='utf-8')
    temporary.replace(run/'exploration_current.txt')


def current_audit(run):
    run = Path(run).resolve(strict=True)
    pointer = run/'exploration_current.txt'
    if not pointer.exists():
        return run/'exploration'  # Read evidence from earlier software versions.
    directory = Path(pointer.read_text(encoding='utf-8').strip()).resolve(strict=True)
    if directory.parent != run or not directory.is_dir() or not directory.name.startswith('exploration'):
        raise ValueError('Exploration evidence pointer escapes or differs from this mapping session')
    return directory
