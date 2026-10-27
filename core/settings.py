"""Project-owned paths and validated operator settings."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
AXES = ('x', 'y', 'z', 'rx', 'ry', 'rz')
LIMITS = {'plane_z_mm': (-500, 2000), 'z_safe': (-500, 2500),
          'movl_speed': (.1, 500), 'rx': (-180, 180), 'ry': (-180, 180), 'rz': (-180, 180)}


def owned(relative):
    path = (ROOT / relative).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError('Resource must be inside application root')
    return path


def load_config():
    return json.loads(owned('config/config.json').read_text(encoding='utf-8'))


def validate_settings(values):
    if not isinstance(values, dict) or set(values) - set(LIMITS):
        raise ValueError('Chỉ cho phép Plane Z, Z_safe, speed, RX/RY/RZ')
    for key, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{key}: cần số hữu hạn')
        if not LIMITS[key][0] <= value <= LIMITS[key][1]:
            raise ValueError(f'{key}: ngoài giới hạn {LIMITS[key]}')
    return values


def fingerprint(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
