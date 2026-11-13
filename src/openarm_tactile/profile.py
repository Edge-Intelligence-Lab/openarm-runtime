"""External serial identity and operator-assigned pad locations."""
import json
import os
from pathlib import Path
import tempfile

POSITIONS = {'unassigned': '未配置', 'left_pad_1': '左手 · 左爪',
             'left_pad_2': '左手 · 右爪', 'right_pad_1': '右手 · 左爪',
             'right_pad_2': '右手 · 右爪'}


def load(path):
    data = json.loads(Path(path).read_text())
    sensors = data['tactile']
    if not isinstance(sensors, dict) or not sensors:
        raise ValueError('tactile must contain sensors')
    serials, devices, positions = set(), set(), set()
    for options in sensors.values():
        identity = options['identity']
        serial = identity['serial']
        device = options['device']
        position = identity.get('position', 'unassigned')
        if not isinstance(serial, str) or not serial or serial in serials:
            raise ValueError('Sensor serials must be unique nonempty strings')
        if not isinstance(device, str) or not device or str(Path(device).resolve()) in devices:
            raise ValueError('Sensor devices must be distinct')
        if position not in POSITIONS or (position != 'unassigned' and position in positions):
            raise ValueError('Pad positions must be valid and unique')
        serials.add(serial)
        devices.add(str(Path(device).resolve()))
        positions.add(position)
    return data


def save_positions(path, mapping):
    """Only position metadata can be changed by the web request."""
    path = Path(path)
    data = load(path)
    sensors = data['tactile']
    if not isinstance(mapping, dict) or set(mapping) != {o['identity']['serial'] for o in sensors.values()}:
        raise ValueError('Provide one position per configured serial')
    values = list(mapping.values())
    if any(not isinstance(v, str) or v not in POSITIONS for v in values):
        raise ValueError('Unknown pad position')
    assigned = [v for v in values if v != 'unassigned']
    if len(assigned) != len(set(assigned)):
        raise ValueError('同一个指垫位置不能分配给两个传感器')
    for options in sensors.values():
        identity = options['identity']
        position = mapping[identity['serial']]
        identity.update(position=position, mount=position if position != 'unassigned'
                        else 'physical_pad_assignment_unverified')
    # Same-directory atomic replacement; concurrent readers see a complete version.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as file:
            temporary = Path(file.name)
            json.dump(data, file, ensure_ascii=False, indent=2, allow_nan=False)
            file.write('\n')
            file.flush()
            os.fsync(file.fileno())
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return data
