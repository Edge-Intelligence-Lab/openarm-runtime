"""External launch configuration for the local platform; no hardware imports."""
from dataclasses import dataclass, field, replace
import json
from pathlib import Path
import sys

DEFAULTS = (
    ('control', '电机控制', 8893, '双臂状态、关节控制与电机标定'),
    ('teleop', '遥操', 8890, '主臂标定、姿态预览与跟随控制'),
    ('tactile', '触觉', 8891, '左臂双传感器 · 触觉分布与力反馈'),
    ('vision', '视觉', 5003, '头部、腕部与深度画面'),
)


def valid_port(value):
    if type(value) is not int or not 1024 <= value <= 65535:
        raise ValueError('Ports must be integers in 1024..65535')
    return value


@dataclass(frozen=True)
class Page:
    id: str
    label: str
    port: int | None
    description: str = ''
    command: tuple[str, ...] = ()
    cwd: Path | None = None
    exclusive_group: str | None = None
    extra_ports: tuple[int, ...] = ()

    path: str | None = None

    def public(self):
        return dict(id=self.id, label=self.label, port=self.port,
                    description=self.description, exclusive_group=self.exclusive_group,
                    extra_ports=list(self.extra_ports), path=self.path)


@dataclass(frozen=True)
class PlatformConfig:
    port: int
    pages: tuple[Page, ...]
    log_dir: Path
    support: tuple[Page, ...] = ()
    title: str = 'OpenArm Platform'
    policy_record_root: Path = field(default_factory=lambda: Path.home() / 'openarm-episodes')
    collection_root: Path = field(default_factory=lambda: Path.home() / 'openarm-collection')
    calibration_root: Path = field(default_factory=lambda: Path.home() / '.local/share/openarm-calibration')

    @property
    def services(self):
        return self.support + tuple(page for page in self.pages if page.path is None)

    def with_port(self, port):
        return validate(replace(self, port=valid_port(port)))


def validate(config):
    ports = [config.port]
    for page in config.services:
        ports.extend((page.port, *page.extra_ports))
    for port in ports:
        valid_port(port)
    if len(set(ports)) != len(ports):
        raise ValueError('Platform, service and internal view ports must be distinct')
    return config


def load_config(path=None):
    data = {} if path is None else json.loads(Path(path).expanduser().read_text())
    if not isinstance(data, dict) or set(data) - {'title', 'port', 'services', 'support', 'log_dir', 'policy_record_root', 'collection_root', 'calibration_root'}:
        raise ValueError('Invalid platform configuration fields')
    base = Path.cwd() if path is None else Path(path).expanduser().resolve().parent
    entries = data.get('services', {})
    if not isinstance(entries, dict) or set(entries) - {x[0] for x in DEFAULTS}:
        raise ValueError('services must contain only control, teleop, tactile and vision')

    def path_value(value):
        if not isinstance(value, str) or not value:
            raise ValueError('Paths must be nonempty strings')
        result = Path(value).expanduser()
        return result if result.is_absolute() else base / result

    def page_from(entry, identity, label, port, description='', group=None):
        if not isinstance(entry, dict) or set(entry) - {'port', 'command', 'cwd', 'extra_ports'}:
            raise ValueError(f'Invalid configuration for {identity}')
        command = entry.get('command', [])
        if not isinstance(command, list) or any(not isinstance(x, str) or not x or '\0' in x for x in command):
            raise ValueError(f'{identity}: command must be a list of nonempty strings, not a shell command')
        command = tuple(sys.executable if x == '{python}' else str(Path(x).expanduser()) if x.startswith('~/') else x for x in command)
        extra = entry.get('extra_ports', [port + 10, port + 11] if identity == 'tactile' else [])
        if not isinstance(extra, list):
            raise ValueError('extra_ports must be a list')
        return Page(identity, label, valid_port(entry.get('port', port)), description, command,
                    path_value(entry['cwd']) if 'cwd' in entry else base, group,
                    tuple(valid_port(x) for x in extra))

    pages = []
    for identity, label, default_port, description in DEFAULTS:
        entry = entries.get(identity, {})
        port = valid_port(entry.get('port', default_port)) if isinstance(entry, dict) else default_port
        pages.append(page_from(entry, identity, label, port, description,
                               'motion' if identity in ('control', 'teleop') else None))
    support = data.get('support', {})
    if not isinstance(support, dict):
        raise ValueError('support must be an object')
    auxiliary = []
    for identity, entry in support.items():
        if not isinstance(identity, str) or not identity.startswith('aux_') or not identity.replace('_', '').isalnum():
            raise ValueError('Support service IDs must start with aux_ and contain only letters, digits or underscores')
        if not isinstance(entry, dict) or 'port' not in entry:
            raise ValueError('Support services need a port')
        auxiliary.append(page_from(entry, identity, identity, entry['port']))
    title = data.get('title', 'OpenArm Platform')
    if not isinstance(title, str) or not title.strip():
        raise ValueError('title must be a nonempty string')
    pages.append(Page('collection', '数采可视化', None, '录制状态、数据流质量与多模态回放', path='/collection'))
    pages.append(Page('policy', '策略可视化', None, '策略仿真、目标与反馈轨迹、记录回放', path='/policy'))
    pages.append(Page('camera_calibration', '头部相机标定', None,
                      '原尺寸借帧、ChArUco 内参与留出检查、离线头部外参', path='/camera-calibration'))
    collection_root = path_value(data.get('collection_root', '~/openarm-collection'))
    if 'collection_root' not in data:
        teleop = next(p for p in pages if p.id == 'teleop')
        if '--collection-profile' in teleop.command:
            index = teleop.command.index('--collection-profile') + 1
            if index < len(teleop.command):
                profile = Path(teleop.command[index]).expanduser()
                if not profile.is_absolute():
                    profile = (teleop.cwd or base) / profile
                if profile.is_file():
                    acquisition = json.loads(profile.read_text())
                    collection_root = (profile.parent / acquisition['root']).resolve()
    return validate(PlatformConfig(valid_port(data.get('port', 8888)), tuple(pages),
                                   path_value(data.get('log_dir', '~/.local/state/openarm-platform')),
                                   tuple(auxiliary), title, path_value(data.get('policy_record_root', '~/openarm-episodes')), collection_root,
                                   path_value(data.get('calibration_root', '~/.local/share/openarm-calibration'))))
