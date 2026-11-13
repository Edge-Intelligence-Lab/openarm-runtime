"""External launch configuration for the local platform; no hardware imports."""
from dataclasses import dataclass, field, replace
import ipaddress
import json
from pathlib import Path
import sys

DEFAULTS = (
    ('control', '电机控制', 8893, '双臂状态、关节控制与电机标定'),
    ('teleop', '遥操', 8890, '主臂标定、姿态预览与跟随控制'),
    ('tactile', '触觉', 8891, '触觉分布、设备身份与指垫位置配置'),
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
    lazy: bool = False

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
    host: str = '127.0.0.1'
    allowed_clients: tuple[str, ...] = ()
    simulation_profile: Path | None = None
    teleop_calibration_profile: Path | None = None
    model_catalog: Path | None = None

    @property
    def services(self):
        return self.support + tuple(page for page in self.pages if page.path is None)

    def with_port(self, port):
        return validate(replace(self, port=valid_port(port)))


def validate(config):
    if config.simulation_profile is not None and any(p.command for p in config.services):
        raise ValueError('Simulation platform forbids all external/hardware service commands')
    if config.simulation_profile is not None:
        if config.support or any(p.path != '/simulation/'+p.id or p.port is not None
                for p in config.pages if p.id in ('control','teleop','tactile')):
            raise ValueError('Simulation requires built-in device pages and no auxiliary services')
    if not isinstance(config.host,str):
        raise ValueError('host must be a literal IPv4 string')
    private = tuple(ipaddress.ip_network(n) for n in ('10.0.0.0/8','172.16.0.0/12','192.168.0.0/16'))
    try:
        host = ipaddress.IPv4Address(config.host)
        networks = tuple(ipaddress.ip_network(n,strict=True) for n in config.allowed_clients)
    except (ValueError,TypeError) as exc:
        raise ValueError('host must be a literal IPv4 address; allowed_clients must be canonical CIDRs') from exc
    if str(host) not in ('127.0.0.1','0.0.0.0'):
        if not any(host in net for net in private) or not networks:
            raise ValueError('LAN access requires an explicit RFC1918 bind address and allowed_clients')
    if any(n.version!=4 or n.prefixlen<24 or not any(n.subnet_of(p) for p in private) for n in networks):
        raise ValueError('allowed_clients must be RFC1918 IPv4 networks of /24 or narrower')
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
    if not isinstance(data, dict) or set(data) - {'title', 'port', 'services', 'support', 'log_dir', 'policy_record_root', 'collection_root', 'calibration_root', 'host', 'allowed_clients','simulation_profile','teleop_calibration_profile','head_calibration_port','model_catalog'}:
        raise ValueError('Invalid platform configuration fields')
    base = Path.cwd() if path is None else Path(path).expanduser().resolve().parent
    simulation='simulation_profile' in data
    if simulation and 'teleop_calibration_profile' in data:
        raise ValueError('Physical teleoperation calibration cannot be enabled in simulation mode')
    if simulation and not {'log_dir','calibration_root','collection_root','policy_record_root'}<=set(data):
        raise ValueError('Simulation requires explicit isolated log/record/calibration roots')
    clients=data.get('allowed_clients',[])
    if not isinstance(clients,list) or any(not isinstance(n,str) for n in clients):
        raise ValueError('allowed_clients must be a list of CIDR strings')
    entries = data.get('services', {})
    if not isinstance(entries, dict) or set(entries) - {x[0] for x in DEFAULTS}:
        raise ValueError('services must contain only control, teleop, tactile and vision')

    def path_value(value):
        if not isinstance(value, str) or not value:
            raise ValueError('Paths must be nonempty strings')
        result = Path(value).expanduser()
        return result if result.is_absolute() else base / result

    def page_from(entry, identity, label, port, description='', group=None):
        if (not isinstance(entry, dict) or set(entry) - {'port', 'command', 'cwd', 'extra_ports', 'lazy'}
                or type(entry.get('lazy', False)) is not bool):
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
                    tuple(valid_port(x) for x in extra), lazy=entry.get('lazy', False))

    pages = []
    for identity, label, default_port, description in DEFAULTS:
        entry = entries.get(identity, {})
        port = valid_port(entry.get('port', default_port)) if isinstance(entry, dict) else default_port
        pages.append(page_from(entry, identity, label, port, description,
                                'motion' if identity in ('control', 'teleop') else None))
    if simulation:
        if any(p.command for p in pages) or data.get('support'):
            raise ValueError('Simulation forbids hardware or auxiliary launch commands')
        pages=[replace(p,port=None,path='/simulation/'+p.id,extra_ports=(),exclusive_group=None)
               if p.id!='vision' else p for p in pages]
        descriptions=dict(control='MuJoCo 场景、关节反馈与标定入口',
            teleop='仿真遥操尚未接入 · 不连接主臂',
            tactile='仿真触觉尚未建模 · 不提供伪造读数',
            vision='MuJoCo 头部与场景渲染相机')
        pages=[replace(p,description=descriptions[p.id]) for p in pages]
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
    if 'teleop_calibration_profile' in data:
        teleop=next(p for p in pages if p.id=='teleop')
        if not teleop.command or '--follow' not in teleop.command:
            raise ValueError('Teleop calibration requires an explicitly configured follower service')
        if any(x.startswith('--head-calibration') or x.startswith('--head-video-port') for x in teleop.command):
            raise ValueError('Configure head calibration through the platform, not duplicate CLI options')
        vision=next(p for p in pages if p.id=='vision')
        args=('--head-calibration-profile',str(path_value(data['teleop_calibration_profile'])),
              '--head-calibration-root',str(path_value(data.get('calibration_root','~/.local/share/openarm-calibration'))/'teleop-head'),
              '--head-video-port',str(vision.port))
        # Separate task/service, not an added panel of the collection operator.
        # Both owners still use the existing motion-group handover and bus lease.
        command=[];skip=False
        for argument in teleop.command:
            if skip:skip=False;continue
            if argument=='--collection-profile':skip=True;continue
            if argument.startswith('--collection-profile='):continue
            command.append(argument)
        port=valid_port(data.get('head_calibration_port',8894))
        if '--port' in command:
            index=command.index('--port')
            if index+1>=len(command):raise ValueError('Teleop service has an incomplete --port option')
            command[index+1]=str(port)
        else:command.extend(('--port',str(port)))
        auxiliary.append(Page('head_alignment','头部相机标定控制',port,
            command=tuple(command)+args,cwd=teleop.cwd,exclusive_group='motion',lazy=True))
    if not isinstance(title, str) or not title.strip():
        raise ValueError('title must be a nonempty string')
    pages.append(Page('collection', '数采可视化', None, '录制状态、数据流质量与多模态回放', path='/collection'))
    pages.append(Page('policy', '模型与执行', None, '选择模型、加载测试、手动执行与记录回放', path='/policy'))
    pages.append(Page('camera_calibration', '相机标定', None,
                      '头部相机标定、左右腕部内参与标定板检查', path='/camera-calibration'))
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
                                   path_value(data.get('calibration_root', '~/.local/share/openarm-calibration')),
                                    data.get('host','127.0.0.1'),tuple(clients),
                                      path_value(data['simulation_profile']) if simulation else None,
                                       path_value(data['teleop_calibration_profile']) if 'teleop_calibration_profile' in data else None,
                                       path_value(data['model_catalog']) if 'model_catalog' in data else None))
