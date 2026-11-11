"""Explicit composition of a simulator into the existing platform, not a second UI."""
from pathlib import Path


def create_simulation(config):
    from openarm_simulation.backend import SimulationBackend
    backend=SimulationBackend(config.simulation_profile)
    root=Path(backend.config['output_root']).resolve()
    for path in (config.log_dir,config.calibration_root,config.collection_root,config.policy_record_root):
        if not Path(path).resolve().is_relative_to(root) or Path(path).resolve()==root:
            raise ValueError('All simulation outputs must use explicit children of the simulator output root')
    vision=next(p for p in config.pages if p.id=='vision')
    if vision.port!=backend.video_port or config.port==backend.video_port:
        raise ValueError('Simulation video port must match the platform route and differ from its main port')
    return backend


def build_simulation_view(backend,*,attach_only=False):
    from nicegui import ui
    ui.label('MUJOCO 仿真 · 不连接实物').classes('text-xl font-semibold text-orange-9')
    ui.label('被动仿真场景；无自动抬臂、装板或搜索。名义动力学不代表实物已校准。').classes('text-sm')
    with ui.row().classes('w-full'):
        with ui.column().classes('grow'):
            ui.label('场景相机')
            ui.html('<img src="/apps/vision/stream/overview.mjpg" style="width:100%;max-width:720px" alt="MuJoCo场景"/>',sanitize=False)
        with ui.column().classes('grow'):
            ui.label('头部相机')
            ui.html('<img src="/apps/vision/stream/head.mjpg" style="width:100%;max-width:720px" alt="MuJoCo头部画面"/>',sanitize=False)
    ui.link('进入头部相机标定 →','/camera-calibration')
    status=ui.label('').props('data-testid=simulation-status')
    joints=ui.label('').classes('font-mono text-sm')
    def refresh():
        import math
        state=backend.world.state()
        status.set_text(f'仿真电机：{"保持中" if state["enabled"] else "失能/支撑"} · 物理步数 {state["physics_steps"]} · '+
            ('已装板' if state['board_inserted'] else '未装板')+(f' · 错误：{state["error"]}' if state['error'] else ''))
        joints.set_text('实测模拟关节：'+' / '.join(f'{math.degrees(q):.1f}°' for q in state['positions_rad']))
    ui.timer(.3,refresh)
