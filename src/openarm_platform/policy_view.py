"""Native NiceGUI policy visualization inside the existing platform origin."""
import asyncio
import math
import time

from openarm_runtime import __version__

STATES = {'idle': '等待策略', 'starting': '初始化', 'running': '运行中',
          'stopped': '已结束', 'failed': '已停止 · 异常', 'incomplete': '记录未结束'}


def joint_label(name):
    side, joint = name.split('.')
    return ('左臂' if side == 'left' else '右臂') + ' · ' + ('夹爪 J8' if joint == 'gripper_motor' else 'J' + joint[5:])


def chart_data(snapshot, name):
    if name not in snapshot['names']:
        return [], []
    index = snapshot['names'].index(name)
    def series(kind):
        points = [[p['time'], None if p['values'][index] is None else math.degrees(p['values'][index])]
                  for p in snapshot['points'] if p['kind'] == kind]
        stride = max(1, math.ceil(len(points) / 700))
        result = points[::stride]
        if points and result[-1] != points[-1]:
            result.append(points[-1])
        return result
    return series('target'), series('feedback')


def joint_rows(snapshot, cursor):
    latest = {}
    for point in snapshot['points']:
        if point['time'] <= cursor:
            latest[point['kind']] = point['values']
    result = []
    for i, name in enumerate(snapshot['names']):
        target = latest.get('target', [None]*len(snapshot['names']))[i]
        measured = latest.get('feedback', [None]*len(snapshot['names']))[i]
        delta = None if target is None or measured is None else target-measured
        result.append(dict(name=name, joint=joint_label(name),
                           **{key: '—' if v is None else f'{math.degrees(v):+.2f}'
                              for key, v in [('target', target), ('feedback', measured), ('error', delta)]}))
    return result


class PolicyView:
    def __init__(self, preview, models=None):
        from nicegui import ui
        self.ui, self.preview = ui, preview
        self.playing = self.updating = False
        self.cursor = self.last_tick = 0.0
        self.signature = None
        self.was_busy = False
        self.render_key = None
        ui.colors(primary='#36735a')
        ui.add_css('''
            body { background: #f6f8f5; color: #263d32; }
            .policy-workspace { width:100%; max-width:none; margin:0; padding:14px 16px; gap:12px!important; }
            .policy-toolbar { width:100%; align-items:center; gap:8px!important; flex-wrap:wrap; }
            .policy-metrics { display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); width:100%; gap:8px; }
            .policy-metric { padding:10px 12px; background:white; border:1px solid #dfe7df; border-radius:7px; }
            .policy-metric-caption { font-size:12px; color:#6f8074; }
            .policy-metric-value { font-size:20px; font-weight:600; margin-top:3px; }
            .policy-card { width:100%; box-shadow:none!important; border:1px solid #dfe7df; border-radius:7px; padding:12px!important; gap:8px!important; }
            .policy-workspace .q-btn { min-height:32px; font-size:12px; }
            .policy-toolbar .model-action-button { flex:0 0 auto; min-width:5.25rem; min-height:2.25rem; padding:.375rem .875rem!important; white-space:nowrap; }
            .policy-toolbar .model-load-button { min-width:4.25rem; }
            .model-action-button .q-btn__content { flex-wrap:nowrap; gap:.375rem; white-space:nowrap; }
            .model-action-button .q-icon { font-size:1.125rem; }
            .model-action-button .q-icon.on-left { margin-right:0; }
            @media(max-width:650px) { .policy-workspace {padding:10px;} .policy-metrics {grid-template-columns:repeat(2,minmax(0,1fr));} .policy-toolbar .model-select {flex-basis:100%; min-width:0!important; max-width:none;} }
        ''')
        with ui.column().classes('policy-workspace') as workspace:
            with ui.row().classes('policy-toolbar'):
                with ui.column().classes('gap-1 grow'):
                    ui.label('模型与执行').classes('text-xl font-semibold')
                    with ui.label(f'OpenArm {__version__}').classes('text-xs text-grey-7'):
                        ui.tooltip('模型部署、独立仿真与记录回放。加载模型不会启动机械臂。')
            if models is not None:
                from .model_view import ModelView
                self.models = ModelView(models)
            legacy_start = len(workspace.default_slot.children)
            self.source = ui.badge('等待策略', color='primary').props('data-testid=policy-source')
            with ui.row().classes('policy-metrics'):
                self.metrics = {}
                for key, label in [('state', '会话状态'), ('duration', '采样时间'), ('commands', '已提交指令'), ('latency', '最近推理延迟')]:
                    with ui.column().classes('policy-metric gap-0'):
                        ui.label(label).classes('policy-metric-caption')
                        self.metrics[key] = ui.label('—').classes('policy-metric-value').props(f'data-testid=policy-{key}')
            with ui.card().classes('policy-card'):
                with ui.row().classes('policy-toolbar'):
                    self.policy = ui.select({'sine': 'J1 正弦演示', 'hold': '保持姿态'}, value='sine', label='仿真策略').classes('w-48')
                    self.duration = ui.number('时长（秒）', value=10, min=1, max=60, step=1).classes('w-28')
                    self.start_button = ui.button('开始仿真', icon='play_arrow', on_click=self.start).props('data-testid=policy-start')
                    self.stop_button = ui.button('停止仿真', icon='stop', on_click=preview.stop).props('outline data-testid=policy-stop')
                with self.start_button:
                    ui.tooltip('仿真使用理想位置模型，不控制实物；指令与反馈分别显示。')
                with ui.expansion('打开策略记录', icon='folder_open').classes('w-full'):
                    ui.label(str(preview.record_root)).classes('text-xs text-grey-7 break-all')
                    with ui.row().classes('policy-toolbar'):
                        self.episode = ui.select(preview.episodes(), label='会话目录').classes('grow').props('data-testid=policy-episode')
                        ui.button('刷新列表', icon='refresh', on_click=self.refresh_episodes).props('flat')
                        self.load_button = ui.button('加载回放', on_click=self.load).props('outline data-testid=policy-load')
                    with self.load_button:
                        ui.tooltip('读取会话中的 events.jsonl；显示记录时的反馈，不控制实物。')
            self.error = ui.label('').classes('text-red-8 whitespace-pre-wrap').props('role=alert data-testid=policy-error')
            with ui.card().classes('policy-card'):
                with ui.row().classes('policy-toolbar'):
                    ui.label('目标与反馈轨迹').classes('text-lg font-medium grow')
                    self.joint = ui.select({}, label='关节', on_change=lambda _: self.render()).classes('w-44').props('data-testid=policy-joint')
                self.chart = ui.echart({
                    'animation': False, 'tooltip': {'trigger': 'axis'},
                    'legend': {'data': ['提交目标', '反馈采样']},
                    'grid': {'left': 60, 'right': 28, 'top': 44, 'bottom': 48},
                    'xAxis': {'type': 'value', 'name': '秒'}, 'yAxis': {'type': 'value', 'name': '角度（°）', 'scale': True},
                    'series': [
                        {'name': '提交目标', 'type': 'line', 'step': 'end', 'showSymbol': False, 'data': [], 'color': '#b58b48'},
                        {'name': '反馈采样', 'type': 'line', 'showSymbol': False, 'connectNulls': False, 'data': [], 'color': '#36735a'},
                    ],
                }).classes('w-full h-80').props('data-testid=policy-chart')
                with ui.row().classes('policy-toolbar'):
                    self.play_button = ui.button('播放回放', icon='play_arrow', on_click=self.toggle_play).props('flat data-testid=policy-play')
                    self.slider = ui.slider(min=0, max=1, step=.01, value=0, on_change=self.seek).classes('grow').props('data-testid=policy-time')
                    self.time_label = ui.label('0.00 s').classes('text-sm')
                with self.slider:
                    ui.tooltip('图表保留最近 2,000 个采样事件；拖动查看对应关节值。')
            columns = [dict(name=key, label=label, field=key, align='left') for key, label in
                       [('joint', '关节'), ('target', '提交目标（°）'), ('feedback', '反馈采样（°）'), ('error', '目标 − 反馈（°）')]]
            self.table = ui.table(columns=columns, rows=[], row_key='name').classes('policy-card').props('flat dense data-testid=policy-joints')
            if models is not None:
                legacy = ui.expansion('仿真与回放', icon='history').classes('w-full')
                with legacy:
                    ui.tooltip('独立仿真与历史回放，不控制实物。')
                for element in list(workspace.default_slot.children)[legacy_start:-1]:
                    element.move(legacy)
        self.render()
        ui.timer(.25, self.render)

    async def start(self):
        try:
            self.preview.start(self.policy.value, self.duration.value)
            self.playing = False
            self.signature = None
            self.render()
        except (ValueError, RuntimeError) as exc:
            self.ui.notify(str(exc), type='negative')

    def refresh_episodes(self):
        try:
            self.episode.set_options(self.preview.episodes())
        except OSError as exc:
            self.ui.notify(str(exc), type='negative')

    async def load(self):
        try:
            if not self.episode.value:
                raise ValueError('请先选择一个记录会话')
            await asyncio.to_thread(self.preview.load, self.episode.value)
            self.playing = False
            self.signature = None
            self.render()
        except (OSError, ValueError, RuntimeError, KeyError, TypeError, ImportError) as exc:
            self.ui.notify(str(exc), type='negative')

    def seek(self, event):
        if not self.updating:
            self.cursor = float(event.value)
            self.playing = False
            self.render()

    def toggle_play(self):
        self.playing = not self.playing
        if self.playing and self.cursor >= self.slider._props['max']:
            self.cursor = self.slider._props['min']
        self.last_tick = time.monotonic()
        self.render()

    def render(self):
        snapshot = self.preview.snapshot()
        points, busy = snapshot['points'], snapshot['busy']
        start = min((p['time'] for p in points), default=0.0)
        end = max((p['time'] for p in points), default=0.0)
        signature = (snapshot['generation'], snapshot['source'], tuple(snapshot['names']))
        now = time.monotonic()
        if signature != self.signature:
            self.signature = signature
            self.cursor = end
            options = {n: joint_label(n) for n in snapshot['names']}
            self.joint.set_options(options, value=next(iter(options), None))
        if busy or self.was_busy:
            self.cursor, self.playing = end, False
        elif self.playing:
            self.cursor = min(end, self.cursor + max(0, now-self.last_tick))
            if self.cursor >= end:
                self.playing = False
        self.was_busy = busy
        self.last_tick = now
        self.cursor = max(start, min(end, self.cursor))
        key = (signature, snapshot['revision'], busy, self.cursor, self.playing, self.joint.value)
        if key == self.render_key:
            return
        self.render_key = key
        self.updating = True
        self.slider._props.update(min=start, max=max(start+.01, end))
        self.slider.value = self.cursor
        self.slider.update()
        self.updating = False
        self.slider.set_enabled(bool(points) and not busy)
        self.play_button.set_enabled(bool(points) and not busy)
        self.play_button.set_text('暂停回放' if self.playing else '播放回放')
        self.start_button.set_enabled(not busy)
        self.stop_button.set_enabled(busy and not self.preview.loading)
        self.load_button.set_enabled(not busy)
        self.source.set_text(snapshot['source'])
        self.metrics['state'].set_text(STATES.get(snapshot['state'], snapshot['state']))
        self.metrics['duration'].set_text(f'{end:.1f} s')
        self.metrics['commands'].set_text(str(snapshot['commands']))
        self.metrics['latency'].set_text('—' if snapshot['latency_ms'] is None else f"{snapshot['latency_ms']:.1f} ms")
        self.error.set_text(snapshot['error'])
        self.time_label.set_text(f'{self.cursor:.2f} s')
        target, feedback = chart_data(snapshot, self.joint.value)
        for series, data in zip(self.chart.options['series'], (target, feedback)):
            series['data'] = data
        self.chart.options['series'][0]['markLine'] = {'silent': True, 'symbol': 'none',
            'label': {'show': False}, 'data': [{'xAxis': self.cursor}]}
        self.chart.update()
        self.table.rows = joint_rows(snapshot, self.cursor)
        self.table.update()


def build_policy_view(preview, models=None):
    return PolicyView(preview, models)
