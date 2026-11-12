"""Model lifecycle controls embedded in the one existing policy workspace."""
import asyncio
import time
from urllib.parse import quote


LABELS = {'unloaded': '未加载', 'loading': '加载中', 'checking': '加载中', 'loaded': '已加载',
          'tested': '已就绪', 'error': '加载失败'}
EXECUTION = {'idle': '未执行', 'starting': '正在交接控制', 'running': '执行中', 'stopping': '正在停止并清理',
             'completed': '会话已结束', 'stopped_error': '异常停止 · 清理已确认', 'failed': '需现场检查'}


class ModelView:
    def __init__(self, workspace):
        from nicegui import ui
        from .video_stream import video_stream
        self.video_stream = video_stream
        self.ui, self.workspace = ui, workspace
        self.busy = self.polling = False
        self.generation = None
        self.input_layout = None
        self.input_images = []
        self.model_streams = []
        self.action_error = ''
        self.client = ui.context.client
        self.owner = self.client.id
        with ui.card().classes('policy-card'):
            with ui.row().classes('policy-toolbar'):
                ui.label('模型部署').classes('text-base font-medium')
                self.status = ui.label('').classes('text-xs text-grey-7').props('data-testid=model-state')
                self.status_hint = self.tip(self.status, '选择模型并加载；加载会自动完成身份和输入输出检查，不会启动机械臂。')
                self.nominal = ui.label('名义几何').classes('text-xs text-orange-9')
                self.tip(self.nominal, '当前几何未独立实测标定。启动属于操作员监督的研究试运行，不代表实物已验收。')
            with ui.row().classes('policy-toolbar'):
                self.select = ui.select({}, label='模型', on_change=self.select_connection).classes('grow min-w-48 max-w-lg').props('dense outlined data-testid=model-select')
                self.load = ui.button('加载', on_click=lambda: self.act('load', self.select.value)).props('dense data-testid=model-load')
                self.tip(self.load, '加载并自动检查。空闲时可直接切换模型，无需先手动卸载；不会启用电机。')
                self.refresh = ui.button(icon='refresh', on_click=lambda: self.act('refresh')).props('flat dense round aria-label="刷新模型目录"')
                self.tip(self.refresh, '重新读取已注册模型目录。')
                self.unload = ui.button(icon='power_settings_new', on_click=lambda: self.act('unload')).props('flat dense round aria-label="卸载模型" data-testid=model-unload')
                self.tip(self.unload, '卸载空闲模型，释放推理进程；执行中不可卸载。')
            self.connection = ui.label('').classes('text-xs').props('role=status data-testid=model-connection')
            self.connection_hint = self.tip(self.connection, '')
            self.blockers = ui.label('').classes('text-xs text-orange-9').props('data-testid=model-blockers')
            self.blockers_hint = self.tip(self.blockers, '')
            with ui.row().classes('policy-toolbar'):
                self.start = ui.button('启动', icon='play_arrow', on_click=self.confirm_start).props('dense data-testid=model-start')
                self.tip(self.start, '完成准备后持续执行，直到手动停止或安全保护触发。请确认现场空间及机械臂状态；加载模型本身不会运动。')
                self.stop = ui.button('停止', icon='stop', on_click=lambda: self.act('stop')).props('outline dense data-testid=model-stop')
                self.tip(self.stop, '请求停止当前会话并完成失能清理。停止或故障不会触发自动返程。')
                self.home = ui.button('归位', icon='keyboard_arrow_down', on_click=self.confirm_home).props('outline dense data-testid=model-home')
                self.tip(self.home, '按已配置路径主动返回自然下垂姿态，夹爪保持；不是避障规划，可随时停止。')
            self.home_note = ui.label('').classes('text-xs text-orange-9')
            self.tip(self.home_note, '当前执行配置缺少自然下垂姿态或反向准备路径；需在个人 profile 中配置。')
            self.error = ui.label('').classes('text-red-8').props('role=alert tabindex=0 data-testid=model-error')
            self.error_hint = self.tip(self.error, '')
            self.input_note = ui.label('尚无模型输入').classes('text-xs text-grey-7')
            self.input_hint = self.tip(self.input_note, '')
            self.inputs = ui.row().classes('w-full gap-3')
            self.rerun_link = ui.link('记录', '/api/models/recording').props('target=_blank')
            self.tip(self.rerun_link, '下载本轮 Rerun 记录；仅在已结束且存在录制时可用。')
            with ui.expansion('详情').classes('w-full'):
                self.description = ui.label('').classes('text-sm whitespace-pre-wrap').props('data-testid=model-description')
                self.identity = ui.label('').classes('text-xs text-grey-7 break-all').props('data-testid=model-identity')
                self.geometry = ui.label('').classes('text-xs text-grey-7')
                self.record = ui.label('').classes('text-xs text-grey-7 break-all')
                self.input_state = ui.label('').classes('text-xs text-grey-7 break-all')
        self.client.on_disconnect(self.disconnect)
        ui.timer(.5, self.render)
        self.render()

    def tip(self, element, text):
        with element:
            return self.ui.tooltip(text).classes('max-w-sm whitespace-pre-wrap')

    async def disconnect(self):
        await asyncio.to_thread(self.workspace.stop, owner=self.owner)

    async def select_connection(self, event):
        if event.value is not None:
            try:
                await asyncio.to_thread(self.workspace.select_connection, event.value)
            except Exception as exc:
                self.connection.set_text('连接配置异常')
                self.connection_hint.set_text(str(exc))

    async def act(self, name, *args, **kwargs):
        # Stop remains available while automatic loading/checking is waiting.
        if self.busy and name != 'stop':
            return
        if name != 'stop':
            self.busy = True
        try:
            await asyncio.to_thread(getattr(self.workspace, name), *args, **kwargs)
            self.action_error = ''
        except Exception as exc:
            connection = self.workspace.connectivity.snapshot()
            waiting = name in ('start', 'return_home') and connection.get('enabled') and connection.get('state') == 'waiting'
            self.ui.notify('等待 CAN 连接' if waiting else '操作失败，悬停异常状态查看详情', type='info' if waiting else 'negative')
            if waiting:
                self.connection.set_text('CAN 连接中')
                self.connection_hint.set_text(str(exc))
            else:
                self.action_error = str(exc)
        finally:
            if name != 'stop':
                self.busy = False
            self.render()

    async def confirm_start(self):
        state = self.workspace.snapshot()
        entry = next((m for m in state['models'] if m['id'] == state['selected']), None)
        if entry is None or not state['can_start']:
            self.ui.notify('请先加载模型，等待就绪', type='warning')
            return
        if self.select.value != state['selected']:
            self.ui.notify('请先加载当前选择的模型', type='warning')
            return
        # The start button is the operator action, not a calibration assertion.
        # Only the backend's explicitly declared nominal profile is eligible.
        await self.act('start', confirmed=True, owner=self.owner,
                       allow_nominal=state['requires_nominal_opt_in'])

    async def confirm_home(self):
        state = self.workspace.snapshot()
        entry = next((m for m in state['models'] if m['id'] == self.select.value), None)
        if not state.get('can_return_home') or not entry or not entry.get('home_configured'):
            self.ui.notify('归位暂不可用', type='warning')
            return
        await self.act('return_home', self.select.value, confirmed=True, owner=self.owner,
                       allow_nominal=entry.get('home_requires_nominal', False))

    def render(self):
        try:
            self._render()
        except Exception as exc:
            # Keep the timer and stop control usable while files/services are
            # temporarily unavailable. No automatic model or motion restart.
            self.error.set_text('状态暂不可用')
            self.error_hint.set_text(str(exc))
            self.error.set_visibility(True)
            self.start.set_enabled(False)
            self.home.set_enabled(False)
            self.stop.set_enabled(not self.workspace.readonly)

    def _render(self):
        state = self.workspace.snapshot()
        if self.generation != state['generation']:
            self.generation = state['generation']
        options = {m['id']: m['label'] for m in state['models']}
        if self.select.options != options:
            value = self.select.value if self.select.value in options else state['selected'] or next(iter(options), None)
            self.select.set_options(options, value=value)
        entry = next((m for m in state['models'] if m['id'] == self.select.value), None)
        inputs = self.workspace.input_status(self.select.value)
        layout = (self.select.value, inputs['run_id'], tuple(inputs['cameras']), tuple(inputs['images']))
        if layout != self.input_layout:
            self.input_layout = layout
            self.inputs.clear()
            self.input_images = []
            self.model_streams = []
            with self.inputs:
                for name in inputs['cameras']:
                    with self.ui.column().classes('grow min-w-48 max-w-sm'):
                        self.ui.label(name+' · 相机预览').classes('text-xs')
                        path = '/apps/vision/stream/'+quote(name, safe='')+'.mjpg'
                        image = self.video_stream(path).classes('w-full')
                        self.input_images.append((image, path))
                for key in inputs['images']:
                    with self.ui.column().classes('grow min-w-48 max-w-xs'):
                        self.ui.label(key+' · 实际预测输入').classes('text-xs')
                        path = '/api/models/input-stream/'+quote(key, safe='')+'?run='+quote(inputs['run_id'] or '', safe='')
                        image = self.video_stream(path, reconnect=state['running']).classes('w-full')
                        self.model_streams.append(image)
                        self.input_images.append((image, path))
        for stream in self.model_streams:
            stream.set_reconnect(state['running'])
        age = None if inputs['recorded_at'] is None else max(0., time.monotonic()-inputs['recorded_at'])
        self.input_note.set_text('输入 #'+str(inputs['observation_sequence'])+f' · {age:.1f}秒前'
                                 if age is not None else '相机预览')
        self.input_hint.set_text('实际预测输入按推理更新；时间是主机记录时间，不代表相机曝光同步。'
                                 if age is not None else '模型第一次推理后显示实际输入；当前仅为相机源预览。')
        self.input_note.set_visibility(entry is not None)
        self.input_state.set_text('输入状态：'+str(inputs['state']) if inputs['state'] else '')
        self.input_state.set_visibility(bool(inputs['state']))
        self.rerun_link.set_visibility(inputs['rerun_available'])
        self.description.set_text(entry['description'] if entry else '未配置模型目录')
        self.identity.set_text(' · '.join(f'{k}: {v}' for k, v in entry['identity'].items()) if entry else '')
        sample = state['sample_test']
        status_detail = (f"{LABELS[state['state']]} · {EXECUTION[state['execution']]}"
            + (' · 正在归位到自然下垂' if state.get('phase') == 'return_home' and state['running'] else '')
            + (' · 已归位到自然下垂' if state.get('home_completed') else '')
            + (' · 已到设定时长' if state.get('stop_reason') == 'duration_reached' else '')
            + (f" · {sample['latency_s']*1000:.1f} ms" if sample else ''))
        self.status.set_text('归位中' if state.get('phase') == 'return_home' and state['running'] else
                             EXECUTION[state['execution']] if state['execution'] != 'idle' else LABELS[state['state']])
        self.status_hint.set_text(status_detail + '\n加载时自动检查身份和样例输入输出，不读取设备或启动机械臂。')
        self.load.set_text('加载中' if state['state'] in ('loading', 'checking') else '加载')
        self.blockers.set_text('执行配置未就绪')
        self.blockers_hint.set_text('\n'.join(state['blockers']))
        self.blockers.set_visibility(bool(state['blockers']))
        self.geometry.set_text('几何：名义模型，未实测标定' if state['requires_nominal_opt_in'] else '')
        self.nominal.set_visibility(state['requires_nominal_opt_in'])
        connection = state.get('connection', {})
        detail = []
        for side in connection.get('sides', []):
            text = side['side']+' / '+side['interface']+'：'+side['message']
            if side.get('retry_delay_s'):
                text += f"（{side['retry_delay_s']:g} 秒退避重试）"
            if side.get('error'):
                text += ' · '+side['error']
            detail.append(text)
        self.connection.set_text({'ready':'CAN 就绪', 'waiting':'CAN 连接中', 'busy':'CAN 控制占用',
                                 'blocked':'CAN 需检查'}.get(connection.get('state'), 'CAN 待检查'))
        self.connection_hint.set_text('\n'.join([connection.get('message', ''), *detail]))
        self.connection.set_visibility(bool(connection.get('enabled')) and bool(entry and entry['robot'] == 'classic'))
        error = state['error'] or self.action_error
        self.error.set_text('加载失败' if state['state'] == 'error' else '执行异常')
        self.error_hint.set_text(error)
        self.error.set_visibility(bool(error))
        self.record.set_text('会话记录：' + state['record'] if state['record'] else '')
        writable = not state['readonly'] and not self.busy
        idle = not state['running'] and state['state'] not in ('loading', 'checking')
        unloaded = state['state'] in ('unloaded', 'error')
        self.select.set_enabled(writable and idle)
        self.refresh.set_enabled(writable and idle and unloaded)
        self.load.set_enabled(writable and idle and entry is not None and
                              (unloaded or self.select.value != state['selected']))
        self.unload.set_enabled(writable and idle and not unloaded)
        self.start.set_enabled(writable and state['can_start'] and self.select.value == state['selected'])
        home_configured = bool(entry and entry.get('home_configured'))
        connection_allows = (not entry or entry.get('robot') != 'classic' or not connection.get('enabled')
                             or connection.get('state') in ('ready', 'busy'))
        self.home.set_enabled(writable and home_configured and state.get('can_return_home', False) and connection_allows)
        self.home_note.set_text('归位未配置')
        self.home_note.set_visibility(bool(entry) and not home_configured)
        self.stop.set_enabled(not state['readonly'] and (state['running'] or state['state'] in ('loading', 'checking')))
