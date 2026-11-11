"""Model lifecycle controls embedded in the one existing policy workspace."""
import asyncio
import time
from urllib.parse import quote


LABELS = {'unloaded': '未加载', 'loading': '加载中', 'loaded': '已加载',
          'tested': '样例预测通过', 'error': '模型异常'}
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
        self.client = ui.context.client
        self.owner = self.client.id
        with ui.card().classes('policy-card'):
            ui.label('模型部署').classes('text-lg font-medium')
            with ui.row().classes('policy-toolbar'):
                self.select = ui.select({}, label='已注册模型').classes('grow').props('data-testid=model-select')
                self.refresh = ui.button('刷新目录', on_click=lambda: self.act('refresh')).props('flat')
                self.load = ui.button('加载模型', on_click=lambda: self.act('load', self.select.value)).props('data-testid=model-load')
                self.test = ui.button('测试预测', on_click=lambda: self.act('test')).props('outline data-testid=model-test')
                self.unload = ui.button('卸载', on_click=lambda: self.act('unload')).props('flat data-testid=model-unload')
            self.status = ui.label('').classes('text-sm').props('data-testid=model-state')
            self.blockers = ui.label('').classes('text-orange-9 whitespace-pre-wrap').props('data-testid=model-blockers')
            with ui.row().classes('policy-toolbar'):
                self.duration = ui.number('策略时长（秒，准备另计）', value=10, min=1, max=30).classes('w-48')
                self.start = ui.button('启动选中模型', icon='play_arrow', on_click=self.confirm_start).props('data-testid=model-start')
                self.stop = ui.button('停止执行', icon='stop', on_click=lambda: self.act('stop')).props('outline data-testid=model-stop')
            self.error = ui.label('').classes('text-red-8 whitespace-pre-wrap').props('role=alert data-testid=model-error')
            self.input_note = ui.label('尚无模型输入').classes('text-xs text-grey-7')
            self.inputs = ui.row().classes('w-full gap-3')
            self.input_state = ui.label('').classes('text-xs text-grey-7 break-all')
            self.rerun_link = ui.link('下载本轮 Rerun 记录', '/api/models/recording').props('target=_blank')
            with ui.expansion('技术详情').classes('w-full'):
                self.description = ui.label('').classes('text-sm whitespace-pre-wrap').props('data-testid=model-description')
                self.identity = ui.label('').classes('text-xs text-grey-7 break-all').props('data-testid=model-identity')
                self.geometry = ui.label('').classes('text-xs text-grey-7')
                self.record = ui.label('').classes('text-xs text-grey-7 break-all')
        self.client.on_disconnect(self.disconnect)
        ui.timer(.5, self.render)
        self.render()

    async def disconnect(self):
        await asyncio.to_thread(self.workspace.stop, owner=self.owner)

    async def act(self, name, *args, **kwargs):
        # Stop remains available even while load/test is waiting.
        if self.busy and name != 'stop':
            return
        if name != 'stop':
            self.busy = True
        try:
            await asyncio.to_thread(getattr(self.workspace, name), *args, **kwargs)
        except Exception as exc:
            self.ui.notify(str(exc), type='negative')
            self.error.set_text(str(exc))
        finally:
            if name != 'stop':
                self.busy = False
            self.render()

    async def confirm_start(self):
        state = self.workspace.snapshot()
        entry = next((m for m in state['models'] if m['id'] == state['selected']), None)
        if entry is None or not state['can_start']:
            self.ui.notify('请先加载、测试并解决执行配置中的问题', type='warning')
            return
        if self.select.value != state['selected']:
            self.ui.notify('下拉选择与已加载模型不同，请先卸载并加载所选模型', type='warning')
            return
        duration = self.duration.value
        # The start button is the operator action, not a calibration assertion.
        # Only the backend's explicitly declared nominal profile is eligible.
        await self.act('start', duration, confirmed=True, owner=self.owner,
                       allow_nominal=state['requires_nominal_opt_in'])

    def render(self):
        state = self.workspace.snapshot()
        if self.generation != state['generation']:
            self.generation = state['generation']
        options = {m['id']: m['label'] for m in state['models']}
        if self.select.options != options:
            value = self.select.value if self.select.value in options else state['selected'] or next(iter(options), None)
            self.select.set_options(options, value=value)
        if state['selected'] is not None and state['state'] not in ('unloaded', 'error'):
            self.select.value = state['selected']
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
        self.input_note.set_text('实际输入：推理帧 #'+str(inputs['observation_sequence'])+f' · {age:.1f}秒前'
                                 if age is not None else '相机源预览 · 实际预测输入将在首次推理后显示')
        self.input_state.set_text('输入状态：'+str(inputs['state']) if inputs['state'] else '')
        self.rerun_link.set_visibility(inputs['rerun_available'])
        self.description.set_text(entry['description'] if entry else '未配置模型目录；在工作站的platform profile中设置model_catalog。')
        self.identity.set_text(' · '.join(f'{k}: {v}' for k, v in entry['identity'].items()) if entry else '')
        sample = state['sample_test']
        self.status.set_text(f"{LABELS[state['state']]} · {EXECUTION[state['execution']]}"
            + (' · 已到设定时长' if state.get('stop_reason') == 'duration_reached' else '')
            + (f" · {sample['latency_s']*1000:.1f} ms" if sample else ''))
        self.blockers.set_text('\n'.join(state['blockers']))
        self.geometry.set_text('几何：名义模型，未实测标定' if state['requires_nominal_opt_in'] else '')
        self.error.set_text(state['error'])
        self.record.set_text('会话记录：' + state['record'] if state['record'] else '')
        writable = not state['readonly'] and not self.busy
        idle = not state['running']
        unloaded = state['state'] in ('unloaded', 'error')
        self.select.set_enabled(writable and idle and unloaded)
        self.refresh.set_enabled(writable and idle and unloaded)
        self.load.set_enabled(writable and idle and unloaded and entry is not None)
        self.test.set_enabled(writable and idle and state['state'] in ('loaded', 'tested')
                              and entry is not None and entry['sample_available'])
        self.unload.set_enabled(writable and idle and not unloaded)
        self.start.set_enabled(writable and state['can_start'] and self.select.value == state['selected'])
        self.stop.set_enabled(not state['readonly'] and (state['running'] or state['state'] == 'loading'))
        if entry:
            self.duration._props['max'] = entry['max_duration_s']
