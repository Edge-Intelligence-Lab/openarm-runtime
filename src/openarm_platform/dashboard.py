"""NiceGUI workspace; optional UI dependencies load only when a page is built.

Each browser owns one dashboard and a cache of iframe elements. Moving to a
sensing page changes visibility, preserving the controlling browser connection.
"""
from dataclasses import dataclass
from importlib.resources import files
import json


ICONS = {'control': 'tune', 'teleop': 'precision_manufacturing',
         'tactile': 'touch_app', 'vision': 'videocam', 'policy': 'timeline', 'collection': 'storage',
         'camera_calibration': 'camera'}
STATUS_LABELS = {'running': '服务可达', 'external': '服务可达', 'starting': '启动中',
                 'standby': '待切换', 'stopped': '已停止', 'unconfigured': '未配置',
                 'error': '服务异常'}


def service_ready(state):
    return state.get('reachable') is True and state.get('state') != 'error'


def ssh_command(local_port, remote_port, remote_host='127.0.0.1'):
    if remote_host == '0.0.0.0':
        remote_host = '127.0.0.1'
    return f'ssh -N -L {int(local_port)}:{remote_host}:{int(remote_port)} USER@WORKSTATION'


@dataclass
class FrameView:
    element: object
    generation: int


class Dashboard:
    """Per-client view state around the application's async service coordinator."""

    def __init__(self, config, coordinator, *, attach_only=False):
        from nicegui import ui
        self.ui = ui
        self.client = ui.context.client
        self.config = config
        self.coordinator = coordinator
        self.attach_only = attach_only
        self.pages = {page.id: page for page in config.pages}
        self.statuses = {}
        self.frames = {}
        self.active = None
        self.busy = False
        self.polling = False
        self.revision = 0
        self.selection_error = ''
        self.status_error = False
        self.stopping = False
        self.connection_state = {}
        self.nav = {}
        self.cards = {}
        self.badges = {}
        self._build()

    @staticmethod
    def identify(element, identity):
        return element.props(f'id={identity} data-testid={identity}')

    def _build(self):
        ui, tag = self.ui, self.identify
        ui.add_css(files('openarm_platform').joinpath('static/platform.css').read_text())
        ui.colors(primary='#36735a', secondary='#788d74', positive='#36735a', warning='#9a794a')
        with ui.row().classes('platform-shell'):
            with ui.column().classes('platform-main'):
                with ui.row().classes('platform-header'):
                    self.home = tag(ui.button('OpenArm', icon='precision_manufacturing', on_click=self.overview)
                        .props('flat dense no-caps aria-label="工作空间概览"').classes('brand-button'), 'home-button')
                    self.mode_button = tag(ui.button('概览', icon='expand_more').props('flat dense no-caps')
                        .classes('mode-button'), 'mode-switcher')
                    self.current_label = self.mode_button  # Keep one accessible current-mode label.
                    with self.mode_button:
                        with ui.menu().classes('mode-menu') as self.mode_menu:
                            ui.label('切换工作空间').classes('menu-caption')
                            self.overview_item = ui.button('概览', icon='grid_view', on_click=lambda: self.choose_mode())
                            self.overview_item.props('flat dense no-caps').classes('nav-button')
                            with tag(ui.column().classes('platform-navigation'), 'navigation'):
                                for page in self.pages.values():
                                    self.nav[page.id] = tag(ui.button(page.label, icon=ICONS[page.id],
                                        on_click=lambda _, name=page.id: self.choose_mode(name)).props('flat dense no-caps')
                                        .classes('nav-button'), f'nav-{page.id}')
                    ui.element('div').classes('header-spacer')
                    if self.config.simulation_profile is not None:
                        tag(ui.label('仿真 · 实物禁用').classes('simulation-badge'), 'simulation-banner')
                    with tag(ui.row().classes('page-actions'), 'page-actions') as self.actions:
                        self.current_status = tag(ui.label('状态未知').classes('current-status'), 'current-status')
                    self.actions.set_visibility(False)
                    self.connection_button = tag(ui.button('CAN 待检查', icon='circle', on_click=self.show_connection)
                        .props('flat dense no-caps').classes('connection-status'), 'connection-toggle')
                    self.stop_button = tag(ui.button('停止运动', icon='stop', color='negative', on_click=self.stop_motion)
                        .props('unelevated dense no-caps').classes('motion-stop'), 'stop-motion')
                    self.stop_button.set_enabled(not self.attach_only)
                    with self.stop_button:
                        ui.tooltip('请求停止本平台管理的运动；不是硬件急停。外部控制需在原入口停止。')
                    with tag(ui.button(icon='more_horiz').props('flat round dense aria-label="更多操作"'), 'more-actions'):
                        with ui.menu().classes('mode-menu'):
                            self.reload = tag(ui.button('重新加载当前模块', icon='refresh', on_click=self.refresh)
                                .props('flat dense no-caps').classes('nav-button'), 'reload-page')
                            self.external = tag(ui.link('新窗口打开 ↗', '/', new_tab=True).classes('external-link'), 'external-page')
                            ui.button('连接设置与帮助', icon='link', on_click=self.show_connection).props('flat dense no-caps').classes('nav-button')
                self.notice = tag(ui.label('').classes('service-notice').props('role=status aria-live=polite'), 'service-notice')
                self.notice.set_visibility(False)
                with tag(ui.column().classes('platform-overview'), 'overview') as self.overview_panel:
                    tag(ui.label(self.config.title).classes('overview-title'), 'overview-title')
                    ui.label('选择工作空间；使用过程中也可从顶部切换。').classes('overview-description')
                    with tag(ui.grid(columns=2).classes('module-grid'), 'module-grid'):
                        for index, page in enumerate(self.pages.values(), 1):
                            with ui.card().classes('module-card'):
                                with ui.row().classes('card-top'):
                                    ui.icon(ICONS[page.id], size='23px').classes('card-icon')
                                    ui.label(f'{index:02d}').classes('card-number')
                                ui.label(page.label).classes('card-title')
                                ui.label(page.description).classes('card-description')
                                with ui.row().classes('card-bottom'):
                                    self.badges[page.id] = tag(ui.label('状态未知').classes('module-status'), f'status-{page.id}')
                                    self.cards[page.id] = tag(ui.button('打开', icon='north_east',
                                        on_click=lambda _, name=page.id: self.select(name))
                                        .props(f'flat dense no-caps aria-label="打开{page.label}"'), f'open-{page.id}')
                    ui.label('控制与遥操共享电机连接，切换时执行原有停止流程。触觉与视觉保持独立。').classes('overview-note')
                with tag(ui.element('section').classes('platform-views'), 'views') as self.views:
                    with tag(ui.column().classes('view-waiting'), 'view-waiting') as self.waiting:
                        ui.spinner(size='26px', color='primary')
                        self.waiting_title = tag(ui.label('等待服务').classes('waiting-title'), 'waiting-title')
                        ui.label('服务就绪后自动显示模块。').classes('waiting-description')
                self.views.set_visibility(False)
                with ui.row().classes('platform-footer'):
                    self.footer = tag(ui.label(f'{len(self.pages)} 个模块 · 同一个工作空间').classes('footer-caption'), 'footer-caption')
                    tag(ui.button('连接帮助', icon='north_east', on_click=self.show_connection)
                        .props('flat dense no-caps').classes('footer-help'), 'footer-connection')
        with tag(ui.dialog(), 'connection-dialog') as self.connection_dialog:
            with ui.card().classes('connection-card'):
                with ui.row().classes('connection-heading'):
                    ui.label('连接工作站').classes('dialog-title')
                    tag(ui.button(icon='close', on_click=self.connection_dialog.close)
                        .props('flat round dense aria-label="关闭连接设置"'), 'connection-close')
                ui.label('在本机直接打开平台。使用其他电脑时，只需保持这一条 SSH 转发开启。').classes('connection-description')
                self.connection_detail = ui.label('连接状态尚未读取').classes('connection-description whitespace-pre-wrap')
                self.command = tag(ui.label(ssh_command(self.config.port, self.config.port,self.config.host)).classes('ssh-command'), 'ssh-command')
                tag(ui.button('复制命令', icon='content_copy', on_click=self.copy_command).props('no-caps'), 'copy-command')
                ui.label('画面和实时连接都通过这一个入口。设备是否正常，请查看模块内的状态。').classes('connection-note')
        with tag(ui.dialog(), 'confirm-dialog') as self.confirm_dialog:
            with ui.card().classes('confirm-card'):
                ui.label('确认操作').classes('dialog-title')
                self.confirm_message = tag(ui.label('').classes('confirm-message'), 'confirm-message')
                with ui.row().classes('dialog-actions'):
                    ui.button('停止运动', icon='stop', on_click=self.stop_motion).props('flat dense no-caps').classes('text-red-8')
                    tag(ui.button('取消', on_click=lambda: self.confirm_dialog.submit(False)).props('flat no-caps'), 'confirm-cancel')
                    tag(ui.button('继续', on_click=lambda: self.confirm_dialog.submit(True)).props('no-caps'), 'confirm-continue')

    def start(self):
        self.ui.timer(.1, self.initialize, once=True)
        self.ui.timer(3.0, self.poll)

    async def initialize(self):
        await self.client.connected()
        location = await self.ui.run_javascript('({port: location.port || (location.protocol === "https:" ? "443" : "80"), hash: location.hash.slice(1)})')
        self.command.set_text(ssh_command(location['port'], self.config.port,self.config.host))
        await self.poll()
        if location.get('hash') in self.pages:
            await self.select(location['hash'])

    async def _set_hash(self, name=None):
        value = '#' + name if name else ''
        self.ui.run_javascript('history.replaceState(null, "", location.pathname + location.search + ' + json.dumps(value) + ')')

    async def overview(self):
        if self.busy:
            return
        self.active = None
        self.selection_error = ''
        self._render()
        await self._set_hash()

    async def choose_mode(self, name=None):
        # Closing/opening the menu never removes controller iframes.
        self.mode_menu.close()
        if name is None:
            await self.overview()
        else:
            await self.select(name)

    async def stop_motion(self):
        if self.attach_only or self.stopping:
            return
        self.stopping = True
        if self.confirm_dialog.value:
            self.confirm_dialog.submit(False)
        self.stop_button.set_enabled(False)
        try:
            self.statuses = await self.coordinator.stop_motion()
            self.selection_error = '已请求停止运动，请等待原控制会话完成退出清理。'
        except Exception as exc:
            self.selection_error = str(exc) or '停止请求未确认，请现场检查。'
        finally:
            self.stopping = False
            self._render()

    async def show_connection(self):
        # SSH local and remote ports may intentionally differ.
        try:
            port = await self.ui.run_javascript('location.port || (location.protocol === "https:" ? "443" : "80")')
            self.command.set_text(ssh_command(port, self.config.port,self.config.host))
        except (TimeoutError, RuntimeError):
            pass
        self.connection_dialog.open()

    async def copy_command(self):
        try:
            await self.ui.run_javascript('navigator.clipboard.writeText(' + json.dumps(self.command.text) + ').then(() => true)')
            self.ui.notify('命令已复制', type='positive')
        except Exception:
            self.ui.notify('请手动复制上面的命令', type='warning')

    async def poll(self):
        if self.busy or self.polling:
            return
        self.polling = True
        revision = self.revision
        try:
            states = await self.coordinator.status()
            if revision == self.revision:
                self.statuses = states
                self.status_error = False
            try:
                connection = await self.coordinator.connection_status()
                if isinstance(connection, dict):
                    self.connection_state = connection
            except Exception:
                self.connection_state = dict(state='unknown', message='连接状态暂不可用，正在重试')
        except Exception:
            if revision == self.revision:
                self.status_error = True
        finally:
            self.polling = False
            self._render()

    async def select(self, name):
        if self.busy or name not in self.pages:
            return
        target = self.pages[name]
        peer = next((page for page in self.pages.values()
                     if page.id != name and target.exclusive_group
                     and page.exclusive_group == target.exclusive_group
                     and self.statuses.get(page.id, {}).get('managed')
                     and self.statuses.get(page.id, {}).get('state') in ('running', 'starting')), None)
        self.busy = True
        self.selection_error = ''
        self.revision += 1
        self._render()
        try:
            if peer is not None and not self.attach_only:
                self.confirm_message.set_text(f'切换将关闭当前{peer.label}服务，并执行原有停止流程。继续切换到{target.label}？')
                if not await self.confirm_dialog:
                    return
            self.selection_error = f'正在打开{target.label}，等待服务完成停止与启动…'
            self._render()
            states = await self.coordinator.activate(name)
            self.statuses = states
            self.status_error = False
            self.selection_error = ''
            # Only discard the old controller frame after coordinated stop succeeds.
            if target.exclusive_group:
                for page in self.pages.values():
                    state = states.get(page.id, {})
                    if (page.id != name and page.exclusive_group == target.exclusive_group
                            and not state.get('reachable')
                            and state.get('state') in ('standby', 'stopped', 'error', 'unconfigured')):
                        self._remove_frame(page.id)
            self.active = name
            await self._set_hash(name)
        except Exception as exc:
            self.selection_error = str(exc) or '无法切换页面，请重试。'
        finally:
            self.busy = False
            self._render()

    async def refresh(self):
        if self.busy or not self.active or not self._ready(self.active):
            return
        name = self.active
        self.busy = True
        self._render()
        try:
            self.confirm_message.set_text('重新加载会中断当前模块的浏览器连接，控制或遥操可能因此停止。继续重新加载？')
            if await self.confirm_dialog:
                self._remove_frame(name)
        finally:
            self.busy = False
            self._render()

    def _ready(self, name):
        return not self.status_error and service_ready(self.statuses.get(name, {}))

    def _remove_frame(self, name):
        frame = self.frames.pop(name, None)
        if frame is not None:
            frame.element.delete()

    def _sync_frames(self):
        if self.active and self._ready(self.active):
            page = self.pages[self.active]
            generation = self.statuses[self.active].get('generation', 0)
            frame = self.frames.get(self.active)
            if frame is not None and frame.generation != generation:
                self._remove_frame(self.active)
            if self.active not in self.frames:
                with self.views:
                    element = self.ui.element('iframe').props(
                        f'src={page.path or f"/apps/{page.id}/"} title="{page.label} · OpenArm" allowfullscreen'
                    ).classes('module-frame')
                    self.identify(element, f'frame-{page.id}')
                self.frames[self.active] = FrameView(element, generation)
        for name, frame in self.frames.items():
            # NiceGUI visibility uses display:none rather than removing the node.
            frame.element.set_visibility(name == self.active and self._ready(name))
        self.waiting.set_visibility(bool(self.active) and not self._ready(self.active))

    def _render(self):
        for name, page in self.pages.items():
            state = self.statuses.get(name, {})
            ready = self._ready(name)
            label = '状态未知' if self.status_error or not state else STATUS_LABELS.get(state.get('state'), '服务不可达')
            if ready:
                label = '服务可达'
            self.badges[name].set_text(label)
            self.badges[name].classes(add='online' if ready else '', remove='' if ready else 'online')
            self.nav[name].classes(add='active' if name == self.active else '', remove='' if name == self.active else 'active')
            self.nav[name].set_enabled(not self.busy)
            self.cards[name].set_enabled(not self.busy)
        self.home.set_enabled(not self.busy)
        self.mode_button.set_enabled(not self.busy)
        self.overview_item.set_enabled(not self.busy)
        # Stop is outside the iframe/inert region and remains reachable during
        # selection, loading and status failures. This is not a hardware E-stop.
        self.stop_button.set_enabled(not self.attach_only and not self.stopping)
        connection = self.connection_state
        label = {'ready': 'CAN 就绪', 'waiting': 'CAN 连接中', 'busy': 'CAN 控制占用',
                 'blocked': 'CAN 需检查', 'unconfigured': 'CAN 未检查', 'unmonitored': 'CAN 待检查',
                 'simulation': '仿真', 'readonly': '只读模式'}.get(connection.get('state'), '连接待检查')
        self.connection_button.set_text(label)
        self.connection_button.props('data-state='+connection.get('state', 'unknown'))
        self.connection_detail.set_text(connection.get('message', '连接状态尚未读取') +
                                       ('\n'+connection['error'] if connection.get('error') else ''))
        self.overview_panel.set_visibility(self.active is None)
        self.views.set_visibility(self.active is not None)
        self.views.props(f'aria-busy={str(self.busy).lower()}')
        if self.busy:
            self.views.props('inert')
        else:
            self.views.props(remove='inert')
        self.actions.set_visibility(self.active is not None)
        message = self.selection_error
        if self.active:
            page = self.pages[self.active]
            state = self.statuses.get(self.active, {})
            ready = self._ready(self.active)
            self.current_label.set_text(page.label)
            self.current_status.set_text('状态未知' if self.status_error else '服务可达' if ready else STATUS_LABELS.get(state.get('state'), '状态未知'))
            self.reload.set_enabled(ready and not self.busy)
            self.external.props(f'href={page.path or f"/apps/{page.id}/"} aria-disabled={str(not ready or self.busy).lower()} tabindex={-1 if not ready or self.busy else 0}')
            self.external.classes(add='disabled-link' if not ready or self.busy else '', remove='' if not ready or self.busy else 'disabled-link')
            if not message:
                if self.status_error:
                    message = '暂时无法读取服务状态，请检查工作站连接或 SSH 转发。'
                elif state.get('error'):
                    message = state['error']
                elif not ready:
                    message = {
                        'starting': f'正在启动{page.label}，就绪后自动显示。',
                        'standby': f'{page.label}处于待切换状态，重新选择该模块以打开。',
                        'unconfigured': f'{page.label}尚未配置启动命令。',
                        'stopped': f'{page.label}已停止，重新选择该模块可重试。',
                    }.get(state.get('state'), f'{page.label}暂时不可达，请检查模块连接。')
            self.waiting_title.set_text(('正在启动' if state.get('state') == 'starting' else '等待') + page.label)
            self.footer.set_text('控制与遥操共享电机连接 · 切换模式会执行原有停止流程' if page.exclusive_group
                                 else '数采页显示录制状态与历史数据 · 当前设备连接保持不变' if page.id == 'collection'
                                 else ('主臂控制姿态与夹爪 · 标定只采样求解 · 保存不等于启用' if self.config.teleop_calibration_profile
                                       else '镜头内参不控制机械臂 · 保存不等于启用') if page.id == 'camera_calibration'
                                 else '模型执行与归位由操作员启动 · 停止不会触发自动返程' if page.id == 'policy'
                                 else '切换到感知页面会保留当前控制或遥操连接')
        else:
            self.current_label.set_text('概览')
            self.reload.set_enabled(False)
            self.external.props('aria-disabled=true tabindex=-1')
            self.external.classes(add='disabled-link')
            self.footer.set_text(f'{len(self.pages)} 个模块 · 同一个工作空间')
        self.notice.set_text(message)
        self.notice.set_visibility(bool(message))
        self._sync_frames()


async def build_dashboard(config, coordinator, *, attach_only=False):
    """Build a NiceGUI root page. Hardware ownership stays in ``coordinator``."""
    dashboard = Dashboard(config, coordinator, attach_only=attach_only)
    dashboard.start()
    return dashboard
