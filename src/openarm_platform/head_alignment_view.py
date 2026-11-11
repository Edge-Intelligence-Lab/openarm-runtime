"""Head-camera-owned calibration workspace, never a collection-page redirect."""
class HeadAlignmentView:
    def __init__(self,workspace):
        from nicegui import ui
        self.ui=ui;self.coordinator=workspace.alignment_coordinator
        self.read_only=getattr(workspace,'attach_only',False)
        self.client=ui.context.client;self.generation=None;self.mode_lock=lambda active:None
        self.busy=False
        with ui.row().classes('w-full items-center justify-between'):
            with ui.column().classes('gap-1'):
                ui.label('机器人对齐 · 主臂控制').classes('text-lg font-semibold')
                ui.label('本页独立完成摆姿态、夹爪控制与标定，不录制遥操示教数据。').classes('text-sm text-grey-7')
            with ui.row().classes('items-center'):
                self.start=ui.button('进入标定控制',on_click=self.confirm_start).props('no-caps')
                self.close=ui.button('关闭标定控制',on_click=self.confirm_close).props('outline no-caps')
        self.message=ui.label('进入后再由你手动开始跟随；不会因打开页面而使能电机。').props('role=status')
        self.frame=ui.element('iframe').props('src="about:blank" title="头部相机标定工作区"').style(
            'width:100%;height:calc(100dvh - 180px);min-height:580px;border:0;border-radius:10px')
        self.frame.set_visibility(False)
        ui.timer(.7,self.refresh)

    def confirm_start(self):
        ui=self.ui
        with ui.dialog() as dialog,ui.card():
            ui.label('切换到头部相机标定').classes('text-lg font-semibold')
            ui.label('将正常停止现有遥操/数采控制服务。请先结束录制并支撑从臂；数据与标定记录都会保留。')
            ui.label('标定控制不会自动开始跟随或使能电机。').classes('text-sm text-grey-7')
            with ui.row():
                ui.button('取消',on_click=dialog.close).props('flat no-caps')
                async def enter():
                    dialog.close();await self.call('activate')
                ui.button('确认切换到标定',on_click=enter).props('no-caps')
        dialog.open()

    def confirm_close(self):
        ui=self.ui
        with ui.dialog() as dialog,ui.card():
            ui.label('停止并关闭标定控制').classes('text-lg font-semibold')
            ui.label('这会停止跟随并失能。请先支撑机械臂；原始标定记录保留。')
            ui.label('建议先完成求解；关闭后本组不能继续采样。').classes('text-sm text-grey-7')
            with ui.row():
                ui.button('继续标定',on_click=dialog.close).props('flat no-caps')
                async def leave():
                    dialog.close();await self.call('deactivate')
                ui.button('已支撑，停止并关闭',on_click=leave,color='negative').props('no-caps')
        dialog.open()

    async def call(self,operation):
        if self.busy:return
        self.busy=True;self.start.set_enabled(False);self.close.set_enabled(False)
        self.message.set_text('正在切换控制服务，请稍候…')
        try:
            await getattr(self.coordinator,operation)('head_alignment')
            self.message.set_text('正在打开标定控制，就绪后会显示完整工作区；跟随仍需手动开始。' if operation=='activate' else '标定控制已关闭；记录保留。')
        except Exception as exc:self.message.set_text(str(exc))
        finally:
            self.busy=False;await self.refresh()

    async def refresh(self):
        if not self.client.has_socket_connection:return
        try:states=await self.coordinator.status()
        except Exception as exc:
            self.message.set_text(str(exc));return
        state=states.get('head_alignment',{})
        ready=state.get('reachable') is True and state.get('state')!='error'
        starting=state.get('state')=='starting'
        self.mode_lock(ready or starting)
        show=ready or starting and self.generation is not None
        self.frame.set_visibility(show)
        key=state.get('generation',0) if ready else self.generation if starting else None
        if key!=self.generation:
            self.generation=key
            self.frame.props('src="/apps/head_alignment/"' if ready else 'src="about:blank"')
            if ready:self.message.set_text('标定控制已就绪：先开始跟随，再确认装板；每点一次采集保存一张。')
        self.start.set_enabled(not self.read_only and not self.busy and not ready and not starting)
        self.close.set_enabled(not self.read_only and not self.busy and (ready or starting))
        if state.get('error'):self.message.set_text(state['error'])
