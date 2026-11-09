"""Capture-first calibration workspace. Device ownership remains external."""
import asyncio
import json
import re
import time


class CalibrationView:
    def __init__(self, workspace):
        from nicegui import ui
        self.ui, self.workspace = ui, workspace
        self.owner = ui.context.client.id
        self.working = False
        self.previewing = False
        self.active_action = None
        self.last_status = None
        self.messages = {}
        self.phase = 'fit'
        self._rendered_evidence = None
        ui.colors(primary='#36735a')
        ui.add_css('''
          body {background:#f6f8f5;color:#263d32}
          .cal-workspace {max-width:1160px;width:100%;margin:auto;padding:24px;gap:20px}
          .cal-header {width:100%;justify-content:space-between;align-items:center}
          .cal-layout {display:grid;grid-template-columns:minmax(0,1fr) 300px;width:100%;gap:20px;align-items:start}
          .cal-panel {width:100%;background:white;border-radius:12px;padding:20px;gap:14px}
          .cal-video {width:100%;aspect-ratio:16/9;background:#1f2925;border-radius:8px;position:relative;overflow:hidden;display:flex;align-items:center;justify-content:center}
          .cal-video img {width:100%;height:100%;object-fit:contain;position:absolute;inset:0}
          .cal-caption {color:#65756c;font-size:.875rem}
          .cal-message {width:100%;border-radius:6px;padding:10px 12px;font-size:.9rem;white-space:pre-wrap}
          .cal-message:empty {display:none}
          .cal-message[data-tone=error] {background:#fff0ed;color:#a62c22}
          .cal-message[data-tone=success] {background:#edf5ee;color:#28583a}
          .cal-message[data-tone=info] {background:#f0f3f1;color:#43574b}
          .cal-main-button {min-height:46px;font-size:1rem}
          .cal-count {font-size:1.5rem;font-weight:600;font-variant-numeric:tabular-nums}
          .cal-settings {width:100%;border-top:1px solid #e6ece6}
          .cal-settings .q-expansion-item__content {padding:12px 0}
          .cal-field {width:100%}
          .cal-end {color:#65756c;font-size:.8rem}
          @media(max-width:800px) {.cal-layout{grid-template-columns:1fr}.cal-workspace{padding:16px;gap:16px}.cal-panel{padding:16px}}
        ''')
        with ui.column().classes('cal-workspace'):
            with ui.row().classes('cal-header'):
                with ui.column().classes('gap-1'):
                    ui.label('头部相机标定').classes('text-xl font-semibold')
                    ui.label('拍摄棋盘，校正镜头畸变。不会移动机械臂。').classes('cal-caption')
                self.status_label = ui.label('准备开始').classes('text-sm').props('role=status aria-live=polite data-testid=calibration-status')
            with ui.element('div').classes('cal-layout'):
                with ui.column().classes('cal-panel'):
                    with ui.row().classes('w-full items-center justify-between'):
                        ui.label('相机画面').classes('font-semibold')
                        self.preview_button = ui.button('查看画面', on_click=self.toggle_preview).props('flat dense no-caps data-testid=calibration-preview-toggle')
                    with ui.element('div').classes('cal-video'):
                        self.placeholder = ui.label('点击「查看画面」连接已有相机').classes('text-grey-4 text-sm')
                        self.image = ui.element('img').props('alt="头部相机实时画面" data-testid=calibration-image')
                        self.image.set_visibility(False)
                        self.image.on('load', self.preview_loaded)
                        self.image.on('error', self.preview_failed)
                    self.feedback('preview')
                    self.capture_hint = ui.label('让整块棋盘入镜，停稳后拍摄。').classes('cal-caption')
                    self.start_button = ui.button('开始标定', on_click=self.start).classes('cal-main-button w-full').props('no-caps data-testid=calibration-start')
                    self.feedback('start')
                    self.session_notice = ui.label('').classes('cal-caption').props('role=status aria-live=polite')
                    with ui.column().classes('w-full gap-2') as self.capture_panel:
                        self.role = ui.radio({'fit':'标定照片', 'validation':'检查照片'}, value='fit',
                                             on_change=self.role_changed).props('inline data-testid=calibration-role')
                        self.capture_button = ui.button('拍摄一张', on_click=self.capture).classes('cal-main-button w-full').props('no-caps data-testid=calibration-capture')
                        self.feedback('capture')
                    with ui.expansion('已拍照片 · 0 张').classes('w-full') as self.samples_expansion:
                        self.samples = ui.table(columns=[dict(name=k,label=l,field=k,align='left') for k,l in
                            [('index','照片'),('role','用途'),('corners','角点'),('center','画面位置'),('coverage','覆盖率')]],
                            rows=[],row_key='index').classes('w-full').props('flat dense data-testid=calibration-samples')
                        ui.label('只有检测成功的照片才计入进度；原图均保留。').classes('cal-caption')
                with ui.column().classes('cal-panel'):
                    self.stage_label = ui.label('准备').classes('text-lg font-semibold')
                    self.board_summary = ui.label('普通棋盘 · 10 × 7 格 · 15 mm').classes('cal-caption')
                    self.next_hint = ui.label('参数已填好。先开始，再拍摄。').classes('text-sm')
                    with ui.column().classes('w-full gap-3') as self.progress_panel:
                        with ui.row().classes('w-full justify-between items-center'):
                            ui.label('标定照片').classes('text-sm')
                            self.fit_count = ui.label('0 / 8').classes('cal-count').props('data-testid=calibration-fit-count')
                        self.fit_progress = ui.linear_progress(0,show_value=False).classes('w-full')
                        with ui.row().classes('w-full justify-between items-center'):
                            ui.label('检查照片').classes('text-sm')
                            self.validation_count = ui.label('0 / 3').classes('cal-count').props('data-testid=calibration-validation-count')
                        self.validation_progress = ui.linear_progress(0,show_value=False).classes('w-full')
                        self.phase_button = ui.button('下一步：拍检查照片', on_click=self.next_phase).props('outline no-caps data-testid=calibration-next-phase').classes('w-full')
                        self.solve_button = ui.button('计算标定结果', on_click=self.solve).props('no-caps data-testid=calibration-solve').classes('cal-main-button w-full')
                        self.solve_hint = ui.label('至少需要 8 张标定照片和 3 张检查照片。').classes('cal-caption')
                        self.feedback('solve')
                    with ui.expansion('棋盘与相机设置').classes('cal-settings') as self.settings:
                        self.board_type = ui.radio({'checkerboard':'普通棋盘', 'charuco':'ChArUco'}, value='checkerboard',
                            on_change=self.board_fields).props('inline data-testid=calibration-board-type')
                        self.camera = ui.input('相机 ID', value='head',on_change=self.settings_changed).classes('cal-field').props('data-testid=calibration-camera')
                        with ui.row().classes('w-full flex-nowrap'):
                            self.nx = ui.number('横向格数',value=10,min=4,max=40,precision=0,on_change=self.settings_changed).classes('w-1/2')
                            self.ny = ui.number('纵向格数',value=7,min=4,max=40,precision=0,on_change=self.settings_changed).classes('w-1/2')
                        self.square = ui.number('格长（米）',value=.015,min=0,max=.2,precision=6,on_change=self.settings_changed).classes('cal-field')
                        ui.label('填黑白方格总数，不是内部角点数。15 mm = 0.015 m。').classes('cal-caption')
                        self.marker = ui.number('Marker 长（米）',min=0,max=.2,precision=6).classes('cal-field')
                        self.dictionary_label = ui.label('打印板字典').classes('cal-caption')
                        self.dictionary = ui.select(['DICT_4X4_50','DICT_5X5_100','DICT_6X6_250']).classes('cal-field').props('data-testid=calibration-dictionary')
                        self.settings_hint = ui.label('开始后锁定参数。换板请先结束本次标定。').classes('cal-caption')
                    with ui.expansion('怎么拍得更准').classes('w-full'):
                        ui.label('覆盖画面中心和四角，改变远近，并向左右、上下倾斜。不要只拍同一个正面姿态。').classes('text-sm')
                        ui.label('保持棋盘平整、完整入镜，避免反光与模糊。拍摄期间不改相机焦距、分辨率或安装位置。').classes('text-sm')
                    self.cancel_button = ui.button('结束本次标定', on_click=self.ask_cancel).props('flat no-caps data-testid=calibration-cancel').classes('w-full')
                    self.feedback('cancel')
            with ui.column().classes('cal-panel') as self.result_panel:
                ui.label('标定结果').classes('text-lg font-semibold')
                self.result_summary = ui.label('').classes('whitespace-pre-wrap').props('data-testid=calibration-result-summary')
                self.quality_label = ui.label('').classes('text-sm whitespace-pre-wrap')
                ui.label('像素误差越小，图像拟合越好；它不等于真实空间精度已验证。').classes('cal-caption')
                with ui.expansion('逐图误差与完整数据').classes('w-full'):
                    self.result = ui.code('',language='json').classes('w-full').props('data-testid=calibration-result')
                self.confirm = ui.checkbox('我已检查结果，保存为候选（不会自动启用）',on_change=self.confirm_changed)
                self.save_button = ui.button('保存标定候选',on_click=self.save).props('no-caps data-testid=calibration-save')
                self.feedback('save')
            with ui.expansion('高级：离线外参导入与记录').classes('w-full'):
                self.board_warning = ui.label('当前普通棋盘只做内参。外参还缺少固定板方向与可靠的图像—关节反馈同步。').classes('text-sm')
                ui.label('外参导入仅用于已有的 head 观测，不证明同步，也不与本次内参自动绑定。').classes('cal-caption')
                self.import_upload = ui.upload(label='导入外参 JSON',auto_upload=True,max_file_size=2_000_000,
                    on_upload=self.import_head,on_rejected=lambda:self.message('import','文件超过 2 MB，请检查文件。','error')).props('accept=.json data-testid=calibration-import')
                self.feedback('import')
                self.session_details = ui.label('').classes('cal-caption whitespace-pre-wrap')
                self.diagnostic = ui.label('').classes('cal-caption whitespace-pre-wrap').props('data-testid=calibration-diagnostic')
                ui.label('取样使用独立原尺寸 PNG；预览是 JPEG 视频。原图、元数据和结果保存在外部证据目录。取消与关闭页面不是急停。').classes('cal-caption')
            ui.label('仅标定头部相机内参 · 不控制机器人 · 保存不会自动启用').classes('cal-end')
        self.capture_panel.set_visibility(False)
        self.progress_panel.set_visibility(False)
        self.result_panel.set_visibility(False)
        self.samples_expansion.set_visibility(False)
        self.cancel_button.set_visibility(False)
        self.solve_button.disable()
        self.save_button.disable()
        self.board_fields()
        self.camera.on_value_change(self.refresh_preview)
        ui.timer(1,self.refresh)
        ui.timer(.05,self.refresh,once=True)

    def feedback(self, name):
        self.messages[name] = self.ui.label('').classes('cal-message').props(
            f'role=status aria-live=polite aria-atomic=true data-testid=calibration-{name}-feedback')

    def message(self, name, text, tone='info'):
        self.messages[name].set_text(text)
        self.messages[name].props(f'data-tone={tone} role={"alert" if tone == "error" else "status"}')

    def settings_changed(self):
        if not hasattr(self,'board_summary') or not hasattr(self,'square'):
            return
        kind = '普通棋盘' if self.board_type.value == 'checkerboard' else 'ChArUco'
        length = f'{self.square.value*1000:g}' if isinstance(self.square.value,(int,float)) else '—'
        self.board_summary.set_text(f'{kind} · {self.nx.value or "—"} × {self.ny.value or "—"} 格 · {length} mm')

    def board_fields(self):
        charuco = self.board_type.value == 'charuco'
        for field in (self.marker,self.dictionary,self.dictionary_label):
            field.set_visibility(charuco)
        self.board_warning.set_visibility(not charuco)
        self.import_upload.set_enabled(charuco and bool(self.last_status and self.last_status['mine']
            and self.last_status['state'] in ('collecting','estimated')) and not self.working)
        self.settings_changed()

    def role_changed(self):
        self.phase = self.role.value
        self.message('capture','')
        self.capture_hint.set_text('检查照片不参与拟合；换几个位置和倾角再拍。' if self.phase == 'validation'
                                   else '让整块棋盘入镜，停稳后拍摄。多换位置、远近和倾角。')

    async def next_phase(self):
        self.role.value = 'validation'
        self.role_changed()
        await self.refresh()

    def preview_loaded(self):
        if self.previewing:
            self.placeholder.set_visibility(False)
            self.message('preview','')

    def preview_failed(self):
        if self.previewing:
            self.placeholder.set_text('相机画面已断开')
            self.placeholder.set_visibility(True)
            self.message('preview','无法显示画面。点击「重连画面」重试；检查已有相机服务是否运行。','error')
            self.preview_button.set_text('重连画面')
            self.previewing = False
            self.image.set_visibility(False)
            self.image.props(remove='src')

    async def toggle_preview(self):
        self.previewing = not self.previewing
        self.image.set_visibility(self.previewing)
        self.preview_button.set_text('暂停画面' if self.previewing else '查看画面')
        self.placeholder.set_visibility(True)
        self.placeholder.set_text('正在连接相机…' if self.previewing else '画面已暂停；不会停止相机服务')
        self.message('preview','')
        if self.previewing:
            await self.refresh_preview()
        else:
            self.image.props(remove='src')

    async def refresh_preview(self):
        if not self.previewing:
            return
        name = self.camera.value
        if isinstance(name,str) and re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]*',name):
            self.image.props(f'src=/apps/vision/stream/{name}.mjpg?subscription={time.monotonic_ns()}')
        else:
            self.preview_failed()
            self.message('preview','相机 ID 无效，请在「棋盘与相机设置」中修改。','error')

    async def action(self, name, function, *args, pending='处理中…', success='', **kwargs):
        if self.working:
            return None
        self.working, self.active_action = True, name
        self.message(name,pending)
        await self.refresh()
        try:
            value = await asyncio.to_thread(function,self.owner,*args,**kwargs)
            self.message(name,success,'success')
            return value
        except Exception as exc:
            self.diagnostic.set_text(f'{name}: {type(exc).__name__}: {exc}')
            self.message(name,self.action_error(name,exc),'error')
            return None
        finally:
            self.working, self.active_action = False, None
            await self.refresh()

    @staticmethod
    def action_error(name, exc):
        text = str(exc)
        if isinstance(exc, OSError):
            return '未能取得相机图像，本次未增加照片。请先重连画面；仍失败时检查相机服务。详细错误在「高级」中。'
        if '有效标定会话' in text:
            return '本页还未开始标定，或会话已失效。请点击「开始标定」后再拍摄。'
        if name == 'capture':
            if '内角点' in text or '不共线' in text:
                return f'这张未收录：{text}。请让整块棋盘入镜、停稳并避开反光，再拍一张。'
            if isinstance(exc, ValueError):
                return f'拍摄失败，未增加照片：{text}。调整后可重试。'
            return '拍摄未完成，本次未增加照片。请重试；详细错误在「高级」中。'
        return text or '操作失败，请重试。'

    async def start(self):
        try:
            def count(value):
                if value is None or isinstance(value,bool) or int(value) != value:
                    raise ValueError('方格数请填整数。')
                return int(value)
            spec = dict(type=self.board_type.value,squares_x=count(self.nx.value),
                        squares_y=count(self.ny.value),square_size_m=self.square.value)
            if self.board_type.value == 'charuco':
                spec.update(marker_size_m=self.marker.value,dictionary=self.dictionary.value)
        except (ValueError,TypeError,OverflowError) as exc:
            self.settings.open()
            self.message('start',str(exc),'error')
            return
        session = await self.action('start',self.workspace.start,self.camera.value,spec,
            pending='正在准备标定…')
        if session:
            self.confirm.value = False
            self.phase = self.role.value = 'fit'
            for name in ('capture','solve','save','cancel','import'):
                self.message(name,'')
            self.settings.close()
            if not self.previewing:
                await self.toggle_preview()
        else:
            self.settings.open()

    async def capture(self):
        self.confirm.value = False
        sample = await self.action('capture',self.workspace.capture,self.role.value,pending='正在取图并检测棋盘…')
        if sample:
            role = '检查照片' if sample['role'] == 'validation' else '标定照片'
            self.message('capture',f'已收录一张{role} · 检测到 {len(sample["detection"]["corner_ids"])} 个角点。换个位置再拍。','success')

    async def solve(self):
        self.confirm.value = False
        result = await self.action('solve',self.workspace.solve,pending='正在计算内参与检查照片误差…',
                                  success='计算完成。请查看下方结果，再决定是否保存。')
        if result:
            self.result_panel.run_method('scrollIntoView',{'behavior':'smooth','block':'start'})

    def ask_cancel(self):
        with self.ui.dialog() as dialog, self.ui.card():
            self.ui.label('结束本次标定？').classes('text-lg font-semibold')
            self.ui.label('已拍原图会保留；当前会话不能恢复。不会停止相机或机械臂。')
            with self.ui.row():
                self.ui.button('继续拍摄',on_click=dialog.close).props('flat')
                async def finish():
                    dialog.close()
                    await self.cancel()
                self.ui.button('结束标定',on_click=finish)
        dialog.open()

    async def cancel(self):
        # Cancellation remains available during a worker; epoch rejects late results.
        try:
            await asyncio.to_thread(self.workspace.cancel,self.owner)
            self.message('cancel','本次已结束，原图已保留。可重新开始。','success')
            await self.refresh()
        except ValueError as exc:
            self.message('cancel',str(exc),'error')

    async def import_head(self, event):
        raw = bytearray()
        async for chunk in event.file.iterate(chunk_size=64*1024):
            raw.extend(chunk)
            if len(raw)>2_000_000:
                self.message('import','文件超过 2 MB，请检查文件。','error')
                return
        self.confirm.value = False
        await self.action('import',self.workspace.import_head,bytes(raw),pending='正在检查并拟合外参…',
                          success='已导入离线外参；未验证同步，未与本次内参绑定。')

    async def confirm_changed(self):
        await self.refresh()

    async def save(self):
        await self.action('save',self.workspace.save,confirmed=self.confirm.value,pending='正在核验原图并保存…',
                          success='已保存标定候选和原图记录。尚未安装或启用。')

    async def refresh(self):
        status = await asyncio.to_thread(self.workspace.status,self.owner)
        self.last_status = status
        state, mine = status['state'], status['mine']
        active = state in ('collecting','estimated')
        owned = mine and active
        busy = status['busy'] or self.working
        samples = status['samples'] if mine else []
        fit = sum(s['role']=='fit' for s in samples)
        validation = sum(s['role']=='validation' for s in samples)
        ready = fit>=8 and validation>=3
        value = status['result'] if mine else None
        head = status['head'] if mine else None
        titles = {'idle':'准备开始','collecting':'正在拍摄','estimated':'结果待检查','saved':'已保存','cancelled':'本次已结束'}
        self.status_label.set_text('其他窗口正在标定' if active and not mine else titles.get(state,state))
        self.stage_label.set_text('其他窗口正在使用' if active and not mine else
            '检查结果' if value else '拍摄检查照片' if owned and self.role.value=='validation' else
            '拍摄标定照片' if owned else '已保存' if state=='saved' and mine else '准备')
        if active and not mine:
            hint = '请回到开始标定的原窗口继续。此页不能接管或结束他人的会话；原窗口关闭后约 2 分钟释放。'
        elif owned and ready:
            hint = '数量已满足。可计算结果，也可以继续补拍不同姿态。'
        elif owned and fit>=8:
            hint = '标定照片已够。接着拍至少 3 张独立检查照片。'
        elif owned:
            hint = '先拍至少 8 张。覆盖中心、四角，并改变倾角。'
        elif mine and state=='saved':
            hint = '候选已保存，未自动启用。需要重新采集时可开始新标定。'
        elif state=='cancelled':
            hint = '原记录保留。点击开始，创建新的标定。'
        else:
            hint = '参数已填好。点击开始，再拍摄。'
        self.next_hint.set_text(hint)
        self.session_notice.set_text('其他窗口正在标定，请回到开始标定的原窗口。' if active and not mine else '')
        self.start_button.set_visibility(not owned)
        self.start_button.set_enabled(not active and not busy)
        self.start_button.set_text('重新开始标定' if state in ('saved','cancelled') else '开始标定')
        self.capture_panel.set_visibility(owned)
        self.samples_expansion.set_visibility(bool(samples))
        self.board_summary.set_visibility(not active or mine)
        self.settings.set_visibility(not active or mine)
        self.progress_panel.set_visibility(owned)
        self.cancel_button.set_visibility(owned)
        self.capture_button.set_enabled(owned and not busy)
        self.capture_button.set_text('正在检测…' if self.active_action=='capture' else '拍摄一张')
        self.capture_button.props('loading' if self.active_action=='capture' else '',remove='' if self.active_action=='capture' else 'loading')
        self.role.set_enabled(owned and not busy)
        self.phase_button.set_visibility(owned and fit>=8 and self.role.value!='validation' and not ready)
        self.phase_button.set_enabled(not busy)
        self.solve_button.set_enabled(owned and ready and not busy)
        self.solve_button.set_text('正在计算…' if self.active_action=='solve' else '计算标定结果')
        self.solve_hint.set_text('数量达到最低要求；姿态覆盖仍需检查。' if ready else
            f'还需 {max(0,8-fit)} 张标定照片、{max(0,3-validation)} 张检查照片。')
        self.fit_count.set_text(f'{fit} / 8')
        self.validation_count.set_text(f'{validation} / 3')
        self.fit_progress.value = min(fit/8,1)
        self.validation_progress.value = min(validation/3,1)
        for field in (self.camera,self.board_type,self.nx,self.ny,self.square,self.marker,self.dictionary):
            field.set_enabled(not active and not busy)
        self.settings_hint.set_text('参数已锁定。需修改时，先结束本次标定。' if active else '换棋盘时在这里修改；开始后锁定参数。')
        self.import_upload.set_enabled(owned and self.board_type.value=='charuco' and not busy)
        self.result_panel.set_visibility(bool(value or head))
        self.confirm.set_enabled(owned and bool(value or head) and not busy)
        self.save_button.set_enabled(owned and bool(value or head) and self.confirm.value is True and not busy)
        self.save_button.set_text('已保存' if mine and state=='saved' else '保存标定候选')
        self.session_details.set_text(f'会话：{status["session_id"] or "—"}' if mine else '本窗口没有采集会话。')
        # Do not rebuild the table/code every heartbeat: preserve scroll and focus.
        evidence = (status['session_id'] if mine else None,len(samples),state,bool(value),bool(head))
        if evidence != self._rendered_evidence:
            self._rendered_evidence = evidence
            self.samples.rows = [dict(index=i+1,role='检查' if s['role']=='validation' else '标定',
                corners=len(s['detection']['corner_ids']),
                center=', '.join(f'{v:.2f}' for v in s['detection']['center_normalized']),
                coverage=f'{s["detection"]["coverage_fraction"]*100:.1f}%') for i,s in enumerate(samples)]
            self.samples.update()
            self.samples_expansion.set_text(f'已拍照片 · {len(samples)} 张')
            self.result.set_content(json.dumps(dict(intrinsics=value,head_extrinsics=head),ensure_ascii=False,indent=2))
            text = f'标定误差 {value["fit_rms_px"]:.3f} px　·　检查误差 {value["validation_rms_px"]:.3f} px' if value else '仅有离线外参估计'
            self.result_summary.set_text(text)
            issues = value.get('quality_issues',[]) if value else []
            self.quality_label.set_text(('需要复查：\n'+'\n'.join(issues)) if issues else '请检查拍摄覆盖和棋盘尺寸。结果尚未经过独立物理验证。')


def build_camera_calibration_view(workspace):
    return CalibrationView(workspace)
