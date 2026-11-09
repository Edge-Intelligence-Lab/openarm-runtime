"""NiceGUI calibration page; only explicit clicks borrow frames or write evidence."""
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
        ui.colors(primary='#36735a')
        ui.add_css('''
          body {background:#f6f8f5;color:#263d32}
          .calibration-workspace {max-width:1200px;width:100%;margin:auto;padding:24px;gap:18px}
          .calibration-card {width:100%;box-shadow:none!important;border:1px solid #dfe7df;border-radius:12px}
        ''')
        with ui.column().classes('calibration-workspace'):
            ui.label('头部相机标定').classes('text-2xl font-semibold')
            ui.label('只读借帧，不启动相机或运动硬件。取消不是急停；保存不等于激活。').classes('text-orange-9')
            ui.label('支持普通棋盘格 / ChArUco 内参 + 普通针孔镜头（OpenCV 五参数畸变）；不支持鱼眼或腕部标定。')
            self.status_label = ui.label('未开始').props('role=status data-testid=calibration-status')
            self.error = ui.label('').classes('text-red-8 whitespace-pre-wrap').props('role=alert data-testid=calibration-error')
            with ui.card().classes('calibration-card'):
                ui.label('1 · 参数与原尺寸预览').classes('text-lg font-medium')
                ui.label('先在外部视频配置填写 calibration_identity（相机序列号/镜头/固定安装身份）。板参数必须来自实际打印板。')
                self.board_type = ui.radio({'checkerboard':'普通黑白棋盘格', 'charuco':'ChArUco'},
                    value='checkerboard', on_change=lambda:self.board_fields()).props('inline data-testid=calibration-board-type')
                ui.label('填写黑白方格总数，不是内角点数；例如 10×7 格对应 9×6 内角点。格长 15 mm 应填 0.015 m。')
                with ui.row().classes('w-full items-center flex-wrap'):
                    self.camera = ui.input('已有视频服务中的相机 ID', value='head').classes('w-64 shrink-0').props('data-testid=calibration-camera')
                    self.nx = ui.number('横向方格数', min=4, max=40, precision=0).classes('w-40 shrink-0')
                    self.ny = ui.number('纵向方格数', min=4, max=40, precision=0).classes('w-40 shrink-0')
                    self.square = ui.number('实测格长（米）', min=0, max=.2, precision=6).classes('w-40 shrink-0')
                    self.marker = ui.number('实测 marker 长（米）', min=0, max=.2, precision=6).classes('w-48 shrink-0')
                self.dictionary_label = ui.label('ChArUco 打印板字典（必须与实际打印板一致）')
                self.dictionary = ui.radio(['DICT_4X4_50','DICT_5X5_100','DICT_6X6_250']).props('inline data-testid=calibration-dictionary')
                ui.button('开始新会话并保存采集证据', on_click=self.start).props('data-testid=calibration-start')
                ui.button('开启/暂停只读原尺寸预览', on_click=self.toggle_preview).props('outline data-testid=calibration-preview-toggle')
                self.image = ui.image('').classes('w-full').style('height:320px').props('fit=contain data-testid=calibration-image')
                self.image.set_visibility(False)
                self.image.on('error', lambda: self.error.set_text('原尺寸预览不可用：检查已有视频服务、相机 ID 和 calibration_identity；本页不会启动服务'))
                ui.label('预览仅缩放显示；求解使用无损原始像素。预览不代表曝光同步或设备安全状态。').classes('text-xs text-grey-7')
            with ui.card().classes('calibration-card'):
                ui.label('2 · 手持板内参采集与留出检查').classes('text-lg font-medium')
                ui.label('手持板覆盖图像中心/边缘、不同距离和两个方向的倾角。先指定样本用途，留出图像永不拟合 K。至少 8 张拟合 + 3 张留出，不自动删除异常值。')
                ui.label('下一帧用途')
                self.role = ui.radio({'fit':'拟合图像','validation':'留出验证图像'}, value='fit').props('inline data-testid=calibration-role')
                with ui.row():
                    ui.button('采集一帧并检测角点', on_click=self.capture).props('data-testid=calibration-capture')
                    ui.button('求解内参并检查留出图像', on_click=self.solve).props('outline data-testid=calibration-solve')
                    ui.button('取消本会话（不是急停）', on_click=self.cancel).props('flat data-testid=calibration-cancel')
                self.samples = ui.table(columns=[dict(name=k,label=l,field=k,align='left') for k,l in
                    [('index','序号'),('role','用途'),('corners','角点'),('center','中心(x,y)'),('coverage','图像覆盖'),('sharpness','清晰度指标')]],
                    rows=[],row_key='index').classes('w-full').props('flat dense data-testid=calibration-samples')
                ui.label('清晰度/位置覆盖仅作采集提示，不是自动验收。留出重投影会重新估计板位姿，不是独立物理精度验证。').classes('text-xs text-grey-7')
            with ui.card().classes('calibration-card'):
                ui.label('3 · 仅离线 head 外参').classes('text-lg font-medium')
                ui.button('实时外参采集不可用').disable()
                ui.label('缺少可信曝光时间与实测 FK 配对。不能用收帧时间冒充曝光时间。头相机须固定于基座、板刚性固定于末端；自由手持板不能解 base_from_camera。')
                ui.label('可导入最多 2 MB 的既有 head 观测 JSON。观测来源由操作者负责；导入不会证明同步或物理验证。')
                self.import_upload = ui.upload(label='导入 head 观测 JSON', auto_upload=True,
                          max_file_size=2_000_000, on_upload=self.import_head,
                           on_rejected=lambda: ui.notify('文件超过 2 MB 限制', type='negative')).props('accept=.json data-testid=calibration-import')
                self.board_warning = ui.label('普通棋盘格角点编号只在单张图中有效；未解决固定板坐标方向前，仅支持内参，不接受本会话外参导入。').classes('text-orange-9')
            with ui.card().classes('calibration-card'):
                ui.label('4 · 检查估计并确认保存').classes('text-lg font-medium')
                self.result_summary = ui.label('尚无估计').classes('whitespace-pre-wrap').props('data-testid=calibration-result-summary')
                with ui.expansion('查看完整估计、逐图残差和来源').classes('w-full'):
                    self.result = ui.code('尚无估计', language='json').classes('w-full').props('data-testid=calibration-result')
                self.confirm = ui.checkbox('我已检查结果；确认仅保存未独立验证候选，不安装或激活')
                ui.button('确认保存不可变候选', on_click=self.save).props('data-testid=calibration-save')
                ui.label('证据目录由可信外部平台配置 calibration_root 指定。所有原图、样本用途和来源保留；不覆盖旧候选。')
        self.timer = ui.timer(2, self.refresh_preview, active=False)
        ui.timer(1, self.refresh)
        self.board_fields()

    def board_fields(self):
        charuco = self.board_type.value == 'charuco'
        for field in (self.marker, self.dictionary, self.dictionary_label):
            field.set_visibility(charuco)
        self.board_warning.set_visibility(not charuco)
        self.import_upload.set_enabled(charuco)

    async def action(self, function, *args, **kwargs):
        if self.working:
            return
        self.working = True
        self.error.set_text('')
        try:
            value = await asyncio.to_thread(function, self.owner, *args, **kwargs)
            return value
        except Exception as exc:
            self.error.set_text(str(exc))
        finally:
            self.working = False
            await self.refresh()

    async def start(self):
        try:
            def count(value):
                if value is None or isinstance(value, bool) or int(value) != value:
                    raise ValueError('填写实际方格整数，不使用默认猜测')
                return int(value)
            spec = dict(type=self.board_type.value, squares_x=count(self.nx.value),
                        squares_y=count(self.ny.value), square_size_m=self.square.value)
            if self.board_type.value == 'charuco':
                spec.update(marker_size_m=self.marker.value, dictionary=self.dictionary.value)
            await self.action(self.workspace.start, self.camera.value, spec)
            self.confirm.value = False
        except (ValueError, TypeError) as exc:
            self.error.set_text(str(exc))

    async def capture(self):
        await self.action(self.workspace.capture, self.role.value)
        self.confirm.value = False

    async def solve(self):
        await self.action(self.workspace.solve)
        self.confirm.value = False

    async def cancel(self):
        # Cancel must not wait for the running worker; its epoch rejects late results.
        try:
            await asyncio.to_thread(self.workspace.cancel, self.owner)
            self.previewing = False
            self.timer.deactivate()
            await self.refresh()
        except ValueError as exc:
            self.error.set_text(str(exc))

    async def import_head(self, event):
        # Browser-side max_file_size is not a security boundary. Bound reads too.
        raw = bytearray()
        async for chunk in event.file.iterate(chunk_size=64*1024):
            raw.extend(chunk)
            if len(raw) > 2_000_000:
                self.error.set_text('文件超过 2 MB 限制')
                return
        await self.action(self.workspace.import_head, bytes(raw))
        self.confirm.value = False

    async def save(self):
        path = await self.action(self.workspace.save, confirmed=self.confirm.value)
        if path:
            self.ui.notify('已保存未验证候选：'+path, type='positive')

    async def toggle_preview(self):
        self.previewing = not self.previewing
        self.image.set_visibility(self.previewing)
        self.timer.activate() if self.previewing else self.timer.deactivate()
        if self.previewing:
            await self.refresh_preview()

    async def refresh_preview(self):
        name = self.camera.value
        if isinstance(name, str) and re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]*', name):
            self.image.set_source(f'/apps/vision/calibration/frame/{name}.png?t={time.monotonic_ns()}')
        else:
            self.error.set_text('无效相机 ID')

    async def refresh(self):
        status = await asyncio.to_thread(self.workspace.status, self.owner)
        for field in (self.camera, self.board_type, self.nx, self.ny, self.square, self.marker, self.dictionary):
            field.set_enabled(status['state'] not in ('collecting', 'estimated') and not status['busy'])
        self.status_label.set_text(f"状态：{status['state']} · {'本浏览器所有' if status['mine'] else '无写入所有权'} · {'处理中' if status['busy'] else '空闲'} · 会话 {status['session_id'] or '—'}")
        # Other browsers see only session state, not another operator's evidence.
        if status['mine']:
            self.samples.rows = [dict(index=i+1, role=s['role'], corners=len(s['detection']['corner_ids']),
                center=', '.join(f'{v:.2f}' for v in s['detection']['center_normalized']),
                coverage=f"{s['detection']['coverage_fraction']*100:.1f}%", sharpness=round(s['detection']['sharpness'],1))
                for i,s in enumerate(status['samples'])]
            self.samples.update()
            self.result.set_content(json.dumps(dict(intrinsics=status['result'], head_extrinsics=status['head']), ensure_ascii=False, indent=2))
            value = status['result']
            text = '内参尚未求解。'
            if value:
                text = (f"拟合 RMS {value['fit_rms_px']:.3f} px · 留出 RMS {value['validation_rms_px']:.3f} px\n"
                        + '\n'.join(value.get('quality_issues', [])))
            if status['head']:
                text += '\n已导入离线 head 外参；观测来源由操作者提供，与网页内参未自动绑定。'
            self.result_summary.set_text(text+'\n估计，未独立验证；无自动安装。')
        else:
            self.samples.rows = []
            self.samples.update()
            self.result.set_content('当前浏览器没有标定会话')
            self.result_summary.set_text('当前浏览器没有标定会话')


def build_camera_calibration_view(workspace):
    return CalibrationView(workspace)
