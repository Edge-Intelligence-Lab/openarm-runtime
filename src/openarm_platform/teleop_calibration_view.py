"""Native Viser calibration workspace, sharing the teleoperator's client owner.

Camera decoding and disk thumbnails run outside control and GUI refresh loops.
No browser can supply file paths, feedback, image timestamps or motion targets.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading
import time
import html


class TeleopCalibrationView:
    def __init__(self,server,session,calibration,*,start_preview=True,
                 stream_url='/apps/vision/stream/head.mjpg'):
        import numpy as np
        self.server,self.session,self.calibration=server,session,calibration
        self.stream_url=stream_url
        self.feedback='用主臂完成装板，画面与采样状态会在这里更新。'
        self.camera_error='正在连接头部相机';self.camera_good=False
        self.thumbnail_error=None
        self.preview_rgb=None;self.preview_at=None;self.preview_lock=threading.Lock()
        self.stop_event=threading.Event();self.worker=None
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='teleop-calibration-ui')
        self.pending=None;self.record_count=-1;self.notice_key=None
        self.last_phase=None
        from importlib.resources import files
        css=files('openarm_platform').joinpath('static/teleop-calibration.css').read_text()
        server.gui.add_html('<span class="teleop-calibration-skin" hidden></span><style>'+css+'</style>',order=-10000)
        self.brand=(54,115,90)
        server.gui.configure_theme(control_width='large',dark_mode=False,
            show_logo=False,show_share_button=False,brand_color=self.brand)
        server.gui.main_panel.dock_right()
        with server.gui.add_folder('头部相机标定',order=0,expand_by_default=True):
            self.stage=server.gui.add_html('')
            self.camera_image=server.gui.add_html(self._stream_html())
            self.camera_status=server.gui.add_html('')
            zoom=server.gui.add_button('放大查看画面',color='gray',icon='maximize',hint='放大预览；不改变保存原图尺寸')
            self.mount=server.gui.add_checkbox('板已夹紧，确认后保持当前夹爪目标',False,
                hint='采样时主臂夹爪不再跟随；停止跟随后解除保持，重新装板请先停止')
            self.begin=server.gui.add_button('确认装板并保持夹爪 · 准备第1张',color=self.brand,disabled=True)
            self.instruction=server.gui.add_html('')
            self.progress=server.gui.add_progress_bar(0.,color=self.brand)
            self.capture=server.gui.add_button('采集当前姿态',color=self.brand,disabled=True,icon='camera')
            self.stop=server.gui.add_button('停止跟随 · 失能',color='red',icon='player-stop',disabled=True)
            self.notice=server.gui.add_markdown(self.feedback)
            self.solve=server.gui.add_button('检查并生成标定候选',color=self.brand,visible=False,disabled=True)
            self.result=server.gui.add_markdown('',visible=False)
            self.restart=server.gui.add_button('同次装夹重新采一组',color='gray',visible=False)
            with server.gui.add_folder('已采样姿态',expand_by_default=False):
                self.record_image=server.gui.add_image(np.full((100,400,3),240,dtype=np.uint8),
                    label='本组采样 · 预览副本',format='jpeg',visible=False)
                self.records=server.gui.add_markdown('尚未采样。拒绝的尝试不会占用拟合或检查名额。')
            with server.gui.add_folder('会话与技术信息',expand_by_default=False):
                self.details=server.gui.add_markdown('未建立采样组。原始图片、时序和关节证据保存在工作站。')
        zoom.on_click(self._zoom)
        self.begin.on_click(self._begin)
        self.restart.on_click(self._begin)
        self.capture.on_click(lambda e:self._submit(e,calibration.capture,'正在检查静止区间并保存原图…'))
        self.solve.on_click(lambda e:self._submit(e,calibration.solve,'正在求解并执行独立检查…'))
        self.stop.on_click(lambda e:session.stop())
        if start_preview:
            self.worker=threading.Thread(target=self._preview,name='head-calibration-preview',daemon=True)
            self.worker.start()

    def _begin(self,event):
        if event.client is None:return
        if not self.mount.value:
            self.feedback='请先用主臂夹紧标定板，再确认装夹固定。';return
        if self.calibration.status()['group']:
            with self.server.gui.add_modal('开始新的一组采样') as modal:
                self.server.gui.add_markdown('现有原图和结果会保留，新组从第1张开始，夹爪继续保持。若要重新装板，请先停止跟随，再用主臂装板并重新确认。')
                yes=self.server.gui.add_button('保留旧记录，开始新的一组',color=self.brand)
                cancel=self.server.gui.add_button('继续当前组')
                yes.on_click(lambda e:(modal.close(),self._submit(e,
                    lambda owner:self.calibration.begin(owner,mounted=True),'正在建立采样组…')))
                cancel.on_click(lambda e:modal.close())
        else:
            self._submit(event,lambda owner:self.calibration.begin(owner,mounted=True),'正在建立采样组…')

    def _submit(self,event,action,message):
        if event.client is None:return
        if self.pending is not None and not self.pending.done():return
        owner=event.client.client_id
        self.feedback=message
        def work():
            try:
                value=action(owner)
                if isinstance(value,int):
                    self.feedback=f'第{value}张已保存。换一个位置或倾角，再保持静止。'
                elif isinstance(value,dict):
                    self.feedback='拟合与独立检查通过。候选已保存，尚未应用到机器人。'
                else:self.feedback='当前夹爪目标已保持。摆好姿态并停稳后，点一次采集保存一张；不会自动连拍。'
            except Exception as exc:
                self.feedback=self.friendly_error(exc)
        self.pending=self.executor.submit(work)

    @staticmethod
    def friendly_error(exc):
        text=str(exc)
        if 'eight non-collinear' in text:
            return '未识别到足够角点。请让整块板进入画面，避免遮挡、反光和过大的倾角；本次不计数。'
        if 'reprojection RMS exceeds' in text:
            return '板的识别精度不足。请减小反光或运动模糊、稍微靠近相机后重试；本次不计数。'
        if 'Indexed board pose failed' in text:
            return '这次没有得到可信的板姿态。请改变倾角并让整块板完整入镜后重试；本次不计数。'
        if 'moved during stationary' in text:
            return '拍摄期间机械臂仍在动。请停稳约一秒后重试；本次不计数。'
        if 'bracket' in text or 'feedback' in text.lower():
            return '本次缺少完整的静止反馈区间。保持姿态，确认遥操连接后重试；本次不计数。'
        if 'inconsistent stationary' in text:
            return '独立一致性检查未通过，未生成可用候选。原图和诊断已保留，请查看技术信息后重新采样。'
        if 'excite two axes' in text:
            return '姿态变化不足：需要绕至少两个方向倾斜标定板，而不只是平移或正对相机旋转。原始记录已保留。'
        return text

    def _zoom(self,event):
        if event.client is None:return
        # Client-local dialog: another operator must not acquire an extra stream.
        with event.client.gui.add_modal('头部相机 · 连续视频') as modal:
            event.client.gui.add_html(self._stream_html())
            close=event.client.gui.add_button('返回采样')
            close.on_click(lambda e:modal.close())

    def _stream_html(self):
        return ('<img src="'+html.escape(self.stream_url,quote=True)+'" '
            'alt="头部相机连续视频流" style="display:block;width:100%;height:auto;'
            'aspect-ratio:16/9;object-fit:contain;border-radius:8px" />')

    def _preview(self):
        from .camera_calibration import decode_frame
        from .calibration_source import board_fits_image
        while not self.stop_event.is_set():
            if not self.server.get_clients():
                self.stop_event.wait(.5);continue
            try:
                camera=self.calibration.camera
                raw,metadata=camera.fetch(camera.port,camera.data['camera_name'])
                if metadata.get('simulation') is not False:
                    raise ValueError('相机来源与实物遥操不匹配')
                rgb=decode_frame(raw,metadata)
                with self.preview_lock:
                    self.preview_rgb=rgb;self.preview_at=time.monotonic()
                # This slower worker qualifies capture readiness only. The browser
                # receives continuous MJPEG independently of PNG/board parsing.
                try:
                    _,observation,_=camera.parse(raw,metadata)
                    if not board_fits_image(observation,camera.intrinsics):
                        raise ValueError('板边缘超出画面，请调整位置')
                    self.camera_good=True
                    self.camera_error=f'标定板可见 · {len(observation["corner_ids"])}个角点 · 重投影{observation["reprojection_rms_px"]:.2f}px'
                except Exception as exc:
                    self.camera_good=False;self.camera_error=self.friendly_error(exc)
            except Exception as exc:
                self.camera_good=False;self.camera_error='相机未就绪 · '+str(exc)
            try:
                self._thumbnails();self.thumbnail_error=None
            except Exception:
                self.thumbnail_error='缩略图暂不可用；原始记录未修改，可在会话目录查看。'
            self.stop_event.wait(.8)

    def _thumbnails(self):
        import cv2
        import numpy as np
        state=self.calibration.status()
        key=(state['group'],state['count'])
        if key==self.record_count:return
        self.record_count=key
        if not state['history']:
            self.record_image.visible=False;return
        tiles=np.full((4*84,4*112,3),235,dtype=np.uint8)
        for item in state['history']:
            path=Path(state['directory'])/item['artifact']/'image.png'
            image=cv2.imread(str(path))
            if image is None:continue
            image=cv2.resize(image,(108,60))[:,:,::-1].copy()
            row,col=divmod(item['index']-1,4);x,y=col*112,row*84
            tiles[y:y+60,x:x+108]=image
            label=f'{item["index"]:02d} '+('FIT' if item['role']=='fit' else 'CHECK')
            cv2.putText(tiles,label,(x+3,y+76),cv2.FONT_HERSHEY_SIMPLEX,.32,(50,65,75),1)
        self.record_image.image=tiles;self.record_image.visible=True

    def refresh(self):
        state=self.calibration.status();count=state['count'];phase=state['phase']
        if phase=='reset' and self.last_phase!='reset':self.mount.value=False
        self.last_phase=phase
        pending=self.pending is not None and not self.pending.done()
        preview_fresh=self.preview_at is not None and time.monotonic()-self.preview_at<2
        camera_good=self.camera_good and preview_fresh
        current=dict(connect=0,mount=1,reset=1,capture=2,review=3,result=3)[phase]
        steps=''.join(f'<span class="cal-step {"current" if i==current else "done" if i<current else ""}"><b>{i+1}</b>{label}</span>'
            for i,label in enumerate(('连接','装板','采样','检查')))
        self.stage.content='<div class="cal-stage"><small>相机 / 机器人坐标</small><h2>相机对齐标定</h2><div class="cal-steps">'+steps+'</div></div>'
        camera_message=self.camera_error if preview_fresh else '相机画面已过期，请检查连接后再采样。'
        self.camera_status.content=f'<div class="cal-camera-state" data-ready="{str(camera_good).lower()}">{html.escape(camera_message)}</div>'
        self.mount.visible=phase in ('connect','mount','reset')
        self.begin.visible=phase in ('connect','mount','reset')
        self.restart.visible=bool(state['group']) and phase not in ('connect','mount','reset')
        self.restart.disabled=pending or not state['ready']
        self.begin.disabled=not state['ready'] or not self.mount.value or pending
        self.begin.label='确认装板并保持夹爪 · 新建一组' if state['group'] else '确认装板并保持夹爪 · 准备第1张'
        self.progress.value=count/16
        self.progress.animated=pending
        self.capture.visible=phase=='capture'
        self.capture.label=f'采集拟合姿态 {count+1}/12' if count<12 else f'采集独立检查 {count-11}/4'
        if pending:self.capture.label='正在检查与保存…'
        self.capture.disabled=not (state['ready'] and state['stationary'] and camera_good) or pending
        self.stop.disabled=not self.session.busy
        self.solve.visible=count==16
        self.solve.disabled=pending or state['solve_attempted']
        if phase=='connect':instruction='### 第一步：开始左臂跟随\n上方按钮只连接跟随，不会开始采样；用主臂完成装板与夹紧。'
        elif phase=='mount':instruction='### 第二步：确认装板\n确认后保持当前夹爪目标，主臂继续控制机械臂姿态；再逐张点击采集，不会自动连拍。'
        elif phase=='reset':instruction='### 采样组已暂停，不是跳页\n'+str(state['error'])+'。旧记录保留；如需重新装板，请先停止跟随。'
        elif count<12:
            reason=('正在检查与保存' if pending else '等待左臂跟随反馈' if not state['ready'] else
                '等待板完整入镜' if not camera_good else '等待机械臂停稳' if not state['stationary'] else '可以采样')
            instruction=f'### 拟合姿态 · 已保存{count}/12张\n**{reason}**。摆好并停稳后点一次采集保存一张。夹爪保持，交替改变位置、距离和两个方向的倾角。'
        elif count<16:
            instruction=f'### 独立检查 · 已保存{count-12}/4张\n这4张不参与拟合。换新的可见姿态，停稳后点一次采集保存一张。'
        else:instruction='### 16个姿态已齐\n可以先停止跟随，再检查并生成候选。检查不通过时不会启用任何结果。'
        title,body=instruction.removeprefix('### ').split('\n',1)
        self.instruction.content=f'<section class="cal-instruction"><h3>{html.escape(title)}</h3><p>{html.escape(body.replace("**",""))}</p></section>'
        message=state['error'] or self.feedback
        if message!=self.notice_key:
            self.notice_key=message;self.notice.content=message
        self.result.visible=state['result'] is not None or state['solve_error'] is not None
        if state['result']:
            metrics=state['result']['metrics'];fit,check=metrics['fit'],metrics['validation']
            import math
            self.result.content=('### 检查通过 · 候选已保存\n**尚未应用到机器人**\n\n'
                f'拟合：{fit["rms_translation_m"]*1000:.2f} mm / {math.degrees(fit["rms_rotation_rad"]):.2f}°\n\n'
                f'独立检查：{check["rms_translation_m"]*1000:.2f} mm / {math.degrees(check["rms_rotation_rad"]):.2f}°')
        elif state['solve_error']:self.result.content='### 检查未通过\n'+self.friendly_error(ValueError(state['solve_error']))
        self.records.content=('| 姿态 | 用途 | 重投影 |\n|---|---|---|\n'+''.join(
            f'| {h["index"]:02d} | {"拟合" if h["role"]=="fit" else "独立检查"} | {h["rms"]:.2f}px |\n' for h in state['history'])) if count else '尚未采样。拒绝的尝试不占采样名额。'
        self.details.content=(f'会话：`{state["group"] or "未开始"}`\n\n'
            f'拟合 {min(count,12)}/12 · 独立检查 {max(0,count-12)}/4 · 拒绝尝试 {state["rejected"]}\n\n'
            f'原始证据：`{state["directory"] or "未建立"}`\n\n'+(state['solve_error'] or '')+
            '\n\n'+(self.thumbnail_error or '')+
            '\n\n'+('夹爪目标保持中；停止跟随后解除。' if state.get('gripper_held') else '夹爪未保持，装板阶段由主臂控制。')+
            '\n\n主臂控制机械臂姿态。视频不会自动存成标定图，每点一次采集只保存一张合格原图；标定候选不会自动启用。')

    def close(self):
        self.stop_event.set()
        if self.worker:self.worker.join(4)
        self.executor.shutdown(wait=True,cancel_futures=True)
