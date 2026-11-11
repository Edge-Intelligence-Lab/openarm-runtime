"""Physical leader → virtual follower. Calibration writes only a local JSON file."""

import math
import copy
import threading
import time

from openarm_viewer.model import load_model
from openarm_viewer.scene import check_port, create_scene
from .mapping import Mapping, SOURCE_IDS
from .transport import Reader
from .calibration import collect_stable


def serve(asset_root, left, right, calibration, port=8890, *, control_profile=None, report_root=None, collection=None,
          head_calibration_profile=None,head_calibration_root=None,head_video_port=5003):
    import numpy as np
    import viser

    mapping = Mapping(left, right, calibration)
    check_port(port)
    models = {side: load_model(asset_root, side, component=True)
              for side in ("body", "left", "right")}
    readers = {path: Reader(path) for path in (left, right)}
    lock = threading.RLock()
    teleop = None
    server = viser.ViserServer(host="127.0.0.1", port=port,
        label="头部相机标定 · 主臂控制" if head_calibration_profile else "OpenArm 遥操作")
    if server.get_port() != port:
        server.stop()
        raise RuntimeError("Requested viewer port is occupied")
    try:
        roots, visuals = create_scene(server, models)
        for side, label in (("left", "左臂"), ("right", "右臂")):
            server.scene.add_label(f"/label_{side}", text=label,
                                   position=(0, 0.32 if side == "left" else -0.32, 0.88))
        statuses = {side: server.gui.add_markdown(f"{label} · 连接中")
                    for side, label in (("left", "左臂"), ("right", "右臂"))}
        calibration_folder = server.gui.add_folder("主臂标定", expand_by_default=False)
        with calibration_folder:
            server.gui.add_markdown(
                "自然下垂 → 记录零位 → 张开／闭合夹爪 → 保存。\n\n"
                "采集时保持静止约一秒。保存软件参考；三维模型显示相对关节变化。")
            selection = server.gui.add_dropdown("标定机械臂", options=("双臂", "左臂", "右臂"), initial_value="双臂")
            progress = server.gui.add_markdown("")
        capture_buttons = []
        busy = False
        saved_data = copy.deepcopy(mapping.data)
        with calibration_folder:
            message = server.gui.add_markdown("")

        for kind, title in (("zero", "① 记录自然零位"), ("open", "② 记录夹爪张开"),
                            ("closed", "③ 记录夹爪闭合")):
            with calibration_folder:
                button = server.gui.add_button(title)
            capture_buttons.append(button)

            @button.on_click
            def capture_reference(_event, kind=kind):
                nonlocal busy
                with lock:
                    if busy or (teleop and teleop.busy):
                        return
                    busy = True
                    sides = {"双臂": ("left", "right"), "左臂": ("left",), "右臂": ("right",)}[selection.value]
                    chosen = {s: readers[mapping.data["ports"][s]] for s in sides}
                    for control in [selection, save, swap, *capture_buttons]:
                        control.disabled = True
                    message.content = "正在采集，请保持静止…"
                try:
                    samples = collect_stable(chosen, kind)
                    with lock:
                        previous = copy.deepcopy(mapping.data)
                        try:
                            for side, sample in samples.items():
                                mapping.capture(side, sample, kind, time.monotonic())
                        except Exception:
                            mapping.data = previous
                            raise
                        message.content = "采集完成。完成三个参考点后点击保存标定。"
                except Exception as exc:
                    with lock:
                        message.content = f"采集失败：{exc}"
                finally:
                    with lock:
                        busy = False
                        for control in [selection, save, swap, *capture_buttons]:
                            control.disabled = False

        with calibration_folder:
            save = server.gui.add_button("④ 保存标定")
        settings = server.gui.add_folder("高级设置", expand_by_default=False)
        with settings:
            swap = server.gui.add_button("交换左右")

        @swap.on_click
        def swap_sides(_event):
            with lock:
                if busy or (teleop and teleop.busy):
                    return
                mapping.swap()
                message.content = "已交换左右，请移动单侧确认对应关系，然后保存。"

        @save.on_click
        def save_mapping(_event):
            nonlocal saved_data
            with lock:
                if busy or (teleop and teleop.busy):
                    return
                try:
                    sides = {"双臂": ("left", "right"), "左臂": ("left",), "右臂": ("right",)}[selection.value]
                    if any(mapping.device(s)[k] is None for s in sides for k in ("zero", "open", "closed")):
                        raise ValueError("请先完成所选机械臂的零位、张开和闭合采集")
                    mapping.save()
                    saved_data = copy.deepcopy(mapping.data)
                    message.content = "已保存标定，下次启动自动加载。"
                except Exception as exc:
                    message.content = f"保存失败：{exc}"

        panels = {}
        for side, label in (("left", "左臂"), ("right", "右臂")):
            with settings, server.gui.add_folder(f"{label}设置", expand_by_default=False):
                with server.gui.add_folder(f"{label} · 方向校对", expand_by_default=False):
                    for index, mid in enumerate(SOURCE_IDS):
                        reverse = server.gui.add_checkbox(
                            f"J{index + 1} 反向",
                            initial_value=mapping.data["signs"][side][index] < 0,
                        )

                        @reverse.on_update
                        def change_sign(_event, side=side, index=index, handle=reverse):
                            with lock:
                                if teleop and teleop.busy:
                                    handle.value = mapping.data["signs"][side][index] < 0
                                    return
                                mapping.data["signs"][side][index] = -1 if handle.value else 1

                with server.gui.add_folder(f"{label} · 原始读数", expand_by_default=False):
                    status = server.gui.add_markdown("")
                    table = server.gui.add_markdown("")
                panels[side] = status, table
        if control_profile is not None:
            from openarm_runtime.teleoperation import attach
            teleop, refresh_teleop = attach(server, control_profile, mapping, readers, report_root,
                                           lock, lambda: busy, lambda: mapping.path.exists() and mapping.data == saved_data,
                                            collection=collection,head_calibration_profile=head_calibration_profile,
                                            head_calibration_root=head_calibration_root,head_video_port=head_video_port)
        for reader in readers.values():
            reader.start()
        print(f"Leader virtual teleoperation http://127.0.0.1:{port}", flush=True)
        last_ui = 0
        while True:
            now = time.monotonic()
            with lock, server.atomic():
                for side in ("left", "right"):
                    path = mapping.data["ports"][side]
                    sample, error, count = readers[path].snapshot()
                    ready = sample is not None and sample.fresh(now) and not error
                    roots[side].visible = bool(ready)
                    angles, opening = ([0] * 7, None)
                    exceeded = []
                    if ready:
                        angles, opening = mapping.angles(side, sample.positions)
                    if ready:
                        values = {f"openarm_{side}_joint{i + 1}": angle
                                  for i, angle in enumerate(angles)}
                        finger = f"openarm_{side}_finger_joint1"
                        limit = models[side].joint_map[finger].limit
                        # Official left opens positive, right negative; zero is closed.
                        extent = limit.upper if side == "left" else limit.lower
                        values[finger] = (opening or 0) * extent
                        for name, angle in values.items():
                            bounds = models[side].joint_map[name].limit
                            if not bounds.lower - 1e-3 <= angle <= bounds.upper + 1e-3:
                                exceeded.append(name.removeprefix(f"openarm_{side}_"))
                        visuals[side].update_cfg(np.asarray(
                            [values[name] for name in visuals[side].get_actuated_joint_names()]))
                    if now - last_ui >= 0.25:
                        device = mapping.device(side)
                        status, table = panels[side]
                        state = "已连接" if ready else "连接中"
                        if ready and any(e & 1 for e in sample.errors):
                            state = "已连接 · 电压偏低"
                        if error:
                            state = ("串口无权限" if "Permission denied" in error else
                                     "供电不稳" if "voltage" in error else "连接中")
                        label = "左臂" if side == "left" else "右臂"
                        statuses[side].content = f"{label} · {state}"
                        status.content = (
                            f"**{state}**\n\n`{path}`\n\n"
                            f"读数帧：{count} · 姿态：{'已对齐' if device['zero'] is not None else '未对齐'}"
                            f" · 夹爪：{'已标定' if opening is not None else '等待两个端点'}\n\n"
                            + (f"供电 {min(sample.voltages):.1f}–{max(sample.voltages):.1f} V\n\n"
                               if ready and sample.voltages else "")
                            + (f"{error}\n\n" if error else "")
                            + (f"舵机告警：{sample.errors}\n\n" if sample and any(sample.errors) else "")
                            + (f"超出模型关节范围（未裁剪）：{', '.join(exceeded)}\n\n" if exceeded else "")
                            + (f"注意：主臂使能状态 {sample.torque}\n\n" if sample and any(sample.torque) else "")
                        )
                        if sample:
                            rows = ["| 主臂ID | 原始位置 | 虚拟角度° |", "|---|---:|---:|"]
                            for index, mid in enumerate(SOURCE_IDS):
                                angle_text = f"{math.degrees(angles[index]):.1f}" if ready else "—"
                                rows.append(f"| {mid} → J{index + 1} | {sample.positions[mid - 1]} | {angle_text} |")
                            grip = f"{100 * opening:.0f}%" if opening is not None else "未标定"
                            rows.append(f"| 8 夹爪 | {sample.positions[7]} | {grip} |")
                            table.content = "\n".join(rows) + (
                                f"\n\n采样耗时 {sample.duration * 1000:.1f} ms"
                                + (" · 原始值为历史数据，已过期" if not ready else ""))
                if now - last_ui >= 0.25:
                    if teleop:
                        refresh_teleop()
                        for handle in [selection, save, swap, *capture_buttons]:
                            handle.disabled = busy or teleop.busy
                    rows = []
                    for side, label in (("left", "左臂"), ("right", "右臂")):
                        device = mapping.device(side)
                        fields = " · ".join(f"{name} {'✓' if device[key] is not None else '待采集'}"
                                            for key, name in (("zero", "零位"), ("open", "张开"), ("closed", "闭合")))
                        rows.append(f"{label}：{fields}")
                    state = "有未保存修改" if mapping.data != saved_data else ("已保存／已加载" if mapping.path.exists() else "尚未保存")
                    progress.content = "\n\n".join(rows + [state])
                    last_ui = now
            time.sleep(1 / 30)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if teleop:
                teleop.close()
            for reader in readers.values():
                try:
                    reader.close()
                except Exception as exc:
                    print(f"Leader reader shutdown: {exc}", flush=True)
        finally:
            server.stop()
