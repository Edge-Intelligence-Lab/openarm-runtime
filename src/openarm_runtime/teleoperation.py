"""Operator-started relative leader control using the existing whole-arm hold."""

from contextlib import ExitStack, contextmanager
from concurrent.futures import ThreadPoolExecutor
import copy
import math
from pathlib import Path
import threading
import time
import uuid
from dataclasses import asdict

from openarm_leader.mapping import delta, SOURCE_IDS
from .arm import ArmHold
from .config import bus_lease
from .control import check_modes
from .report import AsyncReport
from .transport import ClassicBus
from .joints import JOINT_LIMITS
from .gravity import prepare_gravity
from .homing import HomeCoordinator


def leader_sample(reader):
    sample, error, _ = reader.snapshot()
    if error or sample is None or not sample.fresh(time.monotonic()):
        raise RuntimeError(f"主臂反馈中断或过期：{error or '无新鲜读数'}")
    # Operator-approved: voltage-only warnings on fresh, valid feedback are
    # informational, as in the leader viewer. Combined fault bits still stop.
    if any(code & ~1 for code in sample.errors) or any(sample.torque):
        faults = []
        for i, code in enumerate(sample.errors):
            if code & ~1:
                voltage = f"，{sample.voltages[i]:.1f} V" if i < len(sample.voltages) else ""
                kind = "电压告警" if code == 1 else "舵机告警"
                faults.append(f"ID{i + 1} {kind} 0x{code:02x}{voltage}")
        for i, torque in enumerate(sample.torque):
            if torque:
                faults.append(f"ID{i + 1} 已使能（torque_enable={torque}）")
        raise RuntimeError("主臂：" + "；".join(faults))
    return sample


def relative_offsets(mapping, side, origin, sample):
    """Relative takeover with the upstream 65-degree gripper motor span."""
    offsets = {i + 1: sign * delta(sample.positions[mid - 1], origin.positions[mid - 1]) * 360 / 4096
               for i, (mid, sign) in enumerate(zip(SOURCE_IDS, mapping.data["signs"][side]))}
    _, start = mapping.angles(side, origin.positions)
    _, opening = mapping.angles(side, sample.positions)
    if start is None or opening is None:
        raise ValueError("请先完成夹爪标定")
    offsets[8] = (opening - start) * (65 if side == "left" else -65)
    if any(not math.isfinite(v) for v in offsets.values()):
        raise ValueError("Non-finite leader target")
    return offsets


def follower_targets(side, base, offsets):
    targets = {}
    for mid, (lower, upper) in enumerate(JOINT_LIMITS[side], 1):
        goal = math.degrees(base[mid]) + offsets[mid]
        if not math.isfinite(goal):
            raise ValueError("Non-finite follower target")
        targets[mid] = math.radians(max(lower, min(upper, goal)))
    return targets


class Teleoperation:
    def __init__(self, profile, mapping, readers, report_root, collection=None, *, cartesian_factory=None):
        self.profile, self.mapping, self.readers = profile, mapping, readers
        self.report_root = Path(report_root)
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.worker = None
        self.owner = None
        self.closed = False
        self.message = "待机 · 启动时对齐当前姿态"
        self.collection = collection
        self.home = None
        self.origins = {}
        self.takeover_generation = 0
        self.active_mapping = None
        self.active_readers = {}
        self.cartesian_factory = cartesian_factory
        self.cartesian = {}
        self.mode = "joint"
        self.head_calibration = None
        self.head_calibration_view = None
        self.run_id = None

    def _observe(self, side, event):
        if self.collection:
            self.collection.emit(f"robot/{side}", event['dispatch_started_at'], event)
        if self.head_calibration is not None:
            self.head_calibration.observe(side,event,owner=self.owner,
                generation=(self.run_id,self.takeover_generation),
                following=self.home.phase=='following' and side in self.origins)

    def _track(self,side,control,targets):
        calibration=self.head_calibration
        if calibration is None:
            control.track(targets)
        else:
            # Mount confirmation and its feedback snapshot are atomic relative
            # to dispatch. No second owner or hardware command path is created.
            with calibration.reference_lock:
                control.track(calibration.hold_targets(side,targets))

    @property
    def busy(self):
        return self.lock.locked()

    def start(self, owner, sides, *, home=False, mode="joint"):
        if (not self.profile.mapping_verified or not 1 <= len(sides) <= 2
                or len(set(sides)) != len(sides) or any(s not in ("left", "right") for s in sides)):
            raise ValueError("请确认机械臂映射")
        if self.closed or not self.lock.acquire(blocking=False):
            raise RuntimeError("已有跟随任务")
        try:
            if mode not in ("joint", "cartesian"):
                raise ValueError("Unknown teleoperation mode")
            if mode == "cartesian" and self.cartesian_factory is None:
                raise ValueError("未配置 Cartesian IK 模型")
            self.mode = mode
            # Validate/load the optional model before opening or enabling motors.
            self.cartesian = ({s: self.cartesian_factory(s) for s in sides}
                              if mode == "cartesian" else {})
            mapping = copy.deepcopy(self.mapping)
            selected = {s: self.readers[mapping.data["ports"][s]] for s in sides}
            for side, reader in selected.items():
                if any(mapping.device(side)[k] is None for k in ("zero", "open", "closed")):
                    raise ValueError("请先完成并保存主臂标定")
                leader_sample(reader)
            self.cancel.clear()
            self.home = HomeCoordinator(sides)
            if home:
                self.home.request()
            self.origins = {}
            self.takeover_generation = 0
            self.run_id = uuid.uuid4().hex
            self.active_mapping, self.active_readers = mapping, selected
            if self.collection:
                from openarm_runtime import __version__
                with self.collection.lock:
                    # Preserve sensing streams, replace motion streams for selected arms.
                    self.collection.streams = {k: v for k, v in self.collection.streams.items()
                                                if not k.startswith(("leader/", "intent/", "robot/", "cartesian/"))}
                    self.collection.context.update(dict(runtime_version=__version__, profile=asdict(self.profile),
                                                   mapping=copy.deepcopy(mapping.data), sides=list(sides),
                                                    action_semantics="dispatched_joint_position_rad",
                                                    teleoperation_mode=self.mode,
                                                    cartesian_models={s: c.metadata for s, c in self.cartesian.items()},
                                                   joint_order=[f"{s}/j{i}" for s in sides for i in range(1, 9)],
                                                   takeover={}, controller={}))
                for side, reader in selected.items():
                    self.collection.register(f"leader/{side}", encoding="leader_registers", timestamp_semantics="read_started")
                    self.collection.register(f"intent/{side}", encoding="mapped_targets", timestamp_semantics="computed")
                    self.collection.register(f"robot/{side}", encoding="mit_exchange", timestamp_semantics="dispatch_started")
                    if side in self.cartesian:
                        self.collection.register(f"cartesian/{side}", encoding="tcp_intent_ik_feedback", timestamp_semantics="computed_before_dispatch")
                    reader.on_sample = lambda sample, count, s=side: self.collection.emit(
                        f"leader/{s}", sample.received_at, asdict(sample), count)
            self.owner = owner
            self.message = "正在检查从臂并接管…"
            self.worker = threading.Thread(target=self._run, args=(mapping, selected), daemon=True)
            self.worker.start()
        except BaseException:
            self.owner = None
            self.lock.release()
            raise

    def return_zero(self, owner):
        if not self.busy or owner != self.owner or self.cancel.is_set():
            raise RuntimeError("Only the active operator can return to zero")
        # Serialize with recorder start; neither operation can race an episode
        # into the preparatory motion. No storage operation occurs under this lock.
        guard = self.collection.operation if self.collection else threading.Lock()
        with guard, self.home.lock:
            if self.collection and self.collection.episode is not None:
                raise RuntimeError("请先结束录制，再回零")
            self.home.request()
            if self.collection:
                with self.collection.lock:
                    self.collection.ready = False
                    self.collection.context["takeover"] = {}
            self.message = "正在协调回零 · 七关节，夹爪保持"

    def resume(self, owner):
        if not self.busy or owner != self.owner or self.cancel.is_set():
            raise RuntimeError("Only the active operator can resume following")
        with self.home.lock:
            if self.home.phase != "holding":
                raise RuntimeError("请等待从臂回零并保持")
            if not self.home.settled(time.monotonic()):
                raise RuntimeError("从臂尚未稳定在零点，请等待或重新回零")
            samples = {side: leader_sample(reader) for side, reader in self.active_readers.items()}
            self.origins.update(samples)
            if self.collection:
                with self.collection.lock:
                    self.collection.context["takeover"] = {}
                    self.collection.ready = False
            self.takeover_generation += 1
            self.home.phase = "following"
            self.message = "已接管主臂当前位置 · 相对跟随中"

    def stop(self, owner=None):
        if owner is None or owner == self.owner:
            self.cancel.set()

    def close(self):
        self.closed = True
        self.stop()
        if self.worker:
            self.worker.join(timeout=30)
            if self.worker.is_alive():
                raise RuntimeError("Teleoperation shutdown timed out")
        if self.collection:
            self.collection.close()
        if self.head_calibration_view is not None:
            self.head_calibration_view.close()

    def _run(self, mapping, readers):
        report = None
        try:
            report = AsyncReport(self.report_root / f"teleop-{uuid.uuid4().hex}.json",
                            {"action": "relative_teleoperation", "sides": list(readers),
                              "mapping": mapping.data, "target_mode": "direct_mit",
                              "teleoperation_mode": self.mode,
                             "joint_limits_deg": JOINT_LIMITS, "gripper_excursion_deg": 65})
            prepared = threading.Barrier(len(readers), timeout=10)
            ready = threading.Event()
            state_lock, report_lock = threading.Lock(), threading.Lock()
            holding, errors = set(), []
            origins = self.origins

            def record(side, event):
                if self.collection and event["event"] in ("preflight", "gravity_config", "arm_enabled", "takeover", "home_start", "home_complete"):
                    with self.collection.lock:
                        self.collection.context["controller"].setdefault(side, {})[event["event"]] = copy.deepcopy(event)
                with report_lock:
                    report.record(dict(event, side=side))

            def cancelled():
                if self.cancel.is_set():
                    return True
                for reader in readers.values():
                    leader_sample(reader)
                return False

            @contextmanager
            def stop_peers_on_error():
                try:
                    yield
                except BaseException:
                    self.cancel.set()
                    prepared.abort()
                    raise

            def run_arm(side, reader):
                try:
                    with ExitStack() as stack:
                        interface = self.profile.interface(side)
                        stack.enter_context(bus_lease(interface, exclusive=True))
                        bus = ClassicBus(interface, self.profile.feedback_timeout_s)
                        stack.callback(bus.close)
                        preflight = bus.preflight()
                        record(side, {"event": "preflight", "motors": preflight})
                        gravity = prepare_gravity(self.profile, side, bus, preflight)
                        cartesian = self.cartesian.get(side)
                        if cartesian:
                            # Reject invalid motor/model initial angles before enable.
                            cartesian.reset({mid: preflight[mid]["state"]["position"] for mid in range(1, 8)})
                        check_modes(bus, True, self.cancel.is_set)
                        # No arm enables until every selected bus passes preflight.
                        prepared.wait()
                        control = stack.enter_context(ArmHold(
                            bus, cancelled, lambda event: record(side, event),
                            lambda states: None, tracking=True, gravity=gravity,
                            observe=(lambda event: self._observe(side,event))
                             if self.collection or self.head_calibration is not None else None))
                        # Cancel peer loops before this arm's disable readback waits.
                        stack.enter_context(stop_peers_on_error())
                        period = 1 / self.profile.rate_hz
                        with state_lock:
                            holding.add(side)
                            if len(holding) == len(readers):
                                # Capture both origins together after both ramps.
                                if self.home.phase == "following":
                                    origins.update({s: leader_sample(r) for s, r in readers.items()})
                                ready.set()
                                following = "末端 IK 跟随中 · 虚拟主臂" if self.mode == "cartesian" else "跟随中 · 直接位置跟随"
                                self.message = ("双臂" if len(readers) == 2 else "单臂") + (following if self.home.phase == "following" else "准备回零")
                        previous = time.monotonic()
                        recorded = False
                        generation, home_epoch, completed_epoch = 0, 0, 0
                        while not self.cancel.is_set():
                            now = time.monotonic()
                            if now - previous > max(0.25, 3 * period):
                                raise RuntimeError("Teleoperation control deadline missed")
                            previous = now
                            with self.home.lock:
                                phase = self.home.phase
                            if ready.is_set() and phase == "following":
                                if generation != self.takeover_generation:
                                    control.base = dict(self.home.targets[side])
                                    generation = self.takeover_generation
                                    recorded = False
                                if not recorded:
                                    if cartesian:
                                        cartesian.reset(control.base)
                                    record(side, {"event": "takeover", "leader_positions": origins[side].positions,
                                                  "follower_base": control.base})
                                    recorded = True
                                sample = leader_sample(reader)
                                offsets = relative_offsets(mapping, side, origins[side], sample)
                                targets = follower_targets(side, control.base, offsets)
                                if cartesian:
                                    targets, tcp_event = cartesian.step(targets, control.states, period)
                                    if self.collection:
                                        self.collection.emit(f"cartesian/{side}", now, tcp_event)
                                if self.collection:
                                    angles, opening = mapping.angles(side, sample.positions)
                                    self.collection.emit(f"intent/{side}", now, dict(
                                        leader_received_at=sample.received_at, mapped_angles_rad=angles,
                                        gripper_opening=opening, relative_offsets_deg=offsets, targets_rad=targets))
                                    with self.collection.lock:
                                        self.collection.context["takeover"][side] = dict(
                                            leader=asdict(origins[side]), follower_base_rad=dict(control.base))
                                        self.collection.ready = (self.home.phase == "following" and
                                                                 len(self.collection.context["takeover"]) == len(readers))
                            elif ready.is_set():
                                targets = self.home.step(side, control.targets, control.states, now)
                                if home_epoch != self.home.epoch:
                                    home_epoch = self.home.epoch
                                    record(side, {"event": "home_start", "time": now, "speed_deg_s": self.home.speed_deg_s,
                                                  "gripper": "hold", "origin_rad": dict(control.targets)})
                                if self.home.phase == "holding":
                                    self.message = "从臂已回零并保持 · 可从主臂当前位置开始相对跟随"
                                    if completed_epoch != home_epoch:
                                        completed_epoch = home_epoch
                                        record(side, {"event": "home_complete", "time": now,
                                                      "targets_rad": targets, "states": {mid: asdict(s) for mid, s in control.states.items()}})
                                elif self.home.phase in ("collecting", "moving", "settling"):
                                    self.message = f"缓慢回零 {100 * self.home.progress:.0f}% · 5°/s · 夹爪保持"
                            else:
                                # Keep the first ready arm serviced while its peer ramps.
                                targets = dict(control.base)
                            self._track(side,control,targets)
                            self.cancel.wait(max(0, period - (time.monotonic() - now)))
                except BaseException as exc:
                    with state_lock:
                        if not (self.cancel.is_set() and isinstance(exc, (InterruptedError, threading.BrokenBarrierError))):
                            errors.append(f"{side}: {type(exc).__name__}: {exc}")
                    self.cancel.set()
                    prepared.abort()

            with ThreadPoolExecutor(max_workers=len(readers), thread_name_prefix="openarm-follow") as pool:
                futures = [pool.submit(run_arm, side, reader) for side, reader in readers.items()]
                for future in futures:
                    future.result()
            if errors:
                raise RuntimeError("；".join(errors))
            self.message = "已停止 · 所选从臂均已确认失能"
        except BaseException as exc:
            self.message = f"已结束：{type(exc).__name__}: {exc}"
            if report:
                try:
                    report.record({"event": "ended", "error": self.message})
                except Exception:
                    pass
        finally:
            try:
                if self.head_calibration is not None:
                    self.head_calibration.ended()
                if self.collection:
                    self.collection.ready = False
                    for reader in readers.values():
                        reader.on_sample = None
                    try:
                        self.collection.finish("teleoperation_ended", "Teleoperation ended during recording")
                    except Exception as exc:
                        self.message = f"{self.message}；数采未完成：{exc}"
                if report:
                    try:
                        report.close()
                    except Exception as exc:
                        self.message = f"{self.message}；报告未完成：{exc}"
            finally:
                if self.home:
                    self.home.phase = "stopped"
                self.owner = None
                self.lock.release()


def attach(server, profile, mapping, readers, report_root, mapping_lock, calibration_busy, saved, collection=None,
           head_calibration_profile=None,head_calibration_root=None,head_video_port=5003):
    if head_calibration_profile is not None and collection is not None:
        raise ValueError('标定与示教数采必须使用不同工作区')
    import os
    factory = None
    if os.environ.get("OPENARM_CARTESIAN_PROFILE"):
        from .cartesian_teleop import load_factory
        factory = load_factory(os.environ["OPENARM_CARTESIAN_PROFILE"])
    session = Teleoperation(profile, mapping, readers, report_root, collection, cartesian_factory=factory)
    if head_calibration_profile is not None:
        if head_calibration_root is None:raise ValueError('Head calibration requires an explicit evidence root')
        from openarm_platform.teleop_calibration import attach as attach_head_calibration
        attach_head_calibration(server,session,head_calibration_profile,head_calibration_root,head_video_port)
    with server.gui.add_folder("标定姿态控制" if head_calibration_profile else "真实从臂跟随",order=-100):
        server.gui.add_markdown("从当前姿态接管，主臂控制关节与夹爪。标定不发运动命令。" if head_calibration_profile else
            "从臂回零后，以主臂当前位置开始相对跟随，无需精确对齐主臂。回零仅移动从臂七关节，夹爪保持；停止或页面断开会失能。")
        modes = ("末端 IK · 虚拟主臂", "旧版 · 逐关节") if factory else ("旧版 · 逐关节",)
        mode = server.gui.add_dropdown("控制方式", options=modes, initial_value=modes[0])
        if factory:
            server.gui.add_markdown("末端 IK：小主臂驱动虚拟大臂末端，再求从臂关节；不是小主臂真实 TCP 映射。可从当前静止姿态直接接管，无需回零。尚无桌面避障，请保持空旷。")
        choice = server.gui.add_dropdown("跟随机械臂", options=("双臂", "右臂", "左臂"), initial_value="左臂" if factory or head_calibration_profile else "双臂")
        home = server.gui.add_button("① 从臂缓慢回零",visible=head_calibration_profile is None)
        start = server.gui.add_button("② 开始相对跟随" if collection else "开始跟随")
        stop = server.gui.add_button("停止 · 失能", color="red")
        status = server.gui.add_markdown(session.message)
    refresh_collection = lambda: None
    if collection:
        from openarm_collection.ui import attach as attach_collection
        refresh_collection = attach_collection(server, session, collection)

    @start.on_click
    def begin(event):
        if event.client is None:
            return
        with mapping_lock:
            try:
                if calibration_busy() or not saved():
                    raise ValueError("请先完成采集并保存标定")
                sides = {"双臂": ("left", "right"), "右臂": ("right",), "左臂": ("left",)}[choice.value]
                if session.busy:
                    session.resume(event.client.client_id)
                elif mode.value == "末端 IK · 虚拟主臂":
                    session.start(event.client.client_id, sides, mode="cartesian")
                elif collection and head_calibration_profile is None:
                    raise ValueError("请先点击从臂缓慢回零")
                else:
                    session.start(event.client.client_id, sides)
            except Exception as exc:
                session.message = str(exc)

    @home.on_click
    def go_home(event):
        if event.client is None:
            return
        with mapping_lock:
            try:
                if calibration_busy() or not saved():
                    raise ValueError("请先完成并保存主臂标定")
                if session.busy:
                    session.return_zero(event.client.client_id)
                else:
                    sides = {"双臂": ("left", "right"), "右臂": ("right",), "左臂": ("left",)}[choice.value]
                    session.start(event.client.client_id, sides, home=True,
                                  mode="cartesian" if mode.value == "末端 IK · 虚拟主臂" else "joint")
            except Exception as exc:
                session.message = str(exc)

    @stop.on_click
    def end(_event):
        session.stop()

    @server.on_client_disconnect
    def disconnect(client):
        session.stop(client.client_id)

    def refresh():
        refresh_collection()
        if session.head_calibration_view is not None:session.head_calibration_view.refresh()
        status.content = session.message
        phase = session.home.phase if session.home else "idle"
        direct_ik = (mode.value == "末端 IK · 虚拟主臂" or head_calibration_profile is not None) and not session.busy
        start.disabled = calibration_busy() or not saved() or (not direct_ik and (phase != "holding" if collection else session.busy and phase != "holding"))
        start.label = "开始末端 IK 跟随" if mode.value == "末端 IK · 虚拟主臂" else ("② 开始相对跟随" if collection else "开始跟随")
        home.disabled = (calibration_busy() or not saved() or (session.busy and phase not in ("following", "holding"))
                         or bool(collection and collection.episode is not None))
        choice.disabled = session.busy or head_calibration_profile is not None
        mode.disabled = session.busy

    return session, refresh
