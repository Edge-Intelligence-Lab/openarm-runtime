"""Thin composition root; machine/policy choices live in caller-owned JSON."""
from contextlib import ExitStack
import json
from pathlib import Path
import signal

from .adapters import ImportedPolicy
from .observations import VideoObservation, TactileObservation, HTTPVideoObservation
from .recording import EpisodeRecorder
from .robots import ClassicRobot, SimulatedRobot
from .session import PolicySession, SessionConfig


def phase_status(event):
    if event["event"] in ("preparation_started", "preparation_waypoint", "preparation_completed", "task_started", "ended"):
        status = {key: event[key] for key in ("event", "phase", "waypoint", "elapsed_s", "state", "error") if key in event}
        if event["event"] == "preparation_started":
            status["planned_motion_s"] = event["plan"]["planned_motion_s"]
            status["skipped_to_ready"] = event["plan"]["skipped_to_ready"]
        print(json.dumps(status, allow_nan=False), flush=True)


def run(args):
    data = json.loads(args.profile.read_text()) if args.profile else {}
    root = args.profile.resolve().parent if args.profile else Path.cwd()
    unknown = set(data) - {"session", "policy_kwargs", "task", "video_profile", "video_http", "tactile", "cartesian", "preparation", "preparation_profile"}
    if unknown:
        raise ValueError(f"Unknown policy profile keys: {sorted(unknown)}")
    if data.get("video_http") and data.get("video_profile"):
        raise ValueError("Choose camera ownership or borrowed video_http, not both")
    if args.robot == "simulate" and data.get("video_http"):
        raise ValueError("Simulate mode must not read live camera services")
    if data.get("preparation_profile") is not None and data.get("preparation") is not None:
        raise ValueError("Choose inline preparation or preparation_profile, not both")
    preparation_options = data.get("preparation")
    if data.get("preparation_profile") is not None:
        preparation_options = json.loads((root / data["preparation_profile"]).read_text())
    config = SessionConfig(**data.get("session", {}))
    if args.robot == "classic":
        from openarm_runtime.config import Profile
        profile = Profile.load(args.config)
        if profile.rate_hz != config.control_hz:
            raise ValueError("Policy control_hz must match Classic profile rate_hz")
        robot = ClassicRobot(profile, tuple(args.sides), watchdog_s=config.command_timeout_s)
    else:
        robot = SimulatedRobot(tuple(args.sides))
    with ExitStack() as stack:
        recorder = stack.enter_context(EpisodeRecorder(args.record)) if args.record else None
        if recorder is not None and isinstance(robot, ClassicRobot):
            robot.record = recorder.record
        if data.get("cartesian"):
            from .cartesian import CartesianRobot
            options = dict(data["cartesian"])
            options["chain"] = dict(options["chain"])
            options["chain"]["urdf"] = str((root / options["chain"]["urdf"]).resolve())
            robot = CartesianRobot(robot, options, config.control_hz,
                                   recorder.record if recorder is not None else None,
                                   allow_nominal=getattr(args, "allow_nominal_geometry", False))
        sources = []
        if data.get("video_http"):
            source = HTTPVideoObservation(**data["video_http"])
            sources.append(source)
            if recorder is not None:
                recorder.record({"event": "video_source", **source.metadata})
        if data.get("video_profile"):
            from openarm_video import VideoCapture, load_cameras
            capture = stack.enter_context(VideoCapture(load_cameras(data["video_profile"]), simulate=args.robot == "simulate"))
            sources.append(VideoObservation(capture))
        if data.get("tactile"):
            from openarm_tactile import TactileReader
            readers = {}
            for name, options in data["tactile"].items():
                if args.robot == "simulate":
                    options = {"simulate": True}
                readers[name] = stack.enter_context(TactileReader(**options))
            sources.append(TactileObservation(readers))
        preparation = None
        if preparation_options is not None:
            from .preparation import FixedJointPreparation
            options = dict(preparation_options)
            if options.pop("kind", None) != "fixed_joint_waypoints_v1":
                raise ValueError("Unsupported preparation kind")
            preparation = FixedJointPreparation(**options)
        session = PolicySession(robot, ImportedPolicy(args.policy, data.get("policy_kwargs")),
                                sources=sources, config=config, task=data.get("task", ""), recorder=recorder,
                                preparation=preparation, on_event=phase_status)
        previous = {sig: signal.signal(sig, lambda *_: session.stop()) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            session.run(args.duration)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
