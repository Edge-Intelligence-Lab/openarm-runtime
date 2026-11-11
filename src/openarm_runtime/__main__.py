"""Classic OpenArm: prepare assets, check buses, view feedback, or open user controls."""

import argparse
import json
from pathlib import Path
import signal

from .config import Profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="Follower profile for check/view/panel/calibrate or leader --follow")
    commands = parser.add_subparsers(dest="command", required=True)
    prepare = commands.add_parser("prepare", help="Download and verify pinned model assets only")
    prepare.add_argument("--asset-root", type=Path, required=True)
    leader = commands.add_parser("leader", help="Leader calibration and preview; optional operator-started physical following")
    leader.add_argument("--left-port", required=True)
    leader.add_argument("--right-port", required=True)
    leader.add_argument("--calibration", type=Path, required=True)
    leader.add_argument("--asset-root", type=Path, required=True)
    leader.add_argument("--port", type=int, default=8890)
    leader.add_argument("--follow", action="store_true", help="Add operator-started physical follower controls")
    leader.add_argument("--report-root", type=Path)
    leader.add_argument("--collection-profile", type=Path, help="Enable episode controls and own configured acquisition devices")
    leader.add_argument('--head-calibration-profile',type=Path,help='Passive head calibration from this teleoperator feedback')
    leader.add_argument('--head-calibration-root',type=Path,help='External immutable calibration evidence directory')
    leader.add_argument('--head-video-port',type=int,default=5003,help='Existing loopback camera service; never opens another camera')
    commands.add_parser("check", help="Read-only: verify both Classic buses and disabled motors")
    calibration = commands.add_parser("calibrate", help="Move one v2 arm, find stops and write hardware zeros (experimental)")
    calibration.add_argument("--side", choices=("left", "right"), required=True)
    calibration.add_argument("--report-root", type=Path, required=True)
    calibration.add_argument("--allow-unreadable-mode", action="store_true")
    calibration.add_argument("--no-write-zero", action="store_true",
                             help="Run the physical calibration trajectory without rewriting encoder zeros")
    for name, help_text in (("view", "Read-only live viewer"),
                            ("panel", "User-operated whole-arm control; startup is read-only")):
        command = commands.add_parser(name, help=help_text)
        command.add_argument("--asset-root", type=Path, required=True)
        command.add_argument("--port", type=int, default=8890)
        if name == "panel":
            command.add_argument("--report-root", type=Path, required=True)
    video = commands.add_parser("video", help="Read RGB/depth cameras in one browser page")
    video.add_argument("--profile", type=Path, help="External JSON camera configuration")
    video.add_argument("--simulate", action="store_true", help="Software images only; never open cameras")
    video.add_argument("--port", type=int, default=5003)
    tactile = commands.add_parser("tactile", help="View Paxini IP-M2324-Core tactile forces")
    source = tactile.add_mutually_exclusive_group(required=True)
    source.add_argument("--device", help="USB serial converter path")
    source.add_argument("--simulate", action="store_true")
    tactile.add_argument("--right-device", help="Second serial converter; show both sensors side by side")
    tactile.add_argument("--port", type=int, default=8891)
    tactile.add_argument("--baudrate", type=int, default=921600)
    tactile.add_argument("--device-id", type=int, default=1, help="Sensor module number + 1 (1..255)")
    policy = commands.add_parser("policy", help="Run a model-independent policy episode (classic enables motors)")
    policy.add_argument("--robot", choices=("simulate", "classic"), required=True)
    policy.add_argument("--policy", default="openarm_policy.adapters:HoldPolicy", help="Importable module:factory")
    policy.add_argument("--profile", type=Path, help="External sensor/session/policy JSON")
    policy.add_argument("--sides", nargs="+", choices=("left", "right"), default=["left", "right"])
    policy.add_argument("--duration", type=float, default=10)
    policy.add_argument("--record", type=Path, help="New episode directory outside Git")
    policy.add_argument("--record-mode", choices=("summary", "full"), default="summary",
                        help="Compact result receipt by default; full explicitly saves observations/actions")
    policy.add_argument("--allow-nominal-geometry", action="store_true",
                        help="Explicit operator trial with declared uncalibrated model geometry; not a calibration assertion")
    model_serve = commands.add_parser("model-serve", help="Serve an optional model on loopback; never opens robot devices")
    model_serve.add_argument("--profile", type=Path, required=True, help="Predictor factory/kwargs JSON")
    model_serve.add_argument("--port", type=int, default=8765)
    snapshot_command = commands.add_parser("snapshot", help="Read-only disabled-arm feedback; never enable or move")
    snapshot_command.add_argument("--side", choices=("left", "right"), default="left")
    snapshot_command.add_argument("--output", type=Path, help="New JSON file outside Git")
    camera_calibrate = commands.add_parser("camera-calibrate", help="Offline measured-board hand-eye fit; never opens devices")
    camera_calibrate.add_argument("--observations", type=Path, required=True)
    camera_calibrate.add_argument("--output", type=Path, required=True, help="New calibration estimate JSON; never installed automatically")
    gravity = commands.add_parser("gravity", help="Offline gravity prediction; never opens hardware")
    gravity.add_argument("--profile", type=Path, required=True)
    gravity_input = gravity.add_mutually_exclusive_group(required=True)
    gravity_input.add_argument("--positions", type=float, nargs=8, metavar="RAD")
    gravity_input.add_argument("--episode", type=Path, help="Recorded policy episode for offline diagnosis")
    gravity.add_argument("--side", choices=("left", "right"), help="Required with --episode")
    platform = commands.add_parser("platform", help="OpenArm Platform: integrate existing control, teleop, tactile and vision pages")
    platform.add_argument("--profile", type=Path, help="External platform service configuration")
    platform.add_argument("--port", type=int, help="Override the platform port (default 8888)")
    platform.add_argument("--attach-only", action="store_true", help="Only display existing services; never start or stop them")
    from openarm_collection.cli import configure as configure_collection
    configure_collection(commands.add_parser("collection", help="Inspect, review and export demonstration episodes"))
    args = parser.parse_args()
    if args.command == "camera-calibrate":
        from .camera_calibration import run
        run(args)
        return
    if args.command == "snapshot":
        if args.config is None:
            parser.error("snapshot requires --config")
        from .checks import snapshot
        result = snapshot(Profile.load(args.config), args.side)
        payload = json.dumps(result, indent=2, allow_nan=False) + "\n"
        if args.output:
            with args.output.open("x") as handle:
                handle.write(payload)
        else:
            print(payload, end="")
        return
    if args.command == "model-serve":
        from openarm_policy.models.server import serve
        serve(json.loads(args.profile.read_text()), args.port)
        return
    if args.command == "collection":
        from openarm_collection.cli import run
        run(args)
        return
    if args.command == "gravity":
        from .gravity import load_gravity, diagnose_episode
        if args.episode and args.side is None:
            parser.error("gravity --episode requires --side")
        compensation = load_gravity(args.profile, side=args.side)
        result = (diagnose_episode(compensation, args.episode, args.side) if args.episode else
                  {"model": compensation.metadata(), "gravity_nm": compensation.model.torques(args.positions)})
        print(json.dumps(result, indent=2))
        return
    if args.command == "policy":
        if args.robot == "classic" and (args.config is None or args.record is None):
            parser.error("policy --robot classic requires --config and --record")
        from openarm_policy.cli import run
        run(args)
        return
    if args.command == "platform":
        from openarm_platform import load_config
        from openarm_platform.app import serve as serve_platform
        try:
            config = load_config(args.profile)
            if args.port is not None:
                config = config.with_port(args.port)
        except (OSError, ValueError, TypeError) as exc:
            parser.error(str(exc))

        def shutdown_platform(signum, frame):
            raise KeyboardInterrupt(f"Signal {signum}")

        previous = signal.signal(signal.SIGTERM, shutdown_platform)
        try:
            serve_platform(config, attach_only=args.attach_only)
        except KeyboardInterrupt:
            pass
        finally:
            signal.signal(signal.SIGTERM, previous)
        return

    if args.command == "tactile":
        if not 1024 <= args.port <= 65535 or args.baudrate <= 0:
            parser.error("Port must be 1024..65535 and baudrate must be positive")
        if not 1 <= args.device_id <= 255:
            parser.error("Device ID must be 1..255")
        if args.right_device:
            if args.simulate or args.port > 65524:
                parser.error("--right-device requires --device and port <= 65524")
            if Path(args.device).resolve() == Path(args.right_device).resolve():
                parser.error("Left and right devices must be different")
            from openarm_tactile.dual import serve as serve_tactile
        else:
            from openarm_tactile.app import serve as serve_tactile

        def shutdown_tactile(signum, frame):
            raise KeyboardInterrupt(f"Signal {signum}")

        previous = signal.signal(signal.SIGTERM, shutdown_tactile)
        try:
            serve_tactile(args.device, simulate=args.simulate, port=args.port,
                          baudrate=args.baudrate, device_id=args.device_id,
                          **({"right_device": args.right_device} if args.right_device else {}))
        except KeyboardInterrupt:
            pass
        finally:
            signal.signal(signal.SIGTERM, previous)
        return
    if args.command == "video":
        if not 1024 <= args.port <= 65535:
            parser.error("Port must be 1024..65535")
        if args.profile is None and not args.simulate:
            parser.error("video requires --profile or explicit --simulate")
        from openarm_video import VideoCapture, load_cameras, serve as serve_video
        from openarm_video.profile import simulation_cameras
        try:
            configs = load_cameras(args.profile) if args.profile else simulation_cameras()
            capture = VideoCapture(configs, simulate=args.simulate)
        except (OSError, ValueError, TypeError) as exc:
            parser.error(str(exc))

        def shutdown_video(signum, frame):
            raise KeyboardInterrupt(f"Signal {signum}")

        previous = signal.signal(signal.SIGTERM, shutdown_video)
        try:
            serve_video(capture, port=args.port)
        except KeyboardInterrupt:
            pass
        finally:
            signal.signal(signal.SIGTERM, previous)
        return
    if args.command == "prepare":
        from openarm_viewer.model import prepare
        prepare(args.asset_root)
        return
    if args.command == "leader":
        if args.head_calibration_profile and args.collection_profile:
            parser.error('Head calibration and demonstration collection use separate workspaces')
        if args.head_calibration_profile and (not args.follow or args.head_calibration_root is None):
            parser.error('--head-calibration-profile requires --follow and --head-calibration-root')
        if not 1024<=args.head_video_port<=65535:
            parser.error('--head-video-port must be 1024..65535')
        if args.collection_profile and not args.follow:
            parser.error("--collection-profile requires --follow")
        if args.follow and (args.config is None or args.report_root is None):
            parser.error("leader --follow requires --config and --report-root")
        if not 1024 <= args.port <= 65535:
            parser.error("Port must be 1024..65535")
        from openarm_leader.app import serve as serve_leader

        def shutdown_leader(signum, frame):
            raise KeyboardInterrupt(f"Signal {signum}")

        previous = signal.signal(signal.SIGTERM, shutdown_leader)
        try:
            options = {"control_profile": Profile.load(args.config), "report_root": args.report_root} if args.follow else {}
            if args.head_calibration_profile:
                options.update(head_calibration_profile=args.head_calibration_profile,
                    head_calibration_root=args.head_calibration_root,head_video_port=args.head_video_port)
            from contextlib import ExitStack
            with ExitStack() as stack:
                if args.collection_profile:
                    from openarm_collection.sources import configured
                    options["collection"] = stack.enter_context(configured(args.collection_profile))
                serve_leader(args.asset_root, args.left_port, args.right_port, args.calibration, args.port, **options)
        finally:
            signal.signal(signal.SIGTERM, previous)
        return
    if args.config is None:
        parser.error("--config is required for check, view, panel and calibrate")
    profile = Profile.load(args.config)
    if args.command == "calibrate":
        from datetime import datetime, timezone
        import threading
        import uuid
        from .calibration import calibrate_arm
        from .report import Report
        cancel = threading.Event()
        previous = {sig: signal.signal(sig, lambda *_: cancel.set())
                    for sig in (signal.SIGINT, signal.SIGTERM)}
        report = None
        try:
            report = Report(args.report_root / (
                datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")+"-"+uuid.uuid4().hex+".json"),
                {"source": "cli_user", "action": "auto_calibrate", "side": args.side,
                 "interface": profile.interface(args.side),
                 "allow_unreadable_mode": args.allow_unreadable_mode, "write_zero": not args.no_write_zero})
            print("Automatic v2 calibration: " + ("motion only; NO zero writes" if args.no_write_zero else "motion and zero writes")
                  + ". Ctrl+C stops. Report:", report.path, flush=True)
            calibrate_arm(profile, args.side, cancelled=cancel.is_set, record=report.record,
                          progress=lambda message: print(message, flush=True),
                          allow_unreadable_mode=args.allow_unreadable_mode, write_zero=not args.no_write_zero)
        except BaseException as exc:
            if report:
                report.record({"event": "ended", "error": f"{type(exc).__name__}: {exc}"})
            raise
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
        return
    if args.command == "check":
        from .checks import check
        result = check(profile)
        print(json.dumps(result, indent=2))
        raise SystemExit(0 if result["passed"] else 1)
    if not 1024 <= args.port <= 65535:
        parser.error("Port must be 1024..65535")
    from openarm_viewer.app import serve
    options = ({"control_profile": profile, "report_root": args.report_root}
               if args.command == "panel" else {})

    def shutdown(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")

    previous = signal.signal(signal.SIGTERM, shutdown)
    try:
        serve(args.asset_root, profile.left, profile.right, args.port, "classic", **options)
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    main()
