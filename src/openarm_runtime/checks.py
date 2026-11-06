"""Read-only, bounded dual-bus verification shared by the command-line entry."""

from .config import bus_lease
from .transport import ClassicBus


def snapshot(profile, side="left"):
    """One bounded disabled-arm observation; never enable, home, hold or move.

    Take an exclusive advisory lease to avoid interleaving query transactions
    with a controller or another reader. Busy devices fail, never get detached.
    Feedback is sequential, not a synchronized joint sample; retain every timestamp.
    """
    from dataclasses import asdict
    import time
    from .transport import ReadOnlyBus
    if side not in ("left", "right"):
        raise ValueError("Select left or right")
    interface = profile.interface(side)
    with bus_lease(interface, exclusive=True):
        bus = ReadOnlyBus(interface, profile.feedback_timeout_s)
        try:
            preflight = bus.preflight()
            states = {mid: bus.state(mid) for mid in range(1, 9)}
            if any(state.status != 0 for state in states.values()):
                raise RuntimeError("Read-only snapshot requires disabled motors")
            now = time.monotonic()
            if any(not state.fresh(now, .5) for state in states.values()):
                raise RuntimeError("Read-only feedback snapshot stale")
            return dict(schema="openarm.read-only-feedback.v1", side=side, interface=interface,
                        captured_at=now, oldest_received_at=min(s.received_at for s in states.values()),
                        synchronized=False, motion_commands_sent=False,
                        states={mid: asdict(s) for mid, s in states.items()}, preflight=preflight)
        finally:
            bus.close()


def check(profile):
    result = {}
    for side in ("left", "right"):
        interface = profile.interface(side)
        result[side] = {"interface": interface, "passed": False}
        try:
            with bus_lease(interface, exclusive=False):
                bus = ClassicBus(interface, profile.feedback_timeout_s)
                try:
                    result[side]["motors"] = bus.preflight()
                    bus.prepare_encoder(range(1, 9))  # Encode only; never send controls.
                    result[side].update(sdk_version=bus.sdk.__version__, passed=True)
                finally:
                    bus.close()
        except Exception as exc:
            result[side]["error"] = f"{type(exc).__name__}: {exc}"
    return {"passed": all(bus["passed"] for bus in result.values()),
            "mapping_verified": profile.mapping_verified, "buses": result}
