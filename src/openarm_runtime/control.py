"""GUI-independent whole-arm lifecycle built from the validated controller."""

from contextlib import contextmanager

from .arm import ArmHold
from .config import bus_lease
from .telemetry import IDS
from .transport import ClassicBus
from .gravity import prepare_gravity


def _ignore(_event):
    pass


def _running():
    return False


def check_modes(bus, allow_unreadable=False, cancelled=_running):
    """Read control modes; an explicit non-MIT value is always rejected."""
    modes = {}
    for mid in IDS:
        if cancelled():
            raise InterruptedError("User stop")
        try:
            mode = bus.parameter(mid, 10)
        except TimeoutError:
            if not (allow_unreadable is True or
                    isinstance(allow_unreadable, frozenset) and mid in allow_unreadable):
                raise RuntimeError(f"ID{mid}: RID10 unreadable; explicit MIT trial required") from None
            mode = None
        if mode not in (None, 1):
            raise RuntimeError(f"ID{mid}: mode={mode}, expected MIT; not enabled")
        modes[mid] = mode
    return modes


@contextmanager
def prepare_arm(profile, side, *, allow_unreadable_mode=False, cancelled=_running, record=_ignore):
    """Acquire and preflight the CAN bus while all motors are disabled."""
    if not profile.mapping_verified:
        raise ValueError("Confirm physical left/right mapping before control")
    interface = profile.interface(side)
    if cancelled():
        raise InterruptedError("User stop")
    with bus_lease(interface, exclusive=True):
        bus = ClassicBus(interface, profile.feedback_timeout_s)
        try:
            preflight = bus.preflight()
            record({"event": "preflight", "motors": preflight})
            gravity = prepare_gravity(profile, side, bus, preflight)
            modes = check_modes(bus, allow_unreadable_mode, cancelled)
            record({"event": "modes", "readback": modes})
            yield bus, gravity
        finally:
            bus.close()


@contextmanager
def hold_arm(profile, side, *, allow_unreadable_mode=False, cancelled=_running,
             record=_ignore, publish=_ignore, manual_motion=False):
    """Acquire bus, preflight, enable/hold, then disable and close on exit.

    Stop readers on the selected bus before entering. Call ArmHold.tick() from
    one thread at profile.rate_hz, with degree offsets from the enable pose.
    Set manual_motion to share the manual panel's joint and trajectory bounds.
    """
    with prepare_arm(profile, side, allow_unreadable_mode=allow_unreadable_mode,
                     cancelled=cancelled, record=record) as (bus, gravity):
        with ArmHold(bus, cancelled, record, publish, gravity=gravity,
                     manual_side=side if manual_motion else None) as arm:
            yield arm
