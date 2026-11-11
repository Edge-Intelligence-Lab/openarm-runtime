"""Public, hardware-passive API for the OpenArm runtime."""

from .config import Profile
from .control import hold_arm
from .calibration import calibrate_arm
from .telemetry import MotorRange, MotorState, ArmConnection

__version__ = "0.9.0"
__all__ = ["Profile", "hold_arm", "calibrate_arm", "MotorRange", "MotorState", "ArmConnection"]
