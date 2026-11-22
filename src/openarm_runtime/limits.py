"""Stock OpenArm v2 motor coordinates after hardware zero calibration.

Source: enactic/openarm_description assets/robot/openarm_v2.0/urdf/example/v2.urdf
https://docs.openarm.dev/hardware/openarm-2.0/motor/
Angles use the degree endpoints already used by our stop calibration (URDF
radians are rounded). Velocities are model/no-load ceilings, not measured
loaded performance. Finger-link limits do not specify gripper motor travel.
"""

import math

JOINT_VELOCITY_RAD_S = {1: 16.755, 2: 16.755, 3: 5.4454, 4: 5.4454,
                        5: 20.944, 6: 20.944, 7: 20.944}
# The URDF/no-load ceiling is not an operating limit for this 20 Hz position
# controller. High-speed operation needs validated trajectory/effort control.
MAX_MANUAL_SPEED_DEG_S = 5.0
RELATIVE_GRIPPER_LIMIT_DEG = 30.0


def joint_bounds_deg(side):
    if side not in ("left", "right"):
        raise ValueError("Select left or right")
    return {1: (-80.0, 200.0) if side == "right" else (-200.0, 80.0),
            2: (-10.0, 190.0) if side == "right" else (-190.0, 10.0),
            3: (-90.0, 90.0), 4: (0.0, 140.0), 5: (-90.0, 90.0),
            6: (-45.0, 45.0), 7: (-90.0, 90.0)}


def manual_velocity_rad_s(mid, firmware_range, *, arm_speed_deg_s=MAX_MANUAL_SPEED_DEG_S):
    speed = MAX_MANUAL_SPEED_DEG_S if mid == 8 else arm_speed_deg_s
    return min(math.radians(speed), firmware_range)
