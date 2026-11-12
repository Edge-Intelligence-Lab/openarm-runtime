"""Shared Classic v2 joint conventions; values are motor-position degrees."""

# Arm J1..J7: OpenArm 2.0 nominal mechanical intervals, expressed in the
# side-specific motor/model coordinates (not the signs of an illustration).
# Source: enactic/openarm_description@4fe8e42bc4377914c0e800ac1739b1b897fcca3a
# assets/robot/openarm_v2.0/urdf/example/v2.urdf, rounded to the documented degrees.
# These are software guards, not proof of clearance on a modified installation.
# J8 deliberately retains the existing gripper interval: stock left motor zero
# opens positive, right opens negative; custom aperture calibration is separate.
JOINT_LIMITS = {
    "left": ((-200, 80), (-190, 10), (-90, 90), (0, 140), (-90, 90), (-45, 45), (-90, 90), (0, 65)),
    "right": ((-80, 200), (-10, 190), (-90, 90), (0, 140), (-90, 90), (-45, 45), (-90, 90), (-65, 0)),
}
