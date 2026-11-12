# Cartesian policy joint boundaries

The motor-position intervals in `openarm_runtime/joints.py` remain hard software
guards. Arm J1..J7 now use the official OpenArm 2.0 nominal mechanical intervals,
not the old narrower LeRobot operation intervals. The source is
[the official v2 URDF](https://github.com/enactic/openarm_description/blob/4fe8e42bc4377914c0e800ac1739b1b897fcca3a/assets/robot/openarm_v2.0/urdf/example/v2.urdf),
rounded to the documented degrees. Left J1/J2 are -200..80 / -190..10 degrees;
right J1/J2 are -80..200 / -10..190. Both arms have J3/J5/J7 ±90, J4 0..140,
and J6 ±45 degrees. These are not independently measured stops or collision
clearances for a modified installation. J8 retains its existing side-specific
interval and aperture calibration. Do not remove guards to mask a tracking fault.

This is a shared control-layer change: policy joint specs, preparation validation,
teleoperation clamping and Cartesian teleoperation consume the same constants.
It is not a web-port setting. Existing teleoperation gains, timing and cleanup
are unchanged, but their allowed arm joint domain is wider. For Cartesian policy,
IK still intersects these intervals with the configured URDF, so an older,
narrower URDF must be checked separately. No model, expert data or weights change.

A Cartesian execution profile can set `joint_command_margin_rad` (default one degree,
finite scalar between zero and five degrees in radians). This **contracts** the
seven arm-joint IK command intervals inside the intersection of configured motor
and URDF bounds. For example, a ±90-degree arm interval normally commands within
±89 degrees, leaving tracking room; it never expands feedback authorization. The gripper
and fixed-joint preparation keep their existing contracts.

An outward goal at the inner boundary remains a bounded best-effort IK goal:
continue holding that joint while other feasible joints can move. Reaching an
inner boundary alone is not an exception. If the initial/reference position is
between the inner and hard boundaries, the next command may hold or move inward,
but may not move farther outward; normal velocity bounds still apply, with no
instant jump to the inner boundary. Existing feedback-paced execution anchors
new policy chunks to dispatched FK, not an unachieved accumulating goal.

Keep raw measured feedback unchanged. A position outside the hard interval still
rejects, now with the joint name, measured radians, interval and a diagnostic
`joint_feedback_out_of_range` event. NaN, stale feedback, tracking, deadlines,
worker watchdogs and disable cleanup retain their existing behavior. A one-degree
command reserve is an initial configuration choice, not a measured overshoot
bound or a guarantee against future stops. The generic `DifferentialIK` API
keeps a zero-margin default for its other callers; only Cartesian policy opts in
by default. Offline fixture checks are not
physical acceptance. Real trials remain operator-started.
