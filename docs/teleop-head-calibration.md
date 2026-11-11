# Head-camera alignment through leader/follower teleoperation

The operator controls arm pose and gripper through the existing teleoperation
controller. Calibration observes that controller's measured feedback and borrows
frames from the existing local camera service. Explicit mount confirmation asks
the same controller to retain its last dispatched gripper reference until
following ends; arm-pose references still come from the leader. There is no second
CAN owner, autonomous following start, lift, pose search or new gripper goal.

In the external **hardware** platform configuration, set
`teleop_calibration_profile` to an external `openarm.teleop-head-profile.v1` file.
The existing `services.teleop.command` must explicitly configure `leader --follow`.
The platform derives a separate, lazily launched calibration service from that
command. It removes collection options, uses its own loopback port (default 8894,
configurable as `head_calibration_port`), and supplies the calibration profile,
`calibration_root/teleop-head` evidence root and existing vision port. The original
teleoperation/collection service and interface are not modified. Both services
belong to the same exclusive motion group, so they cannot own the arm together.
There is no autonomous alignment controller or alternate movement mode.

The profile has the same `robot_profile`, `kinematics`, frozen `intrinsics`,
`intrinsics_sha256`, indexed ChArUco `board_spec`, `base_frame`, `tool_frame` and
`camera_name` fields described in [camera-calibration.md](camera-calibration.md),
but its schema is `openarm.teleop-head-profile.v1`. It needs **no** preparation
pose, search envelope or candidate library. Its robot profile must equal the
one already selected by the teleoperator. It requires the calibration extra in
addition to the existing teleoperation extras and qualified camera service.
Plain unindexed chessboards are not accepted for this extrinsic solve.

For a standalone leader launch, append `--head-calibration-profile`,
`--head-calibration-root` and optionally `--head-video-port` to the existing
`leader --follow` command. Launching a page does not itself enable following;
the operator still explicitly starts the existing controller. Physical leader
input is not enabled in the separate MuJoCo platform profile.

## Operator workflow

1. Open **头部相机标定 → 机器人对齐**. Click **进入标定控制** and confirm the
   switch after finishing any demonstration recording and supporting the arm.
   Opening the page alone starts no service or following. The complete workspace
   appears inside this page, not in the data-collection interface. Explicitly
   start left-arm following there, then use the leader to position and grip.
2. Insert and clamp the board. Check the fixture confirmation and click
   **确认装板并保持夹爪 · 准备第1张** only when the board is rigidly fixed.
   The existing controller retains its last gripper command (not the
   contact-deflected measured angle), so leader-gripper jitter cannot change
   clamping during capture. This is operator confirmation, not a slip detector.
3. Move to different visible poses using the leader. After stopping for about a
   second, click **采集拟合姿态** (then **采集独立检查**). The arm remains under teleoperation.
    Each click saves at most one qualified original; neither starting following
    nor confirming the mount starts automatic capture. A frame is accepted only
    with fresh enabled per-joint feedback continuously
   bracketing the qualified image acquisition interval. Movement or a cropped
   board rejects capture; it does not command a correction.
4. Collect 12 fitting poses followed by 4 independent check poses. Their roles
   are frozen before the first image. Excite rotations about at least two axes;
   neither repetition nor many almost-frontal views substitutes for diversity.
5. Click **检查并生成标定候选**. A complete group can be solved after the
   operator stops following. The existing consistency thresholds still apply;
the candidate is never automatically installed or activated.

**关闭标定控制** explicitly stops/disables its owner and retains evidence without
starting another controller. Complete the solve before closing; a closed group is
not resumed. Intrinsics stay locked while calibration control is running, so a
hidden tab cannot leave an unnoticed motion owner. The stop button inside the
workspace stops following without closing the page, allowing a complete group to
be solved after motor disable.

Gripper-command or measured-opening changes beyond the existing 0.003-rad
rigidity tolerance invalidate an incomplete group, even if the gripper later
returns to its original reading. The hold remains active even after rejection or
all 16 captures; it must not jump back to the meanwhile-moved leader gripper.
Stop following to release the hold, then restart following to re-clamp. Ending following, changing operator or
retaking control also invalidates an incomplete group. Re-clamp and start a new
group; previous evidence is retained. Small encoder drift is tolerated, not
reissued as a new gripper command. Real measured-opening changes are still
checked against the original 0.003-rad tolerance. Mechanical board slip without an encoder or
command change is not directly observable and may instead fail the solve.

Session/protocol, original PNGs, source metadata, full stationary feedback
brackets and the observations are stored outside Git. A failed solve retains
its observations; no observations are silently deleted or reassigned to improve
the result. This integration has no implicit physical trial or deployment step.

The workspace includes a continuous browser MJPEG head-camera preview and
magnified live view, independent of the slower qualified-PNG/board checks,
whole-board/corner quality feedback, measured stillness gating, stage-specific
buttons and explicit reasons while waiting. A progress bar and accepted-photo
contact sheet distinguish fitting from independent checks. Rejected attempts
do not consume a slot; their available originals and reason are retained under
`rejected/` within the same budget. Similar poses are rejected with a reference
to the prior accepted image, not silently accumulated as additional coverage.
Capture and solve are asynchronous so stop controls remain responsive. Technical
paths/diagnostics are collapsed. Re-sampling requires confirmation and preserves
previous groups; a failed solve is not repeatedly written over.
