# Passive MuJoCo backend

The backend supplies physics feedback and rendered head/overview cameras. It has
no automatic calibration, lift, mounting sequence or pose search. Physical leader
input and tactile simulation are not wired into it; their pages state that
limitation instead of opening hardware or fabricating data. Robot alignment uses
only [teleoperation](teleop-head-calibration.md).

Install `.[platform,calibration,gravity,simulation]`. EGL needs a working graphics
runtime; importing MuJoCo does not verify rendering. Never fall back to real CAN,
serial or cameras. Use an external platform `simulation_profile`, distinct ports,
and explicit log/calibration/collection/policy paths under its `output_root`.
External commands and auxiliary services are forbidden; other services are not
stopped. Old automatic configurations are rejected, not silently migrated.

The `openarm.simulation.v1` profile requires:

- `calibration_profile`: passive `openarm.teleop-head-profile.v1` FK, board and
  frozen synthetic intrinsics configuration.
- `geometry`: pinned `model_xml`, `model_sha256`, `robot_roots`, `tool_body`,
  `gripper_joints` transmission and supported `fixed_joints`. This is geometry,
  not a search plan.
- `output_root`, `video_port`, eight-angle `initial_rad`, `tool_from_board`.
- `cameras`: `head` and `overview`, each with `position_m`, `target_m`, `fovy_deg`,
  `width`, `height`, `fps` in the MuJoCo world frame.
- Optional `board_contact_geoms`: explicit pad geometry names.

Synthetic intrinsics have `simulation: true`, zero distortion and matrix
`[[f,0,width/2],[0,f,height/2],[0,0,1]]`, with `f=height/(2*tan(fovy/2))`.
Identity is `mujoco:<first 16 geometry SHA256 characters>:head`; its file hash is
pinned in the passive calibration profile. Physical intrinsics are rejected.

The private scene keeps robot geometry, not the table/task objects; source assets
are unchanged. Rendering uses an independent snapshot. Disabled/opposite-arm
joints are supported; friction, mass, camera placement and transmission remain
assumptions. The rigid fixture's nominal 12-mm spacer at tool
`(0.00531736,0,-0.17)` m can provide explicit pad contacts, but does not validate
board thickness, slip, retention or physical gripping force. Source/scenario
identities and limitations remain in metadata. This is not a calibrated twin.

## Replay a selected physical camera estimate

`openarm_simulation.calibrated_replay.CalibratedReplay` renders recorded joint
poses with frozen physical camera intrinsics and `base_from_camera`. It does not
launch a platform, run physics, enable following or open hardware. This is a
distinct offline consumer: do not relabel physical intrinsics as synthetic to
bypass `SimulationBackend`'s isolation checks.

An external `openarm.calibrated-replay.v1` binding contains `sources` (estimate,
observations, session, intrinsics: each a relative `path` and file `sha256`),
`base_frame`, `kinematics` and `geometry`. Kinematics retain the calibration's
hash-pinned URDF, motor order/signs/offsets, locked joints, tool frame/TCP and
encoder-limit tolerance. Geometry supplies the pinned scene XML, robot roots,
tool body, gripper mapping and explicit `base_from_collision_world` and
`body_from_tool`. The replay rejects source-digest differences and any discrepancy
between MuJoCo tool FK and the original calibration FK; never hand-adjust the
camera to hide a model-coordinate mismatch. Keep the source assets shared and
record mesh dependency hashes alongside the external binding.

```python
from openarm_simulation.calibrated_replay import CalibratedReplay

replay = CalibratedReplay(binding_path)
try:
    rgb, identity = replay.render(12)  # original sample 13; roles are not reassigned
finally:
    replay.close()
```

The camera preserves unequal focal lengths and off-center principal point, with
the explicit OpenCV-to-MuJoCo optical-axis conversion. Its output is ideal pinhole;
compare with the original PNG undistorted using the same frozen K, not with an
unlabelled warp or an adjusted camera pose. The rendered ChArUco face uses explicit
UVs so indexed markers are not mirrored by automatic box texturing. Renderer
projection tests only verify this conversion, not physical calibration accuracy.

The board pose comes from calibrated `tool_from_board`. Its printed area is not
the backing sheet. Right-arm pose, real gripper transmission, table geometry,
lighting and background are not measured by this calibration; keep their nominal
or absent status visible. The overview camera is illustrative, not calibrated.
Retain source-frame/waypoint identities with each derived image. Selecting the
binding for simulation does not install it into a real robot or certify absolute
accuracy, fixture rigidity, dynamic camera timing or both-arm geometry.
