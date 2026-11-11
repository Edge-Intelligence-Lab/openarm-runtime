# Camera calibration

Robot alignment has one workflow:
[operator-controlled leader/follower calibration](teleop-head-calibration.md).
The operator controls arm pose and grip. Calibration never lifts the arm,
commands mounting actions or searches poses.

## Unified web workflow

Open **相机标定** at `/#camera_calibration` or `/camera-calibration`.
The column contains **头部相机标定** and **腕部相机内参标定**.
**机器人对齐** contains its own operator-controlled calibration workspace when
`teleop_calibration_profile` is configured, separate from teleoperation data
collection. **镜头内参** retains its separate
capture-first intrinsic workflow. Opening a page enables no motors and opens no
camera. Install `.[platform,calibration]` plus the existing teleoperation extras.

### Wrist camera intrinsics

Select **腕部相机内参标定**, then **左腕相机** or **右腕相机**. The selected
camera must already be configured as `left`/`right` in the existing video service
with a distinct `calibration_identity`; the page never opens a second device
handle or starts a robot controller. Opening the page does not fetch or save frames.
Move the board by hand while the camera stays fixed; no arm motion or FK is needed.

The editable wrist default matches the current head board: **ChArUco, 6×4 squares,
22.5 mm squares, 16.5 mm markers, `DICT_4X4_50`**. The earlier 10×7-square plain
checkerboard is not the current physical board; it remains an explicit alternative.
Confirm the actual printed dimensions before starting. Capture
16 fitting and 4 held-out check views with two-axis tilt and edge coverage; the
existing 8 + 3 minimum and quality diagnostics remain unchanged.

Wrist evidence is isolated under `calibration_root/wrist-intrinsics`. Camera
selection is locked during a session; identity, resolution, mode and source
generation may not change. Head and wrist images/results are never combined.
Each saved candidate names one camera and is marked `intrinsics_only` and
`installed: false`; there is no automatic left-to-right copy. Identical model names
do not establish identical optical parameters. Calibrating only the left camera
is sufficient to obtain a candidate for the current left-arm policy, not the right.

After solving, the page shows focal lengths/principal point and fitting/check
errors. **查看去畸变对照** compares the last accepted original evidence frame with
same-K undistortion, without cropping, fresh capture or changing a camera control.
**保存标定候选** explicitly saves without a separate acceptance checkbox. It does
not activate parameters, change a model contract or retrain weights. This workspace
has no robot-coordinate alignment or head-extrinsics import.

### Capture workspace

Intrinsic calibration starts with **开始标定**, a live image and the next action.
Board/camera settings and pose guidance are collapsed until needed. Preview
borrows the existing reader; only explicit captures save full-size lossless PNGs
and source metadata. JPEG previews are not calibration evidence.

The current editable default is indexed ChArUco: 6×4 squares, 22.5 mm squares,
16.5 mm markers, `DICT_4X4_50`, camera `head`. Enter measured dimensions and confirm
print scale. The historical 10×7-square, 15 mm checkerboard has 9×6 internal
corners and is valid for intrinsics, not an orientation-verified extrinsic board.
Teleoperation extrinsics require indexed ChArUco, not a symmetric checkerboard.

For intrinsics, **标定照片** are fitting images and **检查照片** are predeclared
held-out images. Suggestions cover centre/edges, distances and two-axis tilt;
rolling a frontal board is not tilt. Guidance advances only after acceptance.
Recommended counts are 16 fit + 4 check; intrinsic solver minima remain 8 + 3.
Its per-image RMS, matrix and distortion stay visible; no sample is silently
removed. This differs from teleoperation extrinsics' fixed **12 fit + 4 check**.

Inline feedback explains failed captures without advancing counts. Settings lock
for a live session. A foreign window cannot capture, cancel or save another
session. An inactive intrinsic-session lease expires after 120 seconds; status
reads do not acquire ownership. Cancellation retains evidence and is not an
emergency stop. Hidden platform iframes can remain connected: use explicit stop
or cancel controls, not navigation, to end operations.

### Source and timing contract

Configure RGB `calibration_identity` from its serial, lens and installation.
Update it after lens/focus, mounting, mode or crop changes. This is supplied
provenance, not automatic hardware verification. Changed capture generation,
dimensions or identity is rejected within a session.

The video owner serves `/calibration/frame/head.png`, proxied as
`/apps/vision/calibration/frame/head.png`, with atomic metadata in
`X-Calibration-Metadata`. It borrows the existing reader, never reopening a
device. Browsers cannot nominate other URLs or filesystem paths. Frames are
bounded to 4096 pixels/dimension, 8 million pixels and 24 MB PNG. Preview streaming
at `/apps/vision/stream/head.mjpg` drops intervening frames for slow clients.

Robot alignment requires qualified acquisition timing and fresh per-joint
measured feedback. Every joint must continuously bracket acquisition by at least
0.5 s before and 0.2 s after, with gaps ≤0.15 s and position span ≤0.003 rad.
FK uses one original measured row, not a target or invented average. Physical
cameras require V4L2 timing qualification; HTTP receipt cannot be renamed into
exposure time. Physical and simulation sources cannot mix.

### Evidence and output

Evidence stays under external `calibration_root`, not Git. Original PNGs, source
identities, corners and stationary brackets are retained. A rejected manual
capture consumes no pose slot and its available image/reason is retained within
the 256 MiB budget. Closing or starting another group never deletes originals.
Partial writes are not accepted captures or saved candidates.

The intrinsic page retains its advanced offline head-observation import, bounded
to 2 MB and 256 observations. Original upload bytes/hash are retained. Imported
timing, FK and intrinsics provenance are operator-supplied and not automatically
verified or bound to web-solved intrinsics. The intrinsic page does not acquire
robot-state pairs; qualified stationary pairing belongs to the teleoperation
workspace. Intrinsic sessions are bounded to 64 images and 256 MiB; interrupted
or cancelled evidence is retained, with no implicit resumption or deletion.

## Offline external-pose CLI

This tool opens neither camera nor CAN and never installs its output:

```sh
openarm-runtime camera-calibrate \
  --observations /external/measured-observations.json \
  --output /external/new-extrinsics-estimate.json
```

Input `openarm.camera-calibration.observations.v1` requires units `m`,
`joint_state_source: measured_feedback`, mode `head` or `wrist`, and nonempty
`base_frame`, `tool_frame`, `camera_name`, `robot_model_sha256`, `intrinsics_source`.
Samples contain rigid 4×4 `base_from_tool`, `camera_from_board`, RMS ≤1.5 px,
and same-clock `image_captured_at_s`/`robot_received_at_s` within 50 ms. Do not
manufacture these from receive-only recordings. Teleoperation uses its explicit
stationary-bracket schema instead of fabricated timestamp pairs.

For head cameras, fix camera to base and board to tool. For wrist cameras, fix
camera to tool and board to base. A freely hand-held board cannot determine head
extrinsics without another measured reference. This fit does not cover an
actively moving head. Use at least eight diverse views and two-axis rotation
excitation; consistency RMS above 5 mm or 0.03 rad rejects the fit. Validation
never changes fitting; thresholds are not tuned to make a run pass.

## Indexed board extraction API

`openarm_runtime.camera_calibration.charuco_pose(image_rgb, K, distortion,
board_spec)` accepts RGB uint8 and exact `squares_x`, `squares_y`, `square_size_m`,
`marker_size_m`, `dictionary` (`DICT_4X4_50`, `DICT_5X5_100`, `DICT_6X6_250`).
It returns indexed corners, metric `camera_from_board` and RMS. Use measured
print geometry and intrinsics for the exact image mode. Lower-level `board_pose`
requires an already consistently ordered inner-corner grid; symmetric checkerboard
detection alone is not a stable board frame across rotated views.

Results are `tool_from_camera` for wrist or `base_from_camera` for head, marked
`estimated_not_independently_verified`, `installed: false`. Low fitting or
held-out reprojection is not independent physical/metric verification. Verify
scale, image geometry, frames, TCP and physical references before changing an
external machine profile. Synthetic tests do not replace acceptance.
