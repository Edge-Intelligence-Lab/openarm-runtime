# Offline head/wrist hand-eye calibration

This tool never opens cameras, CAN, or motors, and never installs its output.
Install the optional scientific/video dependencies in an offline environment.
The indexed detector additionally needs OpenCV with `aruco.CharucoDetector`.

```sh
openarm-runtime camera-calibrate \
  --observations /external/measured-observations.json \
  --output /external/new-extrinsics-estimate.json
```

## Observation contract

The input JSON has schema `openarm.camera-calibration.observations.v1`, `units`
`m`, `joint_state_source` `measured_feedback`, and `mode` `head` or `wrist`.
Also provide `base_frame`, `tool_frame`, `camera_name`, `robot_model_sha256`,
and `intrinsics_source` (nonempty provenance strings).

Each item in `samples` contains:

- `base_from_tool`: measured-feedback FK as a rigid 4×4 transform.
- `camera_from_board`: metric indexed board PnP, also a rigid 4×4 transform.
- `reprojection_rms_px`: nonnegative, at most 1.5 pixels.
- `image_captured_at_s` and `robot_received_at_s`: finite timestamps on the same
  clock, paired within 50 ms. Camera HTTP receipt time is not an exposure-time
  measurement; do not silently relabel timestamps from old recordings.

For **wrist**, rigidly attach the camera to the tool and keep the board fixed in
the base frame. For **head**, keep the camera fixed relative to base and rigidly
attach the board to the tool. Do not mix installations or move the board/camera
independently inside one fit. An actively moving head needs its own measured
kinematic frame; this fixed-head fit does not cover it.

Capture at least eight diverse views with two-axis rotation excitation, varying
positions and ample image coverage. Physical capture and movement remain
operator-controlled. A fit rejects degenerate motion and consistency RMS above
5 mm or 0.03 rad; it does not quietly delete outliers.

## Indexed board extraction API

`openarm_runtime.camera_calibration.charuco_pose(image_rgb, K, distortion,
board_spec)` accepts a recorded RGB uint8 image. Specify `squares_x`, `squares_y`,
`square_size_m`, `marker_size_m`, and `dictionary` (`DICT_4X4_50`,
`DICT_5X5_100`, or `DICT_6X6_250`). Use the exact printed board geometry and
calibrated intrinsics for that image size/crop; do not infer physical square
dimensions from image pixels. The result includes uniquely indexed corners,
`camera_from_board` and reprojection RMS. Pair it with actual measured robot FK
and exposure timestamps before fitting. No recorded-episode timestamp or FK
provenance is inferred automatically.

The lower-level `board_pose` helper expects an already consistently ordered
inner-corner grid; ordinary symmetric chessboard detections alone do not establish
a stable board frame across rotated views.

## Output boundary

Output is `tool_from_camera` for wrist or `base_from_camera` for head, with status
`estimated_not_independently_verified`. Validate held-out observations, frame
conventions, board scale, TCP and image intrinsics independently before updating
an external machine profile. Low fitting residual alone is not physical
calibration verification. Synthetic tests and nominal simulation cameras do not
replace that work.
