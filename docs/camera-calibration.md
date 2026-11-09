# Offline head/wrist hand-eye calibration

## Unified web workflow (head camera only)

The platform now includes **头部相机标定** at `/#camera_calibration`, or
`/camera-calibration` on the same server. Install `.[platform,calibration]`.
Opening the page does not open a camera, acquire CAN, select a controller or
start a service. It also works in `--attach-only` mode. Existing configured video
capture must already be available; the page never restarts it. All robot movement
remains with the on-site operator and the existing control interface.

Intrinsics support **ordinary black/white checkerboards or indexed ChArUco boards,
and pinhole cameras with OpenCV's five distortion coefficients**, not fisheye
lenses or wrist calibration. Select the board type and enter actual square counts
(not inner-corner counts) and measured square length in metres. ChArUco additionally
requires marker length and dictionary; these fields are hidden for checkerboards.
The page defaults to the operator-confirmed 10-by-7 checkerboard with 15 mm
squares and camera `head`; these are editable, not inferred from the image.
Change them when using another board. No session starts automatically.
ChArUco marker size and dictionary still require explicit entry. Confirm the lens model first.

For example, a 10-by-7-square board with measured 15 mm squares has 9-by-6 inner
corners (54 total) and uses `square_size_m=0.015`. The ordinary checkerboard spec is:

```json
{"type":"checkerboard","squares_x":10,"squares_y":7,"square_size_m":0.015}
```

The full grid must be visible; partial checkerboard detections are rejected.
Checkerboard corner IDs are **per-image grid indices**, not globally indexed
physical corners. This is sufficient for intrinsic fitting, but does not establish
a fixed board coordinate frame across rotated views for hand-eye fitting. Such
sessions explicitly reject external-pose imports; adding a direction mark alone
does not make the current detector verify it. Existing indexed ChArUco sessions
retain the offline head observation import. Real-time external-pose capture stays
disabled for both board types.

In the external video profile, add a nonempty `calibration_identity` to the RGB
camera entry: use the actual camera serial, lens and fixed installation identity.
Update that identity whenever the lens, focus/zoom setting, camera mounting or
driver crop changes. This is operator-supplied provenance, not automatically
verified hardware identity. A changed capture generation, resolution, nominal
capture mode or identity is rejected inside a session. The capture backend does
not currently measure focus, crop or exposure settings; freeze those physically.

The live preview uses `/apps/vision/stream/head.mjpg`: one multipart JPEG
connection, up to 30 fps subject to actual capture/network speed. It borrows the
existing reader, skips duplicate frames, and drops intervening frames for slow
clients rather than queuing them. Pausing removes the image source and closes the
preview connection, not the camera owner. It is not calibration evidence.

The separate sampling endpoint is `/apps/vision/calibration/frame/head.png` (replace
`head` with the configured camera ID). The video owner encodes one existing
snapshot as lossless PNG at its actual capture resolution, with atomic metadata
in `X-Calibration-Metadata`. It does not reopen the device or use the resized
JPEG preview. Frames without a configured identity/generation fail closed. Old
external vision services need this contract; the browser cannot provide a custom
URL as a workaround. RGB frames are bounded to 4096 pixels per dimension,
8 million pixels total and 24 MB encoded PNG.

1. Start a session explicitly; this creates evidence under the external
   `calibration_root` platform setting (default `~/.local/share/openarm-calibration`).
   The browser cannot choose filesystem paths. Starting the platform/page alone
   does not create evidence. Preview is separately opt-in.
2. Move the **hand-held** board through image centre/edges, different distances
   and tilts about two axes. Before every capture, select fit or held-out
   validation. At least 8 fit and 3 held-out images are required. The detector
   requires 8 non-collinear indexed corners with at least 20-pixel span in both
   image axes. Corner counts, image coverage/centre and a sharpness indicator
   help the operator; they are not physical acceptance criteria.
3. Solve and inspect per-image RMS, K and distortion. Only fit images calibrate
   K; held-out images estimate their own board poses with that fixed K. A
   two-axis board-normal excitation check rejects degenerate captures. No sample
   is silently removed. **Held-out reprojection is not independent metric
   verification**: inspect board scale, camera model, image geometry and known
   physical references before any deployment. High residuals remain visible;
   they do not become verified merely because a candidate can be saved.
4. Optionally import an existing compliant **head** observation JSON, up to 2 MB
   and 256 samples, to run the offline external-pose fit below. Camera name must
   match the session. Original upload bytes and their hash are preserved.
   Imported timing/FK/model/intrinsics provenance is supplied by the operator,
   not verified by the web runtime. Imported external poses are explicitly
   **not bound to the web-solved intrinsics**; do not treat the bundle as an
   automatically compatible deployed profile.
5. Check the explicit confirmation box to save `SESSION/candidate.json` without
   overwriting an old result. The candidate records evidence hashes, board and
   camera identity, fit/validation split, numerical-library versions and solver
   source hashes. Original PNGs and their metadata/detected corners live under
   `SESSION/SAMPLE/`; original external-pose imports live under `SESSION/head-*/`.
   All results remain `estimated_not_independently_verified`, `installed: false`.
   Saving never changes a machine profile or activates calibration.

**Real-time external-pose capture is deliberately disabled.** Existing video
`received_at` is host receipt after `read()`, not exposure time. There is no
trusted exposure-time/measured-FK pairing interface yet. Do not relabel receipt
times or ordinary recordings to satisfy the offline 50 ms contract. For head
external-pose calibration the camera is fixed to base and the board must be
rigidly attached to the tool; a freely hand-held board cannot establish
`base_from_camera` without another measured base reference.

One browser owns the session; other browsers cannot capture, cancel or save it.
An inactive lease expires after 120 seconds without owner heartbeat/action.
Cancellation invalidates late computation results and retains evidence. It does
not close borrowed devices or stop another page's motion: **cancel is not an
emergency stop**. Page navigation preserves platform iframes, so use the explicit
cancel button rather than assuming a hidden page has stopped. Sessions are
bounded to 64 images and 256 MiB of PNG data. Evidence already written is retained
on cancellation, failure and server shutdown; session resumption and deletion
are not provided in this first version. Candidate creation is atomic and
no-clobber; interrupted partial evidence is not a saved candidate.

## Offline external-pose CLI

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
