# Changelog

## Unreleased

- Align shared arm J1–J7 motor-coordinate intervals with the official OpenArm 2.0
  URDF; retain existing gripper bounds/calibration. This also changes the allowed
  domain of teleoperation and preparation, not just the web policy workspace.
- Reserve one degree inside arm IK bounds by default for Cartesian policy;
  continue bounded hold/inward motion at the soft boundary rather than treating
  a saturated goal as an error. Keep genuine hard-feedback violations, tracking,
  deadlines, watchdogs and disable cleanup blocking. Report the offending joint
  and exact feedback interval on a hard-limit error. See
  [joint-boundary behavior and limitations](docs/joint-boundary.md).

## 0.9.0 — 2026-11-11

- Integrate registered model loading, sample prediction, explicit policy execution
  and model-input previews into the existing platform. Preserve exclusive motion
  ownership, external-service boundaries, watchdogs and verified disable cleanup.
- Add calibrated native head/wrist image preprocessing, ACT observation variants,
  absolute Cartesian action contracts, feedback-paced execution and opt-in causal
  temporal aggregation. Keep model/input/output contract validation without custom
  checkpoint or adapter content-hash metadata.
- Add head and left/right wrist intrinsic workspaces, continuous video previews,
  and a separate operator-driven head-alignment workspace using existing
  teleoperation feedback. No autonomous pose search or new motor owner is added.
- Add explicit passive MuJoCo platform simulation and offline calibrated replay,
  isolated from physical transports, services and output directories. Add optional
  kernel-timestamped Linux RGB capture and asynchronous latest-frame observation.
- Add opt-in bounded tracking torque integration, disabled by default. Preserve
  gains, firmware-range checks, feedback validity and failure/disable resets.
- Default policy CLI recording to compact `summary.json`; use `--record-mode full`
  for observation/action archives. Optional Rerun recording retains configured
  real observations and submitted commands, not reconstructed missing history.
- Replace local camera parameters and the old head-only asset lock with
  `configs/cameras/current.json`: shared HF repository/commit plus head/wrist paths.
  Consumers must fetch parameters externally and pass explicit paths. The wrist
  file is not two independently measured full calibrations.
- Retain v0.8.2's collection-status freshness fix. Keep tag-only release packaging
  without automatic test jobs; publish wheel/source archives without a custom
  checksum manifest. Remove unused calibration code and imports, and ignore editor
  screenshots without deleting them. No new physical-motion acceptance is claimed.

## 0.8.2 — 2026-11-10

- Publish the operator-approved, installation-specific Astra Pro RGB head-camera
  baseline `20261110-v1`: native 1280×720 intrinsics/distortion, metric
  `base_from_camera`, coordinate conventions, robot FK identity and provenance.
  A SHA-256-pinned current selection prevents silent parameter replacement.
- Bind the same bytes to the existing Hugging Face OpenArm supplied-gripper model
  in asset package `v0.3.1`. The immutable lock and camera scene entry reproduce
  `head_calibrated` without modifying original meshes, task cameras or model/data
  payloads. Retain physical-calibration limitations and the original raw evidence
  externally; this is not independent absolute-accuracy certification.
- Add dependency-free configuration integrity tests and include the baseline and
  asset lock in source distributions. No motor behavior, device selection,
  live-service deployment or automatic hardware activation changes.
- Fix a read-only collection-preview freshness race by sampling wall time after
  reading an atomic status snapshot. Preserve explicit-time checks and stale/future
  rejection; add a deterministic concurrent-publication regression test.

## 0.8.1 — 2026-11-01

- Add same-origin collection visualization at `/#collection`: fresh/stale
  recording status, stream activity and writer queues, plus read-only episode
  inspection, causal camera/tactile/joint playback and acceptance/quality labels.
  A best-effort status publisher stays separate from native acquisition; recording
  controls reuse the existing teleoperation owner.
- Integrate policy visualization into the platform at `/#policy`: built-in
  simulated sessions, submitted-target/feedback curves, inference latency and
  bounded episode replay, available through the same port in attach-only mode.
- Preserve calibrated absolute J1–J7 manual controls, trajectory/effort checks,
  queued motor reports and the calibration trajectory without encoder-zero writes.
  Integrate these with v0.8.0's official gains and acquisition observer while
  keeping pre-command observations and post-command feedback separate.
- Retain v0.8.0 demonstration collection, synchronized return-to-zero workflow,
  native recording and export; preserve read-only CAN reconnection recovery.
- Validate merged behavior with offline fake-device tests, installed-wheel tests
  and browser simulation/replay. This release adds no new physical-motion trial.

## 0.8.0 — 2026-11-01

- Add opt-in teleoperation demonstration collection: native acquisition sinks,
  bounded asynchronous recording, operator episode review, provenance/quality,
  read-only crash inspection, causal alignment and optional LeRobot export.
  Include integrated camera previews and offline simulated operator rehearsal.
  Complete operator-run dual-arm collection trials with three RGB cameras and
  two tactile pads, preserving native timelines and explicit episode acceptance.
- Add operator-triggered synchronized follower return to zero (J1–J7, peak
  commanded speed 5 degrees/s, gripper held), followed by relative takeover from
  the leader's current pose. No precise leader alignment or firmware-zero writes.
  Exclude recording during return/hold and retain stop/disconnect disable cleanup.
- Use official Enactic OpenArm v2 J1–J7 PD gains in shared follower hold/tracking;
  retain existing gripper gains. Operator trials validate the workflow; tracking
  error remains motion-dependent, not a full-workspace accuracy guarantee.
- Move diagnostic report writing off the CAN control loop and record dispatched
  gains, commands, pre-command observations and post-command feedback separately.

## 0.7.0 — 2026-11-01

- Add optional Pinocchio gravity feedforward shared by hold, panel, teleoperation
  and policy execution; explicit model/motor mappings, payload, shadow mode,
  ramp/caps/slew, firmware-range-correct torque encoding and per-cycle audit data.
  Include offline gravity CLI and analytic/lifecycle tests. Operator-authorized
  right-arm 50%/100% compensation trials completed with verified motor disable;
  full-workspace and dual-arm compensation validation remain outstanding.

- Add model-independent policy contracts, timestamped sensor histories, asynchronous
  process-isolated inference, time-anchored action chunks and independent Classic
  CAN execution/watchdogs. Include simulation, plugin CLI, recording/replay and
  offline lifecycle tests. Validate one right-arm sine cycle through the physical
  backend, including recording, gravity feedforward and verified disable.

- Simplify the standalone camera viewer with a primary head view, compact RGB
  views, responsive mobile layout, per-view status and browser update FPS.
  Preview settings and optional depth views are collapsed by default.

## 0.6.0 — 2026-10-31

- Add the local OpenArm Platform workspace with unchanged control, teleoperation,
  tactile and vision subpages, configured single-command startup and independent
  page connections. One loopback port proxies HTTP, streaming images, Viser
  WebSockets and both nested tactile views; remote clients need only one SSH
  forward. Motor control/teleoperation switch exclusively through the existing
  shutdown paths; external services remain externally owned. Failed sensing
  services can be retried explicitly from the workspace.
- Use NiceGUI components and its FastAPI/Uvicorn server for the platform
  dashboard, with aiohttp managing upstream HTTP and WebSocket connections.
  Add the optional `platform` extra; core imports and CLI help stay passive.
  Preserve mounted module views during sensing navigation and keep the existing
  motor-control lifecycle and exclusive mode switching.
- Document replacement of standalone launches, one-port SSH access and wrist
  camera recovery using distinct USB topology paths when device serials collide.
  Hardware profiles and calibration remain external to the package.
- Validate the installed platform wheel on Linux and macOS. Browser acceptance
  covers persistent module views, HTTP/video streaming, WebSockets and clean
  shutdown with software fixtures; read-only workstation checks display all four
  cameras and both tactile viewers through the platform port.

## 0.5.0 — 2026-10-31

- Add the optional `openarm_tactile` capture API and `tactile` extra, pinning the
  external community Paxini SDK. Core imports remain hardware-passive and free
  of GUI/serial dependencies. SDK sources and coordinate data are not vendored.
- Add `openarm-runtime tactile` for IP-M2324-Core 68-point serial sensors, with
  explicit device ID, independent capture, stale/error state, manual reconnect
  and bounded shutdown. Raw force counts are not calibrated newtons.
- Add a left-arm dual-sensor workspace with 3D force views, point readouts, curves
  and independent controls. Preserve the browser-confirmed sensor order; both
  sensors belong to the left arm and the right arm has no tactile sensor.
- Add explicit software simulation, external workstation configuration and
  installed-package regression coverage. Previously documented dual-sensor
  hardware trials observed pressure responses and no capture errors; release
  checks are offline and do not move motors or open tactile hardware.

## 0.4.0 — 2026-10-31

- Dual-arm following with independent CAN workers, all-arm preflight, coordinated
  takeover, peer-fault cancellation and serialized reports. Single-arm selection remains.

- Operator-approved voltage-only leader alarms are nonblocking during physical
  following; stale/corrupt feedback, enabled leaders and other fault bits still stop.

- Optional `leader --follow` adds operator-started physical following,
  relative takeover, direct MIT targets with upstream joint-bound clipping,
  65-degree gripper mapping, stop/disconnect cleanup and reports.
  Operator-run dual-arm following completed a short trial with both disables
  verified; startup remains passive. Quantified tracking accuracy is unmeasured.
- Following no longer inherits manual-slider slew/offset restrictions. Leader
  calibration and diagnostics are collapsed by default in Viser.

- Guided manual leader calibration in Viser: arm selection, stable natural-zero
  and gripper endpoint capture, visible completeness/unsaved status and persistence.
- Capture rejects moving, stale or enabled-leader feedback and retains previous
  values on failure. Advanced direction/device controls remain available.

## 0.3.0 — 2026-10-31

- Add independent `openarm_video` package and `video` extra, with passive imports,
  configurable RGB/OpenNI2 sources, timestamped frames, device leases, reconnect
  lifecycle, and explicit software-only simulation.
- Add `openarm-runtime video`, packaged four-view assets, Safari-compatible finite
  JPEG/PNG requests and raw millimetre-depth download. The vendor SDK and machine
  configuration remain outside the package.
- Fix depth/profile test path comparisons on macOS and let Linux/macOS CI jobs
  complete independently.

## 0.2.0 — 2026-10-31

- Final whole-arm zero convergence corrects drift after intermediate zero moves;
  all eight joints must settle together before disabled-pose checks and writes.

- Calibration moves now use slow, bounded target correction for static position
  error; actual-zero tolerance and manual-hold gains are unchanged. Failure reports
  include target/goal/feedback history captured before disable.
- Calibration move/seek speeds are now 20/10 degrees per second, with ramped
  upstream per-action Kp/Kd settings. Inactive-joint failures include the offending
   feedback.

- Single-arm v2 automatic calibration over Classic CAN, available through the
  Python API, CLI and a simplified arm/action selector in the follower panel.
- Panel assumes MIT when RID10 times out, without an advanced checkbox; explicit
  non-MIT readback is still rejected. CLI/API retain the explicit timeout option.
- Bounded stop seeking, computed-zero positioning, cancellation/disable cleanup,
  and disabled-pose checks with per-motor hardware zero write reports.
- Offline virtual-stop and failure-path tests. Both arms completed zero writes
  (right 139 s, left 126 s); all 16 angle readbacks were unchanged after one user
  power cycle. Absolute accuracy and repeatability remain unquantified.
- Existing hold/leader APIs remain compatible; native SDK and CAN bitrate are
  unchanged. See `docs/calibration.md` for operation and recovery.

## 0.1.0 — 2026-10-31

Initial public release.

- Installable core with composable `leader`, `follower` and `viewer` extras.
- Official OpenArm CAN1.4.1 encoding, Classic CAN telemetry, whole-arm hold and
  bounded relative targets, shared bus ownership and verified disable cleanup.
- Read-only STS3215 leader transport, background reconnect and device-bound
  software pose/gripper calibration.
- Optional Viser follower panel and live leader-to-virtual-model display.
- Public Python APIs, read-only examples, wheel-based offline CI and MIT license.

Known boundaries: Python3.12; follower hardware requires Linux and the native SDK;
GUI labels are Chinese. Physical leader-to-follower teleoperation is not included.
