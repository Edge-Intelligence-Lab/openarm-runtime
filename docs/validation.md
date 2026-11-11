# Hardware validation

## v0.9.0 offline release checks — 2026-11-11

- Fresh Python 3.12 core-only wheel environment: 565 tests ran successfully,
  including 208 optional-dependency/external-asset skips. Passive imports and
  CLI help were checked outside the checkout with installed-package assertions.
- A separate installed-wheel environment with platform, calibration, Cartesian,
  simulation and viewer extras plus Rerun: 561 passed, 4 skipped and 347 subtests
  passed. One upstream Starlette TestClient deprecation warning remains.
- Model-adapter checks in the existing inference environment: 73 passed,
  1 skipped and 112 subtests passed. No training or checkpoint changes were made.
- Wheel/sdist contents include the new browser resources and RGB helper source;
  camera configuration contains only HF references, with no parameter copies.
  The native RGB helper passed a C++17 syntax/warning check without opening devices.
- Software-only policy CLI smoke checks completed in both summary and full
  recording modes. Required optional libraries resolved in an isolated environment;
  runtime gains, feedback checks and disable cleanup remain covered by fake devices.
- These are Linux/offline checks, not new physical-motion or macOS acceptance.
  No real cameras, CAN/serial devices, motors or live hardware services were started.

## Leader calibration and dual-arm following — 2026-10-31

The operator saved both natural-reference poses and gripper endpoints, then
confirmed virtual left/right assignment and direction. The saved calibration
reloads with complete references and valid endpoint spans.

On the Classic v2 workstation, a single-arm follow trial ran for about 51 s after
takeover. A subsequent dual-arm trial ran for about 54 s after coordinated
takeover; both arms reported eight verified disables and no terminal error.
The operator confirmed satisfactory operation. Reports remain outside Git:
`teleop-2398ce95781b4d92895bc5f817fc1292.json` and
`teleop-50f60cdd68ea4b2ba092d9b5e865ff54.json`.

An earlier trial stopped on leader ID6's voltage-only 0x01 alarm at 3.9 V. The
operator explicitly chose to treat voltage-only alarms on valid fresh feedback
as nonblocking. Other/combined fault bits, stale feedback and enabled leaders
remain rejected. This does not establish power-supply robustness.

Offline regression covers wrap-aware stationary capture, direct targets and joint
clipping, all-arm preflight before enable, continued holding during peer startup,
peer-fault cancellation and cleanup. Tracking accuracy, synchronized training
capture, measured full gripper travel, collision avoidance and long-running
dual-arm operation are not quantified by these short trials. The video library
from v0.3.0 remains independently usable; it is not yet synchronized with control.

## Video library extraction (2026-10-31)

- Added `openarm_video` with optional dependencies and the shared runtime CLI.
  Passive-import tests reject socket, process and worker-thread creation and
  confirm that NumPy, OpenCV, Flask and unrelated hardware SDKs stay unloaded.
- Built wheel and sdist, including browser assets, the native adapter source and
  the source distribution's example profile. No vendor binaries are bundled.
- In a fresh Python3.12 environment outside the checkout, the core-only wheel
  passed 96 of111 tests, with15 optional-dependency/external-asset skips. Adding
  the wheel's `video` extra passed110 of111 tests; only external model assets
  were skipped. Imports originated in site-packages; `pip check` passed.
- WebKitGTK2.52.6 ran two simultaneous simulated four-camera pages for26 seconds,
  with100 layout changes and10 pause/resume clicks per page. The2635 HTTP
  requests had no unexpected HTTP/fetch errors (intentional cancellation excluded);
  decoded image hashes changed for each camera on each page.
  This exercises the Safari-compatible finite-request design, but is not a test
  on the user's Safari installation or a physical-camera acceptance run.
- The public build API compiled the depth adapter against the workstation's
  external matched Orbbec SDK without opening a device. Extraction verification
  did not change firmware, start a robot controller or migrate the live cameras
  to this library. Existing standalone hardware evidence remains separate.

## Standalone package extraction (2026-10-31)

This repository is the extracted package, not the original research checkout.
Historical `code/hardware` paths below refer to the source project at validation time.

- Local core-only install: 59 tests, 57 passed, two skipped (external assets and
  installed-wheel-only assertion). Core imports work without GUI, serial or CAN SDK.
- Built both sdist and wheel; installed the wheel in a new Linux/Python3.12 environment
  outside either source checkout with leader/viewer extras. All59 tests passed,
  including actual official-model loading and an assertion that imports originate
  in site-packages. Tests do not inject a source directory into sys.path.
- Installed follower extra from the wheel metadata using the existing official C++
  prefix and documented build configuration. SDK1.4.1 import, all8 stateless MIT
  encoders/zero-feedforward checks and dependency compatibility checks passed.
- Extraction verification opened no CAN or serial device, enabled no motor, and did
  not replace/restart the active workstation service. The new lifecycle adapter is
  covered with mode-rejection, passive-import and exception-cleanup tests.
- Physical validation below remains the original hardware evidence. A package build
  is not new real-machine acceptance. The consumer subsequently switched to the
  installed package and repeated all59 tests successfully. Read-only checks passed
  on both CAN buses (16 motors disabled), with150 complete samples per leader bus.
  Separate Chrome checks loaded both leader and follower interfaces successfully.

## Working baseline

- Linux / Python 3.12, official OpenArm CAN SDK 1.4.1 (pinned source and uv lock),
  Classic CAN 1 Mbps through SocketCAN. No SDK fork or downgrade.
- User-confirmed wiring: can0 = right, can1 = left; IDs1–8 on each bus.
- Viser: user-triggered whole-arm enable/hold, per-joint ±30° target sliders,
  default target speed 5°/s, reference follower gains. One arm at a time.
- Startup is read-only. Control uses SDK packet encoders with fresh batch feedback;
  stopping/disconnecting attempts disable and checks each motor. No automatic zero writes.

## Hardware evidence (2026-10-30)

The user reported that the whole-arm interface moved the robot successfully.
Saved workstation reports contain arm_hold sessions on both buses and successful
disable readbacks for all eight motors of each operated arm:

- Right: `20261030T153349Z-41002449db0d4d8cbf08206cb550bab2.json`,
  `20261030T153734Z-53230d07e1864b11bbd1212b124098e8.json`.
- Left: `20261030T153827Z-aa2e82d6b0f443f38a21c4d33462a11d.json`.
- Final `end-of-day-check.json`: both buses passed, all16 status=0 (disabled).
  Viser and SSH forwarding were then stopped at the user's request.

Reports are outside Git under the workstation's `robot_deps/openarm-reports` and
`robot_deps/openarm-backups`. These establish an interactive working baseline,
not quantified trajectory accuracy, full-range acceptance or unattended reliability.

## Retained protocol findings

Development gravity extension (2026-11-01): optional Pinocchio inverse statics and
bounded feedforward are implemented; see [gravity configuration and validation](gravity.md).
Zero-feedforward behavior below describes the previously commissioned baseline.
The extension supports nonzero torque using the firmware/SDK TMAX ratio without
changing firmware or the official SDK. Operator-authorized right-arm active
trials are recorded in [physical results](gravity.md#physical-right-arm-trials);
these do not constitute full-workspace or dual-arm compensation acceptance.

- Each motor's can_br must read4; host configuration must be Classic 1 Mbps.
- J1/J2 RID10 may time out despite normal state/control replies. The panel's explicit
  MIT-trial option permits this case; a readback identifying another mode still rejects.
- Right ID5 TMAX=3 Nm differs from SDK DM4310's 10 Nm. Firmware is left unchanged.
  Current commands enforce zero-feedforward bits=2047, independent of TMAX; actual
  feedback ranges are used for decoding/effort checks. Position/velocity mismatches reject.
- Command responses can arrive after the next request. Transition reads are spaced,
  previous replies drained, and enable/disable statuses confirmed with bounded waits.

## Cleanup verification (2026-10-31)

The retained SDK/Classic/whole-arm path is unchanged. Consolidated to one runtime CLI
with prepare/check/view/panel and separated operation reports from CLI code.
Superseded isolated-joint controls, unvalidated calibration/zero prototypes and the
duplicate viewer CLI/checker were archived outside the repository and removed from
the runnable tree. Protocol, shutdown, UI and model regressions remain.

Cleanup checks use offline tests, installed SDK import/encoding and model loading;
they do not start a live controller, send motor commands or alter CAN configuration.

- Local: 45 tests, 44 passed and the external-model asset test skipped.
- Workstation: all45 tests passed with the retained official model assets.
- `uv lock --check --offline`, `uv pip check`, SDK1.4.1 import and existing machine
  profile loading passed; installed SDK/uv dependencies were not rebuilt or upgraded.
- Source snapshots before cleanup are preserved outside Git in the local archive
  and workstation `openarm-backups/hardware-source-before-cleanup-20261031T104931.tar.gz`.
  Retired migration logs, DMTool inspection bundle and installer/asset transfer copies
  are grouped under `openarm-backups/retired-tools-20261031`.

## Leader-driven virtual arms (2026-10-31)

Read-only inspection on `yunzhen_ws` identified two USB serial adapters and all16
STS3215 servos (model777, IDs1–8 on each, 1 Mbps/protocol0). All torque-enable
registers read0. Stable device identities end in `5B14114594` and `5B90150374`;
their physical left/right assignment still requires a user movement check.

Using the maintained read-only SDK boundary, 30 complete eight-servo samples per
bus succeeded, averaging 3.95 / 3.89 ms per sample. The new leader Viser route was
started and browser-checked with live counts for both arms and official geometry.
Local tests: 50 passed plus one external-model skip; workstation: all51 passed.
Tests cover transmit read allowlisting, signed feedback, encoder wrap/remap/signs,
device-bound calibration persistence, stale/enabled capture rejection and failure cleanup.

A subsequent concurrent 150-cycle-per-bus read observed one voltage-error reply on
adapter `5B14114594`, ID7. Read voltage ranged3.9–4.9 V on that bus and4.4–4.9 V on
the other; configured minimum was4.0 V. Voltage-only alarms on complete, checksum-valid
replies now remain usable for virtual display and are shown as a nonblocking status.
Other servo faults, transport errors and stale samples still hide the affected arm.
Intermittent power stability therefore remains an on-site check, not a verified property.

This establishes the serial-to-model display path. User movement, physical left/right
mapping, pose alignment, joint directions and gripper endpoints await on-site confirmation.
Software calibration is intentionally separate from follower hardware calibration;
the leader route does not connect to follower CAN or send physical motion commands.

The first user trial exposed a confusing persistent zero-pose preview: serial values
changed while that checkbox deliberately froze the displayed pose. The preview was
removed. The default panel now shows two connection states, realignment and side swap;
calibration and diagnostics are collapsed. Realignment captures both current poses as
relative origins, with rollback on a failed capture. A two-frame viewer regression
checks that changed encoder inputs update both model configurations without a preview
override. Workstation tests after this correction: all52 passed. Voltage-only alarms
are also tested separately from corrupt/short reads and combined fault bits.

## Automatic calibration acceptance — 2026-10-31

One OpenArm v2 rig with stock grippers, Classic 1 Mbps and SDK1.4.1 completed
operator-started calibration on both arms. Right: 138.88 s; left: 126.12 s.
Each report contains eight stop solutions, simultaneous final-zero settling,
verified whole-arm disable, eight verified hardware zero writes and completion.
Reports and machine-specific offsets remain outside the repository.

With the arms kept in place, a user power-cycled motor power. Fresh readbacks
from all 16 motors exactly matched their pre-cycle encoder values (maximum
absolute value about 0.033 degrees). All motors remained disabled and can_br4.
This validates persistence on this rig, not absolute mechanical zero accuracy.

During the outage, active polling left both host links in ERROR-WARNING with
nonzero TX error counters. Cycling host links alone did not clear them. Bounded
read-only can_br queries confirmed the configuration, successful transmissions
reduced counters to zero, and the standard Classic checks passed. The read-only
panel was restarted. Automatic outage recovery is not yet accepted.

## Remaining follower work

Independent sign/geometry verification, gripper transmission mapping, quantified accuracy,
collision-aware joint limits and long-running/disconnect robustness are not accepted.
Calibration repeatability and multi-pose precision remain unmeasured; see
[calibration](calibration.md).
Continue new real-machine validation through explicit
user actions; the SDK's protocol range is not a mechanical safe-contact limit.


## Single-port platform acceptance — 2026-10-31

The platform proxies control, teleoperation, tactile and video through one
loopback address. Local validation covered 197 Python tests (two optional
checks skipped), four Node frontend behavior tests, and a source/wheel build.
A fresh core-only environment outside the checkout ran all 197 tests with
16 optional-dependency checks skipped, including installed-resource assertions.

A headless Chromium trial used actual Viser servers with software-only control
fixtures, the existing dual-tactile document with two fixture viewers, and the
video simulator. It exercised GUI events over WebSocket, sensing navigation
while retaining the controller connection, control/teleoperation service
switching in both directions, and four displayed camera frames. All 1,289 HTTP
requests and five observed WebSocket connections used the single gateway
address. No page JavaScript errors or off-origin module requests were observed.
This trial did not open motor hardware.

The local platform was then started against the workstation's configured
services. A browser opened only tactile and vision: both real tactile viewers
reported acquisition at about 59.8 Hz; head RGB and depth images displayed at
640-pixel width. All 170 observed HTTP requests and both tactile WebSockets
used the platform address, with no page JavaScript errors. Wrist cameras were
disconnected. An independent controller was still listening on its configured
endpoint, so it remained externally owned and physical teleoperation switching
was not exercised. The configured leader serial adapters and saved runtime
leader calibration were absent. Real following requires those prerequisites
and an operator-run motion trial; software gateway acceptance does not claim
physical tracking accuracy or hardware readiness.


## Wrist-camera recovery — 2026-10-31

A subsequent device-level check found both wrist UVC cameras enumerated and
capable of acquisition. The earlier disconnected status came from missing
`/dev/camera_wrist_*` aliases: their udev rules referenced an old USB topology.
Both cameras reported the same serial identifier, so the external video profile
was updated to distinct current topology paths, with the operator confirming
left/right from captured images.

The workstation's three legacy video processes were replaced by the packaged
`openarm-runtime video --profile` service owned by the platform. Profile backups
were retained outside Git. Head RGB, depth and both wrists all reported Online
with empty capture errors. A short counter-based observation measured 30.0 fps
for head RGB, 29.6 fps for depth, and 59.3 fps per wrist at 1280×720; HTTP wrist
previews were 960×540. These are a short live observation, not a sustained
throughput guarantee. Browser verification displayed all four real images and
both tactile viewers through the one platform port, with no off-origin module
requests or JavaScript errors. No motor controls were operated.


## NiceGUI/aiohttp migration acceptance — 2026-10-31

The platform dashboard now uses NiceGUI 3.18.0 on FastAPI/Uvicorn. Its HTTP,
streaming and WebSocket proxy uses aiohttp 3.14.5. The optional `platform` extra
keeps core imports passive. Source validation ran 212 Python tests with two
optional checks skipped. A fresh core-only wheel installation outside the
checkout ran 212 tests with 37 optional checks skipped; installing the platform
extra then passed all 45 platform tests. Package builds and the locked dependency
check passed. Coverage includes same-origin framework polling and WebSockets,
request cancellation while switching services, binding before child startup,
and preserving cleanup failures when Uvicorn re-raises a shutdown signal.

A Chromium trial using actual NiceGUI and Viser with software control fixtures
verified button events over WebSocket, retained controller iframe identity when
visiting tactile/video, both nested tactile viewers, four video images, and
confirmed control/teleoperation switches in both directions. All 732 observed
HTTP requests and six WebSockets used the one gateway address, with no page
JavaScript errors. A separate trial stopped that mock platform while controller
and tactile connections were open: every owned service port closed within the
0.45-second observation, with no ASGI exception trace. This timing is specific
to software fixtures, not a claim about physical motor shutdown duration.

After replacement of the local entry process, a read-only browser visit displayed
both real tactile viewers and all four real camera images through port 8888.
All 356 observed HTTP requests and three WebSockets stayed on that origin, with
no page JavaScript errors. Short counter observations measured 30.0 fps for head
RGB and depth, and 59.4 fps per wrist at 1280×720. Both tactile pages reported
about 59.7 Hz. No motor movement was commanded. Existing external control-service
ownership and the earlier physical-teleoperation validation limits still apply.
