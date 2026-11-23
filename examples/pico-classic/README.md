# PICO to Classic arms

This optional Runtime command serves one HTTPS WebXR page for Classic
arm teleoperation. It uses Runtime's CAN transport, joint limits,
gravity compensation and `DifferentialIK`.

## Install and configure

Use Python 3.12 on a Linux workstation with the verified Classic SocketCAN
and SDK setup. From a clean checkout:

```sh
uv sync --extra pico-classic --extra follower
uv run openarm-runtime pico-classic --smoke
```

Copy the two JSON examples to a machine directory outside the checkout:

- `classic.example.json` identifies the CAN interfaces and points to the
  machine's existing **active** right-arm gravity profile. Use verified
  motor IDs and encoder zeros, then set `mapping_verified` to true. See
  [gravity configuration](../../docs/gravity.md).
- `ik.example.json` is a starting point for the stock arm. Download the
  [pinned OpenArm URDF](https://raw.githubusercontent.com/enactic/openarm_description/4fe8e42bc4377914c0e800ac1739b1b897fcca3a/assets/robot/openarm_v2.0/urdf/example/v2.urdf)
  beside it as `openarm-v2.urdf`. The loader verifies its SHA-256.
  Confirm the ordered J1–J7 signs, offsets, locked joints and tool center
  point for the actual machine before enabling. The example TCP is a
  nominal stock closed-fingertip position. Configure actual payload and
  mounting gravity with the existing gravity profile.

For dual-arm control, pass a JSON object keyed by `"left"` and `"right"`, with
each entry containing that arm's verified IK settings. Each arm may set
`velocity_rad_s` (default 0.25, maximum 0.5). Enable active gravity for both
arms in the Classic profile and use a control rate of at least 20 Hz.

Use a TLS certificate trusted by the PICO browser, with a SAN matching the
workstation address. Choose a token file for password entry or
`--passwordless` on a private lab network.

```sh
umask 077
python3 -c 'import secrets; print(secrets.token_urlsafe(32))' > /path/to/session-token
uv run openarm-runtime --config /path/to/classic.json pico-classic \
  --ik-profile /path/to/ik.json \
  --cert /path/to/server.crt --key /path/to/server.key \
  --token-file /path/to/session-token \
  --host WORKSTATION_IP --port 8443 \
  --camera /dev/v4l/by-path/HEAD_RGB_DEVICE
```

The cameras are optional. Use `--left-wrist-camera` and `--right-wrist-camera`
with the corresponding stable V4L2 device paths to place wrist windows beside
the head view. Press right **A** to show or hide the windows. Press right **B**
to enter layout mode, point at a window and hold the trigger to drag it,
or move the thumbstick vertically to resize it. Press **B** again to leave
layout mode and release the grips before following again.

Use `--allow-unreadable-mode` for an installation verified to use MIT mode
when RID10 readback times out. A readable non-MIT mode fails preflight.
Open the HTTPS address in PICO Browser, enter the token if configured, look
straight ahead and press **Start** with both grips released. Hold either
controller's grip to follow on that arm and release it to hold position.
While gripping, use the stick Y axis to adjust the aperture or the trigger to
close. Exit WebXR to disable the arms. Bookmark the server's root URL. When
the server detects a newer control page it reloads in the same tab.

On a dual-arm page, exit WebXR and select **双臂回零** to return both arms
to their calibrated motor zeros. This uses the
[existing coordinated return](../../docs/natural-hanging-return.md), preserving
each gripper's entry position during motion and disabling both arms afterward.
Use **停止** to cancel. Confirm the return path is clear before starting.

## Data path

The browser sends tracked six-DoF hand poses. The server opens
only one authorized session and waits for valid input before entering
the Classic session. A grip press anchors the current measured TCP.
Controller translation and rotation use the same XR-to-robot basis as
simulation: forward, right and up map to robot +X, −Y and +Z. The page fixes
the control heading from the first valid horizontal headset direction when
entering WebXR. Look along the intended forward direction when starting.
Walking, turning your head or reconnecting keep the same control heading.
If WebXR recenters, the reset rotation preserves that heading when available.
Start a new session if the headset cannot report the reset rotation.
TCP targets use the configured `KinematicChain`, model/physical joint bounds and
`DifferentialIK`. Each arm limits joint speed and the command lead over measured
feedback. Within a grip, hand motion continues across the available workspace.
The controller caps how far the target leads the arm reference and discards
opposing backlog when the hand reverses. Loss of one controller stops its arm
independently.
While WebXR stays open, the browser retries interruption of the control
connection automatically. Each stopped arm waits for valid tracking and
released grip and trigger before it can start again from the measured pose.
An operator stop, WebXR exit or hardware fault ends control.

The solver first computes the hand-driven TCP step. When joint room remains,
it uses the redundant joint direction to prefer a moderately bent elbow and
centered wrist across clutches. A forward-kinematics check rejects corrections
that reverse the requested motion or move the TCP more than 0.1 mm or 0.046°
from the primary step. The correction shares the same step and tracking
limits as the primary command and fades in as the hand moves away from the
clutch anchor. At a joint or reach boundary, the primary step may be all
that is feasible.

The Classic session opens the existing exclusive bus lease, checks
the arm and reads its configured `Profile.gravity` entry through
Runtime’s `prepare_arm` and `prepare_gravity`. The launcher and session
both require active gravity. `ArmHold` adds the ramped gravity
torque to each MIT tracking exchange. Returning from the session invokes
the existing verified-disable cleanup. Invalid input, input loss, and
WebSocket close end the session, including during enable ramp-up.

## Verification

Hardware-free install and lifecycle tests:

```sh
uv sync --extra pico-classic
uv run --extra pico-classic openarm-runtime pico-classic --smoke
uv run --extra pico-classic python -m unittest discover -s tests -p test_pico_classic.py -v
```
