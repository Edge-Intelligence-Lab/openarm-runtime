# PICO to Classic right arm

This optional Runtime command serves one HTTPS WebXR page for the Classic
right arm. It uses the same Runtime CAN, joint limits and gravity code as
the other hardware entry points. The lab's earlier dora trial used Enactic
mink IK, while the command in this repository uses Runtime's
`DifferentialIK`. The existing simulation workspace is independent.

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

Create a private token file and use a TLS certificate trusted by the
PICO browser, with a SAN matching the address you will open.

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

The head camera is optional. The lab rig previously needed an explicit
MIT trial for motors with unreadable RID10. Use `--allow-unreadable-mode`
only for that verified setup. A readable non-MIT mode always fails preflight. Open the HTTPS address in PICO Browser,
enter the token, release the right grip and press **Start**. Hold the
right grip for Cartesian following. Release it to hold position. While holding the grip, use the right stick Y to adjust the aperture or
hold the trigger to close. Exit WebXR to end the
session and disable. To follow again, enter a new session with **Start**.

## Data path

The browser sends tracked six-DoF right-hand poses. The server opens
only one authorized session and waits for valid input before entering
the Classic session. A grip press anchors the current measured TCP.
Controller translation and rotation become a TCP target via Runtime's
PICO mapping and `cartesian_start`. That helper uses the configured
`KinematicChain`, model/physical joint bounds and `DifferentialIK`.
The bridge bounds command lead against current motor feedback.

The Classic session opens the existing exclusive bus lease, checks
the arm and reads `Profile.gravity["right"]` through
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

**Operator trial on the lab's preceding dora deployment.** The operator
verified right-arm following during PICO use and reported normal
shoulder response after enabling full Runtime gravity feedforward.
A separate recorded run confirmed verified right-arm disable after a UI
stop, and watchdog disable when IK targets stopped arriving. The later
single-entry dora flow was deployed with both arms disabled before Start.
These observations cover that deployment. The new Runtime launcher,
WebXR camera rendering and disable on immersive exit still need an
operator-run hardware repeat. Record J1/J2 target and feedback, the
`gravity_config` event, and the verified "arm_disable" event when running it.
