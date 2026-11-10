# OpenArm Platform

`openarm-runtime platform` brings motor control, teleoperation, tactile and
vision, collection and policy visualization into one local workspace at **http://127.0.0.1:8888**. Use the sidebar to
open each module; all pages, images and WebSocket connections pass through this
one address. The dashboard uses NiceGUI components on FastAPI/Uvicorn, and
aiohttp handles upstream HTTP, streaming responses and WebSocket connections.
Viser continues to provide the robot and tactile 3D views. The existing service
manager coordinates hardware ownership and shutdown.

## Local workstation

The platform is available from v0.6.0. In a Python 3.12 environment, install
the optional platform dependencies and configure the machine as described below:

```sh
python -m pip install "openarm-runtime[platform] @ git+https://github.com/Edge-Intelligence-Lab/openarm-runtime.git@v0.8.1"
openarm-runtime platform --profile ~/.config/openarm-runtime/platform.json
```

For development from a checkout:

```sh
cd /path/to/openarm-runtime
# Run with this checkout's activated Python 3.12 environment.
python -m pip install -e ".[platform]"
python3 run_platform.py
```

Open http://127.0.0.1:8888 . The development launcher uses this checkout's `.venv` and
`~/.config/openarm-runtime/platform.json`. It starts configured missing services,
reuses existing listeners and leaves reused processes alone when it exits.
Ctrl+C closes only services started by this platform. Child output is in
`~/.local/state/openarm-platform/` (`control.log`, `teleop.log`, etc.).

When using a Mac browser over SSH, keep this command running on the Mac:

```sh
ssh -N -L 8888:127.0.0.1:8888 USER@WORKSTATION
```

Then open http://127.0.0.1:8888 on that computer. No module or tactile viewer
port needs a separate forward. The interface generates this command under
connection settings. If local port 8888 is already occupied, use a different
local port, for example `-L 8889:127.0.0.1:8888`, then open local port 8889.
The platform's `--port` option changes the workstation entry port.

## Replace separate launches

### Trusted LAN access (explicit opt-in)

The default entry remains loopback-only. To open the same unified page from a
laptop on a trusted LAN, add the workstation's **actual assigned private IPv4**
and an explicit client allowlist to the external platform profile, for example:

```json
{"host":"192.168.1.20","allowed_clients":["192.168.1.10/32"]}
```

Merge these fields into the existing profile; do not replace its services or
evidence paths. The example permits one laptop. A trusted `/24` subnet may be
configured instead; broader allowlists, public bind addresses and
arbitrary hostnames are rejected. The LAN address is printed at startup. Restart
the same `run_platform.py` process after editing configuration; no system service
or separate camera launch is required. An address change requires updating the
profile. Internal camera/control ports remain loopback-only behind the gateway.

To explicitly listen on **all IPv4 interfaces without any source-IP restriction**:

```json
{"host":"0.0.0.0","allowed_clients":[]}
```

This is a separate, unrestricted-source opt-in. It admits LAN, VPN and publicly
routed clients wherever the workstation is reachable; there is no application
login. OS firewalls and routers are not changed. Open the workstation's actual
IP in the browser, not `0.0.0.0`. Normal direct access via any IPv4 address is
accepted. Exact same-origin checks and IP-literal Host validation remain to
block cross-site requests and DNS rebinding; they are not client-IP allowlists.
SSH-forward localhost origins remain supported. The default stays loopback-only.

This is **not authentication**: permitted network clients can access the operator
console. Use only a trusted wired/private LAN; do not forward this port to the
Internet or expose it on public/shared Wi-Fi. When configured, the peer filter reads the actual
socket address, not `X-Forwarded-For`. Host checks reject unconfigured names and
wrong LAN ports; mutation and WebSocket requests still require exact same origin.
This entry uses HTTP, not encryption. Prefer the existing SSH tunnel when the
network is untrusted. Never disable origin checks or expose the internal ports
as a workaround. With a LAN bind, an SSH forward must target that bind address
rather than remote loopback; the dashboard generates the appropriate target.

1. Stop the old standalone control, teleoperation and sensing processes that
   this platform will manage, using their normal stop procedures.
2. Copy and configure the example below once, then start only the platform.
3. Use the platform address as the browser bookmark and retain one SSH forward.

Existing listeners can also be reused, but remain externally owned: the
platform will not stop them, and cannot switch away from an externally owned
motor controller. Stopping the old controller lets the platform manage both
control modes through the same interface. Opening the platform never enables
motors or starts following.

The old CLI commands remain available for Python/API users and individual module
diagnostics. They do not need to be started separately for normal platform use.

## Configuration

Copy [platform.example.json](../platform.example.json) outside the repository,
then set the local model, profile, calibration and device paths. Commands are
argument arrays executed without a shell. `{python}` expands to the platform's
Python interpreter; `~/` arguments expand to the current user's home. `cwd` and
`log_dir` are resolved relative to the configuration file. No other shell
expansion occurs. Startup never fabricates missing calibration.

The top-level `port` is the only browser-facing port. Each of
`services.control`, `services.teleop`, `services.tactile` and `services.vision`
can specify `port`, `command`, `cwd` and `extra_ports`. These service ports are
internal loopback endpoints used by the gateway. Keep them distinct and match
each command's actual listening port. If a command is omitted, that endpoint
is available only when externally started.

Tactile defaults to its page port plus 10/11 for its two nested viewers; set
`extra_ports: []` for a single-sensor page. Both nested viewers are proxied by
the platform too. The sample keeps each module in its own process so its
hardware lifecycle and existing cleanup behavior remain intact.

Optional `support` entries use IDs such as `aux_rgb` or `aux_depth` and the same
configuration shape. They are managed as background dependencies but do not
create navigation tabs. This lets a workstation retain an existing vision
page and its capture services. Other machines can use the package's single
`video` command as shown in the example.

The optional `[platform]` extra installs NiceGUI and aiohttp. Configuration
loading, service management, core imports and CLI help remain usable without
these dependencies; starting the platform requires them. Each child service
needs its own dependencies: `[leader,viewer,follower]`, `[tactile,viewer]`,
or `[video]`. Install the combination used by your configured commands.
See [workstation setup](workstation.md) for the native CAN SDK, and run
`openarm-runtime prepare --asset-root ~/.cache/openarm-models` once for the
verified official model assets.

Installed-package equivalent:

```sh
openarm-runtime platform --profile ~/.config/openarm-runtime/platform.json
# Display existing endpoints without starting/stopping processes:
openarm-runtime platform --profile ~/.config/openarm-runtime/platform.json --attach-only
```

Without `--profile`, the CLI provides only the default endpoint mapping. It does
not infer launch commands or discover arbitrary running hardware services.

## Server lifecycle

Use the platform CLI with one server process and no development reloader. The
ASGI lifespan owns the shared aiohttp client session and the service manager;
startup does not enable motors. Blocking process selection and shutdown run
outside the event loop so camera and WebSocket traffic can keep flowing.
Shutdown closes proxy connections and the services owned by this platform.

Do not run multiple Uvicorn workers or automatic reload against a hardware
configuration: each process would otherwise create its own service lifecycle.
Configuration paths, backend ports and the public CLI are unchanged by the
framework migration. Existing profiles can be used after installing `[platform]`.

## Navigation and service ownership

- Overview shows listener reachability, not sensor health or motor state. Each
  module remains the source of hardware status. A disconnected sensor can have
  a reachable page service; read its device status before operating it.
- Each browser has its own NiceGUI dashboard. Module frames load when selected
  and remain mounted while hidden; sensing navigation retains their WebSocket
  connections and the current controller view. Selecting a sensing page does not stop motion;
  use the original page's stop control. Explicit refresh retains its normal
  disconnect behavior and asks for confirmation.
- Motor control and teleoperation form one mutually exclusive group. The control
  page holds read access to CAN; physical following requires exclusive access.
  Startup chooses an existing member, or starts the configured control page.
  Selecting the other mode stops the owned previous service through its original
  shutdown path, then starts the selected service. The old frame is discarded;
  its replacement is loaded when ready. No enable or follow action is automatic.
- Existing services remain externally owned. If an independent control page is
  running, switching to managed teleoperation is refused until its operator
  stops it. The platform never terminates an externally started process.
- Existing CAN locks are per Linux user, so they cannot exclude another user's
  controller. This platform coordinates its configured control/teleop endpoints;
  it cannot detect every other controller or port on a shared machine.
- Owned motion services have 40 seconds to stop cleanly. A timeout or abnormal
  exit blocks switching and is reported; motion processes are never forcibly
  killed. Other owned service groups have bounded terminate/kill cleanup. No
  service automatically restarts after failure.

The server binds `127.0.0.1` by default; the explicit trusted-LAN option above
changes only the unified entry address. The gateway routes only to configured local
services. REST selections require same-origin JSON; dashboard actions use the
NiceGUI session. Upstream mutation and WebSocket requests are restricted to the
platform origin. Commands and private paths are not exposed by the public
config API. Module pages now share the platform origin; browser preferences saved on the old
separate-port pages may need to be set again. Machine configuration, calibration,
log files and SDK/model artifacts stay outside version control.

The HTTP/WebSocket boundary accepts the exact `openchamber-preview.localhost`
hostname used by OpenChamber desktop previews, alongside `localhost`, `127.0.0.1`
and `::1`. This permits its authenticated development tunnel (including paired
remote connections) without another browser-side SSH forward. Other `.localhost`
names and lookalike domains remain rejected. Requests with an Origin must still
match the full HTTP origin, including its port; mutations and WebSockets require
that Origin. This does not change the loopback bind or start any hardware service.

## Single-port routes

These paths are useful for diagnostics; normal navigation stays in the platform.

| Module | Gateway path |
| --- | --- |
| Motor control | `/apps/control/` |
| Teleoperation | `/apps/teleop/` |
| Tactile | `/apps/tactile/` |
| Vision | `/apps/vision/` |

Tactile's nested viewers use `/apps/tactile/_extra/0/` and
`/apps/tactile/_extra/1/`. HTTP assets, camera responses and Viser WebSockets
are forwarded through these paths without requiring client access to the
internal ports. `/api/status` reports page service availability; it does not
report synchronized hardware data or certify that a motor is disabled.

## Head camera calibration

Open **头部相机标定** at `/#camera_calibration` or `/camera-calibration`.
Install the optional `[platform,calibration]` dependencies. The built-in page
borrows native lossless frames from the existing vision owner; it never opens a
device, starts a service or moves the robot. Supply actual board parameters and
the camera's external `calibration_identity`. An optional `calibration_root` in
the platform JSON sets the external evidence directory (default
`~/.local/share/openarm-calibration`); relative paths resolve beside that JSON.
No path or upstream URL can be supplied from the browser.

The workflow implements ChArUco and ordinary-checkerboard pinhole intrinsics, predeclared held-out
image checks, bounded offline head-extrinsic JSON import and explicit immutable
candidate saving. Real-time extrinsic capture is disabled because trusted
exposure-time/FK pairing is unavailable. Saving does not install a machine
profile. See [camera calibration](camera-calibration.md) for the workflow,
limits, cancellation/ownership and the distinction between image checks and
physical verification.

## Policy visualization

The platform includes an internal **策略可视化** module at
`http://127.0.0.1:8888/#policy`; `/policy` opens it without the dashboard shell.
It uses the same NiceGUI server and needs no extra port or device service.
Existing platform profiles continue to work. The page is also available in
`--attach-only` mode and does not switch or stop a hardware module.

- Start the built-in J1 sine or hold-pose simulation (1–60 seconds). The page
  runs a `PolicySession`, process-isolated inference and `SimulatedRobot`.
  Simulation is an ideal position plant, not measured hardware performance.
- Choose a joint to compare submitted targets with timestamped feedback
  observations. The page also shows the latest inference latency and command
  count. After the session, play or drag the timeline to inspect joint values.
- Open a CLI recording from a child directory of `~/openarm-episodes`. Set the
  optional top-level `policy_record_root` in the external platform JSON to use
  another location. Relative paths resolve beside that JSON file. The loader
  reads `events.jsonl` and only robot-state array sidecars, never image arrays
  or policy plugins. Recordings with no ending event are marked incomplete.

The view retains the latest 2,000 sampling events and limits recording files
to 64 MiB. Curves are sampled to at most 701 points per series for rendering;
the timeline table uses the retained samples. Recorded feedback is historical,
not the current robot pose. Replay does not resubmit commands to hardware.

One simulated session is shared by all browser views. Closing a tab leaves the
bounded simulation running; Stop or server shutdown cancels it and closes the
inference worker. Physical policy execution remains an explicit CLI/API operation
as documented in [policy.md](policy.md).

## Collection visualization

Open **数采可视化** at `/#collection`, or `/collection` for the standalone page
on the same server. No additional device process or port is created. The page
also works in `--attach-only` mode. “进入遥操录制” selects the existing teleoperation
view through the dashboard; starting/stopping/reviewing an episode stays with
that view's owner and the original control lifecycle.

Set `collection_root` in the external platform JSON to the raw collection root.
If omitted, the platform reads `root` from the teleop command's
`--collection-profile` file when available, otherwise uses `~/openarm-collection`.
The viewer never starts acquisition to discover a directory.

Every `Collection` instance publishes a small status snapshot under
`ROOT/.runtime/INSTANCE.json` every 0.5 seconds, in a separate best-effort thread.
It includes state, task/operator, duration, stream sequence/age and pending
writer queue size. Publishers never serialize camera images or perform I/O on
the CAN thread. Preview write failure does not stop raw acquisition; snapshots
older than three seconds are displayed as stale rather than current recording
state. Existing v0.8.0 processes need to restart on v0.8.1 to publish status.

Select an existing episode to view its task, acceptance and quality issues, then
play or scrub its native timeline. Camera and tactile samples are selected only
from timestamps at or before the cursor. Target dispatch and motor feedback
receipt timestamps remain separate. Camera images are JPEG previews of the
original lossless arrays; depth previews use a fixed 0–4 m colour scale. Tactile
XYZ values remain raw counts. Inspecting or playing a record never writes review
files, changes acceptance, exports training data, or resubmits motor commands.

Preview reads at most 128 MiB of journal data and retains the latest 2,000 records
per stream. Array previews require the optional `video` extra. The common retained
window is shown on the timeline; array and journal checksum issues remain visible.
Metadata/status inspection needs no camera, tactile SDK or native CAN SDK.
