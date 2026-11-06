# OpenArm Runtime

A small Python toolkit for **OpenArm follower control, Feetech leader readout,
RGB/depth video capture, Paxini tactile sensing, and optional browser interfaces**. Use the Python API in your own application, or
the CLI for live inspection and user-operated control.

Python 3.12 · MIT · Linux SocketCAN for followers · Independent community project

## Policy execution and gravity compensation

The platform also provides **数采可视化** at `/#collection`: recording state,
stream updates and writer queues, then camera/tactile/joint playback from native
collection archives. Recording controls remain in the existing teleoperation
page. See [collection visualization](docs/platform.md#collection-visualization).

The platform includes **策略可视化** to the platform at `/#policy`
(standalone same-origin page: `/policy`). It displays submitted targets and
recorded feedback separately, inference latency and a joint timeline. Built-in
simulated policies and episode replay work in `--attach-only` mode. This page
uses the version 1 policy contracts; it does not launch the Classic motor backend.
See [platform policy visualization](docs/platform.md#policy-visualization).

Operator-driven [teleoperation data collection](docs/collection.md) adds explicit
episode recording/review, native-time robot/camera/tactile archives, quality checks
and offline aligned/LeRobot export. Enable it with `leader --follow
--collection-profile /path/to/collection.json`; rehearse without hardware using
`examples/collect_simulated.py`.

Optional [physical gravity compensation](docs/gravity.md) adds shared model-based
feedforward to hold, teleoperation and policy control, with shadow diagnostics and
per-machine configuration. Install the `gravity` extra; active physical validation
is separate from the offline tests.

`openarm_policy` connects model adapters to timestamped robot/video/tactile
observations and single-step or chunked joint actions. Inference runs in a separate
process; Classic CAN control retains independent workers, deadlines and disable
cleanup. Model loading and preprocessing live in optional adapters, separate from
hardware transport; trained weights and datasets remain external assets.

```sh
openarm-runtime policy --robot simulate --duration 3
```

See [policy contracts, adapters, recording and physical execution](docs/policy.md).
For reusable checkpoint/input/output plugins and the initial pure-visual ACT
upright integration, see [model adapters and deployment](docs/model-adapters.md).
Cartesian policy execution is an explicit geometry-declared capability, not joint-vector
relabeling; trained-policy hardware acceptance remains separate.
An optional fixed preparation can connect from actual feedback, move to task-ready,
open the gripper and hand off to the model in the same operator-started episode.
Shared bilateral gravity configuration and per-arm feedforward apply beneath all
these modes; see [gravity configuration](docs/gravity.md#bilateral-deployment-configuration).
Offline tests and an operator-authorized right-arm policy/gravity trial passed.
See [measured results and remaining validation boundaries](docs/gravity.md#physical-right-arm-trials).

**[中文使用文档 / Wiki](https://github.com/TCXM/openarm-runtime/wiki/Home)** · [常用命令](https://github.com/TCXM/openarm-runtime/wiki/CLI-Reference) · [Python API](https://github.com/TCXM/openarm-runtime/wiki/Python-API) · [Mac / Windows 远程访问](https://github.com/TCXM/openarm-runtime/wiki/Remote-Access)

## Install only what you need

Create a Python 3.12 virtual environment, then install from a release tag:

```sh
python3.12 -m venv .venv
source .venv/bin/activate

# Core: state types, software mapping and control API; no GUI or hardware SDK.
pip install "openarm-runtime @ git+https://github.com/Edge-Intelligence-Lab/openarm-runtime.git@v0.8.1"

# Serial leader readout, with a browser-based virtual robot.
pip install "openarm-runtime[leader,viewer] @ git+https://github.com/Edge-Intelligence-Lab/openarm-runtime.git@v0.8.1"
```

Extras compose freely:
- `leader`: Feetech SDK + PySerial for STS3215 buses.
- `viewer`: Viser + URDF/mesh loading.
- `tactile`: pinned community Paxini SDK + PySerial; combine with `viewer` for visualization.
- `video`: OpenCV headless + NumPy + Flask; independent of Viser, leaders and CAN.
- `platform`: NiceGUI/FastAPI dashboard and aiohttp gateway.
- `follower`: official OpenArm CAN SDK 1.4.1 and its build dependencies.

The follower extra requires a Linux machine and the matching native C++ SDK.
Follow [workstation setup](docs/workstation.md) before installing it. The core
and leader APIs import without Viser; importing a module never opens hardware.
Distribution is through GitHub tags and release artifacts, not PyPI.

## Using the package vs developing it

**Use a released version in an existing project:** no manual clone is needed.
The installer fetches the tagged source and installs the package into your
project's environment. With uv, declare and lock the dependency directly:

```sh
# Run in your own Python 3.12 project.
uv add "openarm-runtime[leader,viewer] @ git+https://github.com/Edge-Intelligence-Lab/openarm-runtime.git@v0.8.1"
```

**Modify the runtime while developing your application:** clone the source
anywhere you like, then install it in editable mode into your application's
activated Python 3.12 environment:

```sh
git clone https://github.com/TCXM/openarm-runtime.git /path/to/openarm-runtime
python -m pip install -e "/path/to/openarm-runtime[leader,viewer]"
```

Replace `/path/to/openarm-runtime` with your chosen directory. It does not need
to be inside or next to your application's repository. Editable installation
makes imports use that checkout, so Python source edits take effect when you
restart your application; you do not need to reinstall for each edit. Keep the
checkout in place while using it. Dependency, entry-point or build changes may
require reinstalling; native SDK changes require rebuilding the SDK.

For a uv-managed application, use a local editable dependency so later `uv sync`
commands preserve your development setup:

```sh
uv add --editable "/path/to/openarm-runtime[leader,viewer]"
```

This records a local source dependency. Before sharing or deploying the
application, replace it with a published tag or commit using the versioned
`uv add` command above and commit the updated dependency files. The `follower`
extra still requires [native SDK setup](docs/workstation.md) in either mode.

To develop the runtime on its own, use `uv sync --locked` in this repository;
see [contributing](CONTRIBUTING.md).

## Use all modules through one address

The **OpenArm Platform** combines motor control, teleoperation, tactile and
vision at **http://127.0.0.1:8888**. NiceGUI provides the dashboard; its
FastAPI/Uvicorn server and aiohttp proxy carry module pages and real-time data
through that address. Available from v0.6.0, it replaces separate module
launches with one entry point. Install the optional platform dependencies,
configure the machine once using [platform.example.json](platform.example.json),
then launch:

```sh
python -m pip install "openarm-runtime[platform] @ git+https://github.com/Edge-Intelligence-Lab/openarm-runtime.git@v0.8.1"
openarm-runtime platform --profile ~/.config/openarm-runtime/platform.json
```

Install each configured module's extras as well (`leader,viewer,follower`,
`tactile,viewer`, and/or `video`); the platform extra does not install hardware
SDKs. In a configured development checkout, `python3 run_platform.py` selects
the project's environment automatically. For another computer, only one SSH
forward is needed, including camera images and all Viser connections:

```sh
ssh -N -L 8888:127.0.0.1:8888 USER@WORKSTATION
```

Control and teleoperation switch as mutually exclusive modes from the same
sidebar; tactile and vision can stay connected alongside them. Startup never
enables a motor or starts following. Stop the old independent controllers using
their normal procedure before replacing their launches with the platform.
Existing services can be reused but remain externally owned.

See [platform setup](docs/platform.md) for configuration and migration. The
individual commands below remain available for module diagnostics and API use;
normal platform use does not require launching them separately.

## Read a leader in Python

```python
from openarm_leader import LeaderBus

port = "/dev/serial/by-id/YOUR_LEADER_ADAPTER"
with LeaderBus(port) as leader:
    sample = leader.sample()
    print(sample.positions)  # Eight encoder readings, ordered by servo ID 1–8.
    print(sample.voltages)   # Volts; alarms remain available in sample.errors.
```

For continuous background reads, use `LeaderReader`. For calibrated joint angles
and normalized gripper opening, use `Mapping`. See the [API guide](docs/api.md).

Runnable examples accept device paths rather than assuming a machine layout:

```sh
git clone https://github.com/TCXM/openarm-runtime.git
cd openarm-runtime
python examples/read_leader.py --port /dev/serial/by-id/YOUR_LEADER_ADAPTER
python examples/read_follower.py --interface can0
```

Both examples only read hardware. Press Ctrl+C to close the connection.

## Open the interfaces

Prepare the official model cache once (downloads assets outside the repository):

```sh
export MODEL_ROOT="$HOME/.cache/openarm-models"
openarm-runtime prepare --asset-root "$MODEL_ROOT"
```

**Leader → virtual robot:** move a physical leader and inspect its mapped pose.

The panel provides manual calibration: select left/right/both, let
the leader hang naturally and record its reference, capture open/closed gripper
endpoints, then save. Each capture checks 0.7 seconds of stationary feedback;
failed captures preserve the previous calibration. Saved settings load on restart.
Direction and device assignment are under advanced settings. The virtual model
shows joint changes relative to the reference, not the leader's physical pose.
This software reference is separate from follower takeover alignment;
the default page does not command physical followers or write leader servo registers.

### Physical following (v0.4.0)

Add global `--config /path/to/classic.json` before `leader`, and add `--follow
--report-root /external/reports` to its options. The page starts passive; choose
left/right/both and click **开始跟随** to take over the current poses. Saved, complete
leader calibration is required. Stop other follower viewers/panels first so
the controller can acquire the exclusive CAN lease.

The implementation follows one or both arms with direct MIT position targets,
without the manual panel's 5°/s slew or ±30° takeover envelope. Start and restart
sample fresh leader origins without modifying saved calibration. Gripper opening
changes map to a 65-degree motor span relative to takeover; this is not a measured
millimeter mapping. Targets clip to the side-specific joint ranges below.

Reference: [Enactic unilateral follower control](https://github.com/enactic/openarm_teleop/blob/eb2d49338bf70ace95282ea724903849397b7811/src/controller/control.cpp)
sends leader references to MIT control directly. [LeRobot OpenArm configuration](https://github.com/huggingface/lerobot/blob/408e0ca763658f9294ef816d2b07f7d281ddeeb0/src/lerobot/robots/openarm_follower/config_openarm_follower.py)
defaults `max_relative_target` to `None` and supplies these joint bounds (degrees):
J1 ±75, J2 left −90…9/right −9…90, J3 ±85, J4 0…135, J5 ±85,
J6 ±40, J7 ±80, gripper 65-degree span. Our calibrated left gripper opens
positive (0…65); the right opens negative (−65…0). This motor-coordinate
adaptation differs from LeRobot's normalized negative gripper bounds.

Neither source establishes a universal maximum physical speed. Runtime keeps
the existing MIT gains and Classic transport, and sends current references at
the configured control rate (20 Hz on the workstation). Tracking does not apply
the manual-hold 2 rad/s feedback cutoff or predicted-effort rejection; motor MIT
effort saturation still applies. Fault, temperature, finite-value and fresh-feedback
checks remain active. Manual slider control retains its original limits.

The main page expands following controls; calibration and advanced diagnostics
are collapsed by default and can be expanded when needed.

Voltage-only leader alarms on fresh valid feedback are informational and do not
interrupt following (operator-approved); their voltage remains visible in diagnostics.
Stop, owning-browser disconnect, stale leader feedback, other/combined leader
alarms, and control failures terminate the session and attempt
verified whole-arm disable. Support the arm when stopping. The virtual model
continues to show the leader mapping, not measured follower geometry. Runtime
reports are diagnostic samples, not a synchronized training dataset. Simultaneous
dual-arm following uses one worker per CAN bus: both preflights finish before
either enables, each arm remains serviced while its peer ramps, and both leader
origins are captured when both holds are ready. A fault on either side cancels
both loops and each arm verifies disable independently. Report writes are serialized.
An operator completed a short dual-arm trial and confirmed usable following;
both eight-motor disables were verified. Quantified tracking accuracy and
long-duration robustness remain unmeasured; see [validation](docs/validation.md).
This mode reads serial buses only; it does not command follower hardware.

```sh
openarm-runtime leader \
  --left-port /dev/serial/by-id/LEFT_ADAPTER \
  --right-port /dev/serial/by-id/RIGHT_ADAPTER \
  --calibration "$HOME/.config/openarm/leader.json" \
  --asset-root "$MODEL_ROOT" --port 8890
```

**Follower:** copy [classic.example.json](classic.example.json) to your own config
directory, set the CAN interfaces, and confirm their physical left/right mapping.
The host links must already be configured for Classic CAN at 1 Mbps.

```sh
export MACHINE_CONFIG="$HOME/.config/openarm/classic.json"
openarm-runtime --config "$MACHINE_CONFIG" check
openarm-runtime --config "$MACHINE_CONFIG" view --asset-root "$MODEL_ROOT"
openarm-runtime --config "$MACHINE_CONFIG" panel \
  --asset-root "$MODEL_ROOT" --report-root "$HOME/openarm-reports" --port 8891
```

Run `view` and `panel` separately. `check` and `view` are read-only. The control
panel starts read-only; clicking enable holds the selected arm's current pose,
then sliders set relative targets. Stop attempts to disable the complete arm
and verifies feedback. UI labels are currently Chinese.

Servers bind to loopback. For a remote robot, forward ports with SSH and open
`http://127.0.0.1:8890` (leader) or `http://127.0.0.1:8891` (follower) in Chrome:

```sh
ssh -N -L 8890:127.0.0.1:8890 -L 8891:127.0.0.1:8891 your-workstation
```

## Video capture and the four-view browser

Video is available starting with v0.3.0:

```sh
python -m pip install "openarm-runtime[video] @ git+https://github.com/Edge-Intelligence-Lab/openarm-runtime.git@v0.8.1"
# No cameras or vendor SDK required; clearly labelled SIMULATION.
openarm-runtime video --simulate --port 5004
```

For hardware, copy [video.example.json](video.example.json) outside the repository
and replace its placeholder device paths. USB `by-id` paths or stable udev aliases
are preferable to `/dev/videoN`. The Astra Pro depth source needs an external
Orbbec OpenNI2 SDK and our compiled helper; see [video setup](docs/video.md).

```sh
openarm-runtime video --profile ~/.config/openarm/video.json --port 5003
# On your client computer:
ssh -N -L 5003:127.0.0.1:5003 your-workstation
```

Open `http://127.0.0.1:5003`. One process owns all configured cameras and the web
server; no separate RGB/depth servers are required. Stop other applications that
own these devices before starting hardware capture. Packaging tests do not stop
or replace already-running services.

The white/blue interface includes simultaneous views, single-view/fullscreen,
mirror, pause, per-camera status, depth colour range and raw 16-bit depth download.
Each browser request returns one JPEG/PNG, avoiding Safari's MJPEG Fetch failures
and long-connection limits. Images are independent observations, not synchronized
RGB-D pairs or calibrated registration.

The library works without a browser:

```python
import time
from openarm_video import CameraConfig, VideoCapture

configs = [CameraConfig("head", device="/dev/v4l/by-id/YOUR_CAMERA-video-index0")]
with VideoCapture(configs) as capture:
    while True:
        frame, error, count = capture.snapshot("head")
        if frame is not None and not error and frame.fresh(time.monotonic()):
            print(frame.image.shape, frame.pixel_format, count)  # NumPy BGR image.
        time.sleep(0.1)
```

Use `CameraReader` for one source, `OpenCVCamera`/`OpenNIDepthCamera` for synchronous
reads, and `create_app(capture)` to embed the UI in an application that already
owns capture. Importing `openarm_video` loads no OpenCV, Flask or vendor SDK and
opens no hardware. [Video API](docs/api.md#video) · [setup](docs/video.md) ·
[runnable example](examples/read_video.py).

## Integrate without a browser

The public API consists of:

- `openarm_video`: `CameraConfig`, `Frame`, `CameraReader`, `VideoCapture`,
  `OpenCVCamera`, `OpenNIDepthCamera`, `load_cameras`, `build_depth_helper`,
  `encode_frame`, `create_app`, `serve`.
- `openarm_leader`: `LeaderBus`, `LeaderReader`, `Sample`, `Mapping`.
- `openarm_runtime`: `Profile`, `hold_arm`, `calibrate_arm`, `ArmConnection`, `MotorState`, `MotorRange`.

`hold_arm(profile, side)` owns preflight, bus locking, enable and exit cleanup.
Your application owns the control loop and supplies relative targets to
`arm.tick()`. Entering the context enables the physical arm. The
[API guide](docs/api.md) explains units, scheduling and a complete hold-only loop.

Configuration, calibration, assets and logs are explicit paths owned by the caller.
No research repository, sibling checkout, or running web server is required.

## Automatic follower calibration

Select an arm and **自动标定** in the panel to perform single-arm
stop-based calibration and hardware zero writes over Classic CAN. The same
operation is available through `calibrate --side left|right --report-root PATH`
and `calibrate_arm()`. Both arms completed zero writes and one power-cycle
readback test on the development rig; quantified accuracy and repeatability are
not yet established. Read the [operator guide](docs/calibration.md) for the
required starting pose, motion sequence, stop behavior and partial-write recovery.

The motor-control panel also provides **自动标定（不写零位）**, available
as `calibrate --no-write-zero` or `calibrate_arm(..., write_zero=False)`. It runs
the same stop-seeking motion and disables at completion, preserving encoder zeros.
The final pose need not read as zero in the existing encoder coordinates.

## Scope

The working baseline covers eight motors per follower arm, Classic CAN,
whole-arm hold/relative targets, two STS3215 leader buses and virtual pose mapping.
The manual-control panel provides calibrated absolute J1–J7 angle sliders
and a ±30° relative gripper offset, with 0.1–5°/s target speed and 10°/s²
acceleration smoothing. Its fixed gains, feedback checks, queued reports and
read-only CAN reconnect are retained. The default `hold_arm()` API keeps its
±30° relative offsets. See [validation](docs/validation.md) for measured evidence.

Long-duration dual-arm acceptance, collision avoidance, quantified
calibration accuracy and long-term reliability are future work. Firmware encoding
ranges are not mechanical joint limits. Software cleanup cannot replace a physical
stop mechanism when a process or connection fails.

## Develop

```sh
uv sync --locked
uv run --locked python -m unittest discover -s tests
uv build
```

Offline tests require no hardware. The official-model test additionally needs
the `viewer` extra and `OPENARM_VIEWER_ASSET_ROOT`. CI tests the built wheel outside
the source tree on Linux and macOS. See [CONTRIBUTING.md](CONTRIBUTING.md) for the
release workflow and compatibility policy.

Implementation lives in `src/`, regression tests in `tests/`, and small API usage
examples in `examples/`. [Changelog](CHANGELOG.md) · [Provenance](ORIGIN.md) ·
[Third-party notices](THIRD_PARTY.md) · [MIT license](LICENSE)

## Tactile sensing

Paxini IP-M2324-Core (68 points) with the single-channel USB serial converter:

```sh
pip install "openarm-runtime[tactile,viewer] @ git+https://github.com/Edge-Intelligence-Lab/openarm-runtime.git@v0.8.1"
openarm-runtime tactile --device /dev/serial/by-id/YOUR_CONVERTER --device-id 2 --port 8891
```

Use `--right-device /dev/serial/by-id/SECOND_CONVERTER` for two sensors on one page.
The workstation display identifies both sensors as belonging to the left arm;
left/right refer to their positions, not two robot arms. The right arm has no tactile sensor.
Use `--simulate` instead of `--device` for offline visualization. Set the device ID
from your sensor configuration; the validated units use 2. The viewer shows raw
force counts, 3D taxel positions and force vectors; it does not calibrate hardware.

For capture without a viewer, install only `[tactile]`:

```python
import time
from openarm_tactile import TactileReader

with TactileReader("/dev/serial/by-id/YOUR_CONVERTER", device_id=2) as reader:
    for _ in range(100):
        snapshot = reader.snapshot()
        if snapshot["state"] == "live":
            print(snapshot["sample"].resultant)  # XYZ raw counts, not newtons.
        time.sleep(0.1)
```

For a configured development checkout, `python3 tactile.py` uses `.venv` and the
external mapping in `~/.config/openarm-runtime/tactile.json`. This convenience
script is not needed when using the installed CLI. See [tactile setup](docs/tactile.md)
and the [Python API guide](docs/api.md#tactile).
