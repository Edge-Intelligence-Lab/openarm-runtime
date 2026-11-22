# MuJoCo simulation

Install the simulation extra with Python 3.12. From the repository root:

```sh
python -m pip install '.[simulation]'
openarm-runtime simulate --steps 100
openarm-runtime simulate --view
```

The headless command prints a JSON summary after stepping physics. Use
`--steps 0` to compile the scene and run forward kinematics only. The interactive
viewer requires a desktop graphics session and uses MuJoCo's actuator controls.
The bundled scene contains the OpenArm v2 dual-arm robot, floor, light and
overview camera, and a `calibrated_head` camera. Its geometry and control
parameters come from the pinned upstream model. The included head calibration
provides the head camera's intrinsics and pose relative to the robot base.
Select `calibrated_head` in the viewer to see its perspective. See [third-party notices](../THIRD_PARTY.md).

For a task scene, pass `--scene /path/to/scene.xml`. To reuse the exact robot,
copy the directory containing the packaged `base.xml` into your task workspace,
then edit the copy of `base.xml` and keep the sibling robot XML and its mesh
paths intact. Python callers can use
`openarm_simulation.standalone.base_scene_path()` to locate the installed XML.
Keep task objects, policies, datasets and project-specific camera calibrations
in your application or experiment workspace.

## Bundled camera parameters

The installed package includes the original head and wrist JSON from
`tacpilot/tacpilot@e6e979a1fa9bd3bc40cdeb78acb68f91d23d3759` under
`assets/simulation/cameras/{head,wrist}/calibration.json`. The runtime reads the
packaged copy directly.

```sh
openarm-runtime camera-parameters
openarm-runtime camera-parameters --camera wrist
openarm-runtime simulate --calibration /path/to/other-head.json --view
```

Python callers can use
`openarm_runtime.camera_parameters.load_camera_parameters(camera="head", path=None)`.
Pass a local JSON with the same schema to override the package default.
A custom scene uses its own cameras by default. To apply an explicit head
calibration with `--scene`, declare a fixed `arm_origin` site with the
calibration's robot base frame and pass `--calibration`. The scene must leave
the `calibrated_head` camera name free.

The head parameters describe the measured OpenArm V2 Astra Pro installation
at native 1280×720. The renderer uses the pinhole intrinsics, including
unequal focal lengths and the off-centre principal point. To compare with a
real frame, undistort it using the original OpenCV coefficients and intrinsics.
The wrist file provides one set of intrinsics and positions for the two wrists,
without complete rotational extrinsics. Physical use requires confirming the
camera, lens, mounting and image geometry or supplying a new calibration.
Reading these defaults does not open devices or register them on a workstation.

## Passive platform backend

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

## PICO 4 simulation teleoperation

Start the existing MuJoCo platform with its simulation model and profile; install
`.[platform,simulation,calibration,cartesian]`. The dashboard's **PICO 仿真遥操**
opens `/simulation/teleop`. The simulation page at `/simulation/practice` uses
the running scene and simulated arm.

Set `OPENARM_PICO_OFFICIAL_SIM_URL` to the HTTPS URL of a running
[Enactic WebXR + MuJoCo dataflow](https://docs.openarm.dev/tutorial/data-collection-webxr/)
to point the simulation entry at the official headset page. The headset must
also be able to reach the WebRTC media endpoint or its TURN relay.

The headset and the computer running MuJoCo need to be on a trusted LAN. Append
these fields to the existing **simulation platform** JSON (alongside
`simulation_profile`). Use the actual computer and headset IPv4 addresses.

```json
{
  "host": "192.168.1.20",
  "allowed_clients": ["192.168.1.21/32"],
  "tls_cert": "certs/platform.pem",
  "tls_key": "certs/platform-key.pem"
}
```

The paths are resolved relative to the platform JSON. Use a certificate with the
platform LAN IP in the subject alternative names and a chain trusted by the
headset browser. Keep the key outside version control. With this example and the
default port, open `https://192.168.1.20:8888/` in PICO Browser, select
**PICO 仿真遥操**, select **仿真** if prompted, then **进入 VR**. You can also open
`/simulation/practice` directly. The browser preview and immersive view show the overview as the
main view and the simulated head camera as a smaller view beside it.
Tap either controller's side grip to cycle the main view. Grasp practice adds a
task close-up as the second view in the cycle, between the overview and the
head camera. Its framing changes with the selected scene. In grasp practice,
hold the side grip for about 1.2 seconds to reset the arms
and objects and reload the teleoperation page. The browser toolbar also offers
a Reset practice button.
Both camera viewpoints are fixed. The browser subtracts the headset's position
from each controller position locally, so a translation of the headset and hands
together does not move the simulated arms. Face forward as you enter VR. The
browser uses that heading to align controller movement and wrist rotation with
the simulation. Turning your head or swapping camera views leaves the heading
fixed. If the headset recenters its tracking space, face forward again while
the arms realign.

On a shared PICO, open the teleoperation URL once and use PICO Browser's
Add as Web App action to save a launcher entry for later users. The icon opens
the same HTTPS address, so it still needs a valid certificate and network access.

Face forward and lower both controllers to your sides when starting the session.
Keep them steady until the two-second countdown ends, then move or rotate them
to control the simulated arms.
Left and right follow the overview image,
forward goes into the scene on the horizontal plane, and up follows world up.
The Cartesian solver targets the same displacement and rotation as the controller,
up to 15 cm and 60 degrees from alignment. Joint limits and the reachable
workspace can constrain the result. Loss of tracking pauses following until the
controller is tracked and held steady again. Exit VR to stop the session.

For OpenArm V2 grasp practice, set `"pico_grasp": true` in the simulation profile.
This starts both arms over a table with two movable wooden blocks. Each controller
drives its matching arm. The arms reset to a relaxed hanging pose at the start
of a new session. Push the thumbstick forward to open the fingers or back to
close them. Centering the thumbstick retains the last gripper setting. The
practice scene uses simplified contact pads on each jaw.
The practice profile allows up to 28 cm of target travel and includes a central
beam with a grasp point at either end. When both jaws on both arms contact the
beam with both grippers closed, the two arms follow the average translation and
keep their relative position and wrist orientations. Opening either gripper
returns the arms to independent control. The two individual blocks remain available.

Before entering VR, use **选择场景 / Choose scene** to select the wood practice,
a fallen cylinder with a pedestal, a tube and hollow socket, or a key with a
rotating slotted plug.
Switching scenes resets the props and arm poses. The reset button restores the
current task. The tube and key use simplified collision geometry with openings
and are not dimensioned replicas of any physical training fixtures.

The practice page shows both arms' joint angles, jaw opening in degrees, and
simulated grip force in newtons. The browser panel also shows the tool center
in world coordinates. The grip force is the smaller of the two jaws' summed
normal contact forces on the same object, so free motion and one-sided
touches read zero. The optional CSV records both arms at roughly 5 Hz, including
joint velocities, scene ID and TCP coordinates. Stop and download the recording before
refreshing or resetting the page.

Only the writable simulation instance exposes the receiver. The page cannot
control physical devices. A disconnected session releases the simulation arm;
a busy arm cannot be acquired by a second session.
