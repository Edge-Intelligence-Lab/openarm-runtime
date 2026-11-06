# Reusable model adapters and first upright deployment

The model integration belongs to **openarm-runtime**, not a duplicate robot loop
inside a research repository. Research code still owns training and benchmarks.
No TacPilot installation is needed by these adapters. Imports open no devices,
download no weights, and do not import Torch, LeRobot, OpenCV or Pinocchio.

## Four responsibilities, four boundaries

1. **Observation encoder**: declared streams -> model-native inputs. Owns camera
   selection, BGR/RGB, resize/crop, state layout and causal sample selection.
2. **Predictor**: checkpoint/runtime -> native predictions. Owns optional ML
   dependencies, normalization, device selection and checkpoint-bound transforms.
3. **Action decoder**: native predictions -> explicitly typed absolute targets.
   Owns units, rotation conventions, normalization saturation and chunk length.
4. **Robot capability**: targets -> feedback-supervised execution. Owns IK and
   calibration, command limits, feedback freshness, motor transport and lifecycle.

`openarm_policy.models:ModelPolicy` composes three JSON component specifications:
`{"factory": "installed.module:Factory", "kwargs": {...}}`. Predictor implements
`reset(context)`, `predict(inputs)`, `close()`; encoder implements `reset(context)`,
`encode(observation)`; decoder implements `reset(context)`,
`decode(output, observation)`. Optional `close()` is called for every component,
including partially initialized components. Components are constructed in the
existing spawned inference process, before robot startup.

A new model family or checkpoint format adds a predictor and, where needed, input
and output adapters. It does not change CAN, the scheduler, teleoperation or other
models. No extension is claimed to support arbitrary models without adapters.
The existing `ImportedPolicy` and `ChunkPolicyAdapter` APIs remain supported.

## Initial built-ins

- `models.act:TacPilotACT`: strict local full-policy load for the released
  state8/action7/H100 ACT format. Complete safetensors, original moments and the
  optional fixed-relative orientation adapter are required; unknown schemas fail.
  It does **not** implement generic LeRobot processor formats or fetch assets.
- `models.tacpilot:UprightObservation`: two required BGR streams, explicit RGB
  conversion and area resize to 256². Optional per-stream `[x,y,w,h]` crops are
  recorded in the caller's profile. Measured task TCP -> xyz/rotvec + two physical
  half-gaps in metres. No object position, task phase, depth or tactile input.
- `models.tacpilot:UprightActions`: task-frame translation increments at 0.02m
  scale, left-multiplied rotvec increments at 0.1rad scale and +1=closed gripper.
  Saturation to [-1,1] is explicit, matching simulation. Opening endpoints are
  supplied physical calibration, not guessed motor angles. It produces absolute
  goals from the observation's last dispatched goal; unexecuted predictions never
  advance hidden reference state.
- `models.http:NumpyHTTP`: bounded no-pickle NPZ loopback transport. Performs a
  health/model-hash check during reset, before robot startup; disables environment
  proxies and redirects. A remote GPU requires an externally managed SSH tunnel.

`ModelPolicy` records raw predictions and predictor/checkpoint identity in chunk
diagnostics. The Cartesian wrapper separately records requested TCP, joint targets,
measured TCP, FK of commanded joints, IK residual and calibration provenance.
Measured state is never substituted for a recorded command.

## A separate Cartesian capability, not joint vectors with a different label

The additive action kind `tcp_pose_xyzw_opening_m` has exactly eight scalars:
task-frame x/y/z metres, unit quaternion x/y/z/w, aperture metres. Joint-only
robots still accept **only** `joint_position_rad`; they cannot consume TCP chunks.
One selected arm is currently supported by `CartesianRobot`.

The optional `cartesian` section in a policy profile wraps the existing robot.
IK is solved at each local control tick from a persistent velocity-bounded joint
reference. Fresh measured feedback independently bounds command lead; this avoids
the known static-friction stall of reseeding every bounded step at measured joints.
Calibration uses a hash-pinned URDF, full joint mapping, TCP transform, explicit
task-from-base transform and a monotonic measured gripper lookup table. Task-space
and commanded-FK workspace bounds, deadlines and following error are checked before
dispatch. This is **not** tabletop/self-collision avoidance or a force controller.
The original gains, bus leases, worker watchdogs and verified disable are unchanged.

Verified geometry remains the default. A research trial may explicitly pass
`--allow-nominal-geometry` **and** declare `calibration_verified: false` plus
`calibration_status: "nominal_model_not_hardware_calibrated"`. This is a separate
operator opt-in, not a calibration bypass disguised as verified data: all joint,
workspace, tracking, freshness and watchdog checks still apply, and the nominal
label is recorded unchanged. It does not establish physical obstacle avoidance,
table-height adaptation or hardware readiness.

Gripper semantics need particular care: the simulation's normalized gripper
action interpolates **finger-joint targets**, while its measured state contains
physical gaps. Those are not generally linearly related. Supply an explicit
`gripper_action_curve` of `[normalized_action, opening_m]` pairs derived from the
source gripper geometry or verified equivalent hardware mapping, covering -1
(open) to +1 (closed). It is separate from the hardware motor/aperture table.
Without a curve, the decoder explicitly uses linear aperture endpoints as a
deployment approximation, recorded in chunk diagnostics; this is not exact
simulation gripper-command parity. Model prediction parity alone does not verify
the hardware command mapping. Encoder camera transforms and action mapping/scales
are also recorded in chunk diagnostics.

Absolute TCP chunks are safe to hold and to skip expired prefixes. They are not
precomputed joint chunks and are not repeatedly integrated delta actions. The
last requested Cartesian goal is retained across holds; requested and realized
goals remain separately observable. Constraint-induced lag remains a deployment
change, not an assertion of identical simulation physics.

## Deployment profiles and environment ownership

Copy `examples/upright-model.example.json` and
`examples/upright-policy.example.json` **outside Git**. They are templates, not
calibrated machine profiles: nulls and `calibration_verified: false` deliberately
prevent execution with fabricated TCP, task frame, opening or workspace values.

Fill:

- local complete upright checkpoint from the pinned private HF release; selected
  weights SHA256 `1ac3efac000201cdc13219c5ad43040f0978fc21f1c66f615000b50435398263`;
- verified model/motor signs and offsets, every other URDF joint explicitly locked;
- TCP matching the model's fingertip reference, **not an identity EE-base proxy**;
- rigid task-from-base transform, measured workspace and monotonic motor/gap pairs;
- action open/closed aperture endpoints within that calibration;
- the two actual camera streams and any crop/resize decisions.

The worker's current virtual-leader EE-base profile is not automatically a
calibrated ACT deployment profile. A user assertion alone cannot determine the
missing physical transform; measure or verify it before setting the flag.

Keep model inference in a separate Python 3.12 environment when needed. The
reference backend uses LeRobot 0.6.1 plus compatible Torch/torchvision,
safetensors and NumPy. Do not install LeRobot over a running camera/control
environment: the research inference lock and the video extra can require
different NumPy versions. Install this runtime package with `--no-deps` into the
already resolved inference environment; only the robot environment needs the
`cartesian`, `video` and physical `follower` dependencies. Pin both environments
and source versions in the deployment receipt. CPU inference may miss the live
deadline: measure the real latency before selecting local CPU, GPU or a tunnel.

Start the model service (no motor or camera access):

```sh
"$MODEL_PYTHON" -m openarm_runtime model-serve \
  --profile "$EXTERNAL/upright-model.json" --port 8765
```

Alternatively select the ACT predictor directly in `policy_kwargs.model` if the
robot environment intentionally includes the full compatible ML dependencies.

The operator then releases the existing teleoperation/controller and camera
owners normally. Do not restart another user's services or open the same camera
from multiple processes. Use a two-camera policy video profile; the unused right
camera is not a required dependency of this left-arm task. The policy has **not**
learned hanging-to-ready. Use the fixed preparation section below to perform it
within the same operator-started session; profiles without preparation still
require an already prepared pose. Never replay the simulator's 300 incremental
setup commands as an absolute hardware startup routine.

If cameras already belong to a running video service, replace `video_profile`
with `"video_http": {"url": "http://127.0.0.1:15003", "names": ["head", "left"]}`.
`HTTPVideoObservation` borrows that service without opening devices or changing
its configuration. Source sequence and conservative age are retained; missing,
stale, oversized or invalid images fail. These are JPEG **preview** images,
possibly resized by the owner, not lossless native frames. Actual decoded shapes
and transport provenance are recorded. Revalidate predictions for this input
path; do not silently call it identical to native acquisition. Simulate mode
rejects this option rather than contacting real peripherals.

Only the following explicit operator command enables and moves the selected arm:

```sh
"$ROBOT_PYTHON" -m openarm_runtime --config "$EXTERNAL/classic.json" policy \
  --robot classic --sides left --policy openarm_policy.models:ModelPolicy \
  --profile "$EXTERNAL/upright-policy.json" --duration 60 \
  --record "$EXTERNAL/episodes/unique-upright-episode"
```

The template requests 20Hz control and a new inference every 1.25s, using 25
actions from each 100-step prediction. Unlike simulation, the clock never pauses
for inference: expired absolute-goal prefixes are skipped, gaps hold only until
the timeout, then stop. Record latency and all scheduler/config changes; do not
claim bit-identical simulation execution or preserve timing by replaying late
commands. Ending/pause uses the existing disable lifecycle, **not a controlled
retreat or gravity-supported hold**. The operator must arrange physical support
and an emergency stop as appropriate to the installation.

The deployment task is still righting, releasing and retreating, leaving the
object independently standing for at least two seconds. Manual interventions and
protective stops remain failures in autonomous evaluation. Logs do not implement
an automatic real-object success judge. No hardware task performance is claimed
by model-load or offline regressions.

## Fixed preparation and automatic handoff

The upright example uses 3-degree joint arrival at the intermediate swing/elbow
waypoints. Task-ready and open-gripper waypoints instead declare `tcp_tolerance`:
`position_m: 0.02`, `orientation_rad: 0.174533` (10 degrees), and `gripper_rad:
0.05236` (3 degrees of motor angle). All three conditions must hold for five
consecutive samples; individual arm joints need not meet the arrival tolerance.
Each motion segment allows 20 seconds of settling after its trajectory finishes.
The total preparation still has a 120-second bound.

The robot's explicit `preparation_pose_error(actual, goal)` FK capability compares
measured feedback against the waypoint goal, not the last dispatched pose against
itself. No hardware access is needed for this calculation; missing FK capability
rejects TCP arrival configuration before enable. Rotation and gripper checks remain
necessary: matching xyz alone does not establish a usable grasp-start pose. Records
include position/orientation/gripper arrival errors. With nominal geometry these
are model estimates, not external measurements or a calibrated accuracy claim.

Arrival is separate from per-joint tracking-abort limits, speed, physical joint
bounds and freshness; those protections and the posture-based startup-region
check remain unchanged. Intermediate joint waypoints preserve the intended
preparation posture, not obstacle avoidance. Other tasks may configure their own
arrival regions without changing the generic sequencer.

The optional inline `preparation` object or a `preparation_profile` JSON path
(relative to the policy profile) selects `kind: "fixed_joint_waypoints_v1"`.
Only one may be supplied. The generic sequencer lives in
`openarm_policy.preparation`; task-specific poses remain external configuration.
`examples/upright-preparation.example.json` adapts the existing upright route:
outward alongside the table, flex elbow, reach task-ready, then open the gripper.
The first waypoint uses J4=0.03rad rather than the exact lower bound. The other
arm is never commanded by this single-arm capability. No leader is actuated.

One `policy --robot classic` invocation now does all of the following under the
existing device lease and stop lifecycle:

1. Validate model/sources, preparation bounds and configuration before enabling.
2. Start the robot once, holding actual feedback. Smoothly connect from the current
   pose, not a fabricated all-zero state. An explicit start-region check rejects
   unexpected postures; an already-ready posture skips the swing route.
3. Run minimum-jerk absolute joint waypoints with velocity/acceleration bounds.
   Retain the initial gripper angle until task-ready. Each waypoint must settle
   within configured measured joint tolerances for consecutive samples; failure,
   stale feedback, cancellation or timeout stops through the original cleanup.
4. Open and settle the gripper. Re-anchor the Cartesian IK posture and command
   reference at ready, clear observation history, and start learned inference.
   The robot is not disabled/re-enabled and stale predicted chunks do not cross
   the boundary. A bounded prepared-pose hold covers initial inference latency.

`--duration` counts the learned task only; preparation has its own declared
maximum duration. There are no model inference jobs during preparation (weights
are loaded beforehand). Setup commands use explicit joint names/kind, even when
task commands are Cartesian. `preparation_started`, `preparation_waypoint`,
`preparation_command`, `preparation_completed`, `task_started` and `handoff_hold`
events keep setup, waiting and model execution distinct. Observation events have
`phase: preparation|task`. Use `replay_observations(path, phase="task")` to exclude
setup from policy inspection/training; the default still replays all observations.

The source-scene preparation scan is an offline geometry check, not physical
validation. Fixed waypoints are not a planner for arbitrary real obstacles or
table heights. Retain the source and nominal/verified geometry labels, and do not
call scripted startup a learned-policy success. Installing or inspecting a profile
does not execute it; an operator must explicitly run the physical command.

## Read-only observation on an existing workstation

`openarm-runtime --config "$CLASSIC_PROFILE" snapshot --side left --output
"$NEW_EXTERNAL_JSON"` obtains a bounded disabled-arm snapshot. Its separate
`ReadOnlyBus` rejects non-query packets at the transport boundary, including
enable, disable, MIT targets and register writes. A busy bus or enabled motor
fails; it does not stop or take over another controller. Every joint timestamp
is retained because sequential feedback is not a synchronized sample.

`examples/observe_upright.py` composes these snapshots, borrowed HTTP cameras,
the released observation encoder and the loopback predictor. It constructs no
Robot, PolicySession or action decoder and sends no policy actions. Use an
external explicitly labeled `nominal_model_not_hardware_calibrated` kinematics
profile for this initial input inspection. A model-derived TCP, task transform
and pad gap are estimates, not physical calibration. Actual encoder readings
are not clipped to fit nominal model limits; deviations are recorded separately.
Raw state/RGB batches and full predictions are saved for replay. This is not a
20Hz control benchmark, safe-trajectory certificate or closed-loop result.

The source simulation TCP includes both its translation and an X-axis half-turn
(`quat="0 1 0 0"` in MuJoCo wxyz order). Dropping that rotation while copying
the TCP offset changes the orientation input. Deployment profiles must preserve
the complete transform and label whether it is nominal or physically verified.

Do not apply the simulation mount height directly to an arbitrary URDF root.
The initial deployment's pinned `56bd506b...` URDF and the `06b4c8a...` MuJoCo
asset have a **0.697m root-height offset**. Four offline paired joint configurations
matched by a fixed `[0, 0, -0.697]` translation (transform variation below
`1.4e-15`). Thus that source pair's nominal task-from-URDF z translation is
`1.35 - 0.697 = 0.653m`, not `1.35m`. This is a source-coordinate alignment,
not measurement of the real table or mount; recompute it for other assets.
The first real-input probe using the unaligned nominal frame is retained with
a coordinate-review sidecar and must not be used as evidence of correct state
semantics. Corrected probes retain the nominal-geometry label. Physical encoder
feedback and model-based TCP/pad estimates are distinct provenance categories.

## Offline checks

```sh
PYTHONPATH=src python -m unittest discover -s tests -p 'test_model_adapters.py' -v
PYTHONPATH=src python -m unittest discover -s tests -p 'test_policy*.py' -v
```

Core composition tests run without ML or hardware dependencies; numerical tests
skip explicitly when their optional libraries are absent. A real-checkpoint
parity check should compare the full H100 prediction against the historical
TacPilot loader on identical state/RGB batches, not just test random weights.
