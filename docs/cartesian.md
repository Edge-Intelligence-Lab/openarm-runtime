# Cartesian teleoperation: offline foundation

Install the optional `cartesian` extra. Legacy joint following remains selectable.
An experimental **virtual-leader** live mode is available when the workstation
sets `OPENARM_CARTESIAN_PROFILE` to an external JSON profile; it is not real
small-leader FK. Nothing enables or moves motors until an operator clicks start.

## Backend choice

Use Pink 4.4.0 / Pinocchio 4.1.0 and DAQP for constrained differential IK.
Pink and Mink both support weighted frame/posture tasks and bounded local QPs.
Pink reads our URDF directly and reuses the runtime's Pinocchio ecosystem; Mink
would add a MuJoCo model conversion and a second model-validation path. This is
an integration choice, not a claim that Pink is globally optimal or benchmarked
faster than Mink. Posture costs are soft, not strict task-priority nullspace IK.

`cartesian.KinematicChain` requires a hash-pinned expanded URDF, ordered input
joints, encoder signs/offsets, explicit locked joints, parent frame and TCP
transform. It is usable for leader FK and follower FK, but these are **different
models and calibrations**. Never substitute the virtual follower for the leader.
`RelativePose` captures both current TCP poses at takeover. Translations and
base-frame rotational deltas use an explicit proper axis rotation; scale applies
only to translation. Re-clutching recreates the mapping without a pose jump.

`DifferentialIK.step(measured, target, dt)` returns a local bounded joint step,
residuals and elapsed solve time. It starts from measured angles each cycle,
applies intersected physical/URDF joint limits and configured velocity bounds,
and independently verifies the result. Stale timestep (>50ms), invalid input,
out-of-limit feedback or solver failure raises, rather than returning a fallback
command. Successful QP execution is not a target-reachability certificate. The
caller must enforce feedback freshness, wall-clock deadlines, tracking error and
persistent residual policy before dispatch; these are not yet wired to hardware.

## Remaining activation gates

- Verified actual STS3215 leader geometry, axes, encoder zeros, TCP and frame mapping.
- Calibrated follower motor/model mapping, physical limits and TCP.
- Natural-hanging start, no-jump takeover and pause/re-clutch lifecycle integration.
- Separate Cartesian intent, dispatched joint commands and measured TCP streams.
- UI explicit mode selection; offline acceptance and operator-controlled handover.

No leader actuation, service restart, enable, homing or motor command is performed
by these classes themselves. They are not thread-safe; each worker owns its instance.

## Experimental live virtual-leader mode

`cartesian_teleop.VirtualLeaderIK` is integrated into the existing teleoperation
worker and ArmHold cleanup. The profile maps `left`/`right` to `KinematicChain`
constructor arguments plus `velocity_rad_s` (default 0.25, maximum 0.5). All paths,
calibration and model assets stay outside Git. Models load before motor access,
and preflight angles are checked against model/physical bounds before enable.

The existing relative leader-to-virtual-joint mapping supplies a **virtual** pose:
FK of the corresponding follower model becomes the six-DoF objective, then IK
computes the physical joint references with a fixed takeover posture preference.
Thus different joint configurations may satisfy the same Cartesian goal. This
is not true scaled small-leader TCP motion and does not establish its geometry.
The initial test TCP is explicitly the EE base frame, not a calibrated fingertip.

Select `末端 IK · 虚拟主臂` and `左臂`, then click `开始末端 IK 跟随` from the
current stationary posture. No home is required for this mode. The optional home
button still runs the original seven-joint homing procedure; do not use it unless
that motion is intended. Stop/disconnect/failure retains verified disable cleanup.
There is no table avoidance. Joint outputs are velocity-bounded and the gripper
has a separate 0.5rad/s slew limit. Feedback older than 150ms or a solve exceeding
one period rejects dispatch. The existing loop deadline and feedback checks remain.

The live adapter integrates a persistent commanded reference, reseeded only on
takeover/re-clutch. Resetting to measured angles every cycle was found in the
first physical trial to stall against friction: at 20Hz and 0.25rad/s, command
lead could never exceed 0.0125rad. The corrected adapter retains velocity bounds
between references and stops if reference-to-measured error exceeds 0.15rad
(gripper 0.20rad), preventing wind-up when obstructed. These are following-error
stop thresholds, not allowable joint travel. Measured TCP error and commanded
configuration residual are logged separately.

Collection preserves `intent/` mapped input and dispatched `robot/` joint actions,
adds `cartesian/<side>` with requested virtual TCP, measured pre-dispatch TCP and
IK output/residual/timing, and tags mode/model provenance. Recorded joint command
exports remain joint actions; they must not be labeled real leader Cartesian data.

## Future obstacles

The solver accepts Pink barrier objects through `barriers`; no default barriers
or collision geometry are configured. Virtual tabletop/self-collision constraints
need calibrated geometry, distances, margins, sampling and failure tests. Local
collision avoidance can stop at a local minimum: it is **not route planning**.
Deliberately moving around a table can require a separate waypoint/motion planner.

References: https://pink-kinematics.github.io/pink/ and
https://kevinzakka.github.io/mink/ .

## Leader geometry source candidate

`pkooij/open-arms-mini` at revision
`0628e081cfe105dc4156dba5099c2274c4aca2e8` documents seven arm joints plus
STS3215 gripper, and a deliberate software swap of wrist IDs 6/7 relative to the
follower. That agrees with the existing mapping convention, but is not proof that
the physical device is identical. Its complete Git tree contains individual
STEP/STL parts and assembly instructions, no URDF or assembled kinematic chain.
Confirm the physical build, derive or obtain joint-axis transforms, then verify
FK against measured positions/orientations before enabling Cartesian following.
Source: https://github.com/pkooij/open-arms-mini .

## Offline validation (2026-11-04)

With the pinned optional packages installed, all nine Cartesian tests passed:
relative takeover/re-clutch, rotated axes, encoder order/sign/offset/TCP mapping,
invalid model/pose rejection, fixture convergence, singular/unreachable fixture
targets, failure/output rejection, and official follower small-target tracking.
The complete available suite ran 329 tests with 47 optional-dependency/assets
skips and no failures. This is not a fully provisioned GUI/hardware suite.

The official-model check verifies the pinned Git blob, locks the other arm and
grippers, and runs 500 steps at a simulated 10ms timestep with 0.5rad/s bounds.
In one run: median 0.112ms, p99 0.138ms, max 0.201ms per step including residual
FK. The final model residual was 1.33e-6m / 4.40e-7rad. These are model-only,
small-target results, **not physical accuracy**, a workspace-wide benchmark,
obstacle acceptance or a hard real-time guarantee. The test uses identity
motor/model signs and an EE-base frame as a test TCP, not a deployment calibration.

```sh
OPENARM_CARTESIAN_URDF=/absolute/path/to/pinned/v2.urdf \
  python -m unittest discover -s tests -p 'test_cartesian*.py' -v
```
