# Model-independent policy platform

`openarm_policy` defines the boundary between models and devices. Imports open no
hardware and load no model, NumPy, Viser, serial or CAN SDK. Model weights,
training frameworks, preprocessing and network transports stay in caller plugins.

## Design references

Reviewed 2026-11-01:

- [OpenPI BasePolicy](https://github.com/Physical-Intelligence/openpi/blob/main/packages/openpi-client/src/openpi_client/base_policy.py)
  separates inference/reset from the robot; its
  [ActionChunkBroker](https://github.com/Physical-Intelligence/openpi/blob/main/packages/openpi-client/src/openpi_client/action_chunk_broker.py)
  separates prediction from step consumption.
- [LeRobot Robot](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/robot.py)
  declares observation/action features independently of device lifecycle.

We follow those boundaries without framework dependencies or copied implementation.
Host-time deadlines, freshness, subprocess inference and independent CAN workers
extend them for this robot. A synchronous Gym-style `step` is not our control loop:
waiting for a model must not stop servicing enabled motors.

## Quick start (no hardware)

```sh
openarm-runtime policy --robot simulate --duration 3 \
  --record "$HOME/openarm-episodes/smoke-001"
```

Default `openarm_policy.adapters:HoldPolicy` returns chunks of the measured pose.
Simulation is an ideal position plant, not a physics model. The recording directory
must be new. No GPU/serial dependencies are needed for this command.

Select an installed research plugin with `--policy my_research.policy:create`.
The factory is imported inside the spawned inference process and receives kwargs
from the optional profile. It returns `reset(context)`, `infer(observation)` and
`close()`. Load weights during reset, not module import. A plugin may instead own a
remote client: model-server deployment, credentials and RPC remain in that adapter.

## External demo plugin

From the source checkout, run a changing action sequence through the same platform:

```sh
PYTHONPATH="$PWD/examples" openarm-runtime policy --robot simulate \
  --policy demo_policy:SinePolicy --duration 10 \
  --record "$HOME/openarm-episodes/sine-001"
```

The plugin reads the initial pose, then predicts 20-step chunks of joint1 motion
(both arms, amplitude 3 degrees, period 4 seconds), keeping other joints fixed.
It validates the full excursion against the declared limits. It is a standalone
example file: changing the plugin does not change the backend or scheduler.
Use the archive to check target changes, feedback, termination and action bounds.

## Protocol version 1

`Context` declares ordered actions, stream specs, control rate and task text. A
plugin validates required modalities and semantics in reset, before motor startup.

`Observation` contains a sequence, host-monotonic `captured_at`, and named bounded
histories of `SensorSample(sequence, received_at, data, valid, error)`:

- `robot/left`, `robot/right`: 8 x 3 position/velocity/torque, rad, rad/s, Nm;
  motor IDs 1–7 then stock gripper motor ID8.
- `camera/<name>`: original BGR uint8 images or uint16 depth in mm. No silent
  RGB conversion, resizing, crop or depth registration.
- `tactile/<name>`: 69 x 3 raw counts: resultant first, then taxels 0–67.
  Resultant and distribution are separate serial transactions. No implicit zero.

Receipt times are not exposure times or hardware synchronization. History samples
at the session rate and deduplicates source sequences; streams can have different
history lengths. Required stale/missing samples stop the episode; unavailable
optional streams have empty histories. Adapters choose causal alignment and any
additional history warmup (initial histories may contain just one sample). Payloads
are copied at the boundary and transferred to the inference process. Treat them as
read-only. Schemas specify encoding, shape, units, frame and freshness requirements.

`ActionChunk` contains source observation sequence, start time, step period and
H x A absolute motor targets with exact ordered names. H=1 is a single-step policy.
The default robot capability is `joint_position_rad`: side order follows configuration, each
side has `joint1`…`joint7`, then `gripper_motor`. Stock gripper bounds are left
0..+65°, right -65°..0; 0 is closed, not aperture in metres. Adapters convert their
normalized output to these motor radians. Bounds match teleoperation. Invalid
versions, modes, joint orders, nonfinite/out-of-range targets and horizons fail
explicitly; there is no silent clipping or inferred unit conversion.

All action times use the robot host monotonic clock. Remote plugins reconstruct
times from the original observation, never compare clocks on separate hosts.
Chunks normally anchor to observation time: if inference takes 120 ms, expired
prefix steps are skipped rather than replayed late. Entirely expired chunks fail.
Positive lead time can be explicit. Maximum horizon bounds lead plus duration.

Scheduling is latest-chunk replacement and zero-order hold between waypoints, with
one inference in flight. A future chunk waits while the active chunk continues.
Temporal ensembling/blending/inpainting belong in the model adapter because these
change the policy, not just transport. An action gap holds only for the configured
command timeout, then stops.

## Execution and lifecycle

`PolicySession.run()` is one episode. Model reset and required peripheral readiness
precede robot startup. `stop()`/`pause()` ends the episode and disables; resume means
a new session/robot with fresh reset, histories and chunk state. Old commands cannot
survive a pause. A UI can build resume around this explicit contract.

An optional fixed preparation runs after the single startup and before any model
inference job. It uses feedback-gated absolute joint waypoints, then clears setup
history and hands off without releasing the device lease or re-enabling motors.
Task duration excludes preparation. See [fixed preparation](model-adapters.md#fixed-preparation-and-automatic-handoff)
for configuration, provenance, nominal-geometry trial opt-in and recording phases.

Each CAN bus has its own worker, exclusive lease, existing ArmHold gains and feedback
validation. All arms preflight before any enable; an early ready arm keeps holding
while its peer starts. Target deadlines are enforced independently of model and
recording threads. Either arm's failure cancels the peer. Cleanup verifies disable;
this is not gravity compensation or a controlled retreat. Python is soft real-time.

Inference uses a spawned process with reset/inference deadlines. Shutdown attempts
cleanup then terminates a stuck worker. Plugins must not spawn unmanaged descendants;
remote model services own their lifecycle. We do not promise forcible cleanup of
arbitrary external services created by a model.

## Model adapter

```python
from openarm_policy import ChunkPolicyAdapter

def create():
    return ChunkPolicyAdapter(
        load=load_model,          # context -> callable predictor
        encode=prepare_inputs,   # observation, context -> model input
        decode=restore_actions,  # output, context -> H x A absolute joint radians
        period_s=0.05,
    )
```

Hooks own history selection, RGB conversion, normalization, text conditioning,
device placement, action denormalization and name/order mapping. Predictors may
use PyTorch, JAX, ONNX, rules or remote calls. Implement Policy directly for recurrent
state/custom timing. This boundary does not make an arbitrary pretrained model
compatible with this robot's training conventions.

## Machine composition

Optional caller-owned profile (`--profile`; paths relative to process working directory):

```json
{
  "session": {"control_hz": 20, "inference_hz": 10, "history": 8,
              "inference_timeout_s": 0.4, "command_timeout_s": 0.5,
              "max_horizon_s": 2},
  "task": "fold the cloth",
  "policy_kwargs": {},
  "video_profile": "/absolute/path/video.json",
  "tactile": {
    "left_inner": {"device": "/dev/serial/by-id/YOUR_SENSOR", "device_id": 2}
  }
}
```

CLI owns configured capture lifetimes. Python users can borrow already started
VideoCapture/TactileReaders with VideoObservation/TactileObservation; these never
open a second handle. Independent preview processes must release devices or expose
a caller-supplied snapshot adapter. Simulate mode never opens configured hardware
devices, though simulated peripherals still need their optional libraries.

Explicit physical execution (**enables and commands motors**):

```sh
openarm-runtime --config /absolute/path/classic.json policy --robot classic \
  --sides left right --policy my_research.policy:create \
  --profile /absolute/path/policy.json --duration 10 \
  --record "$HOME/openarm-episodes/physical-001"
```

Policy control rate must match the Classic profile. The optional explicit
`cartesian` profile wraps the joint robot with feedback-supervised TCP execution;
see [model adapters and upright deployment](model-adapters.md). Its separate
`tcp_pose_xyzw_opening_m` action kind cannot be sent to a joint-only robot. Torque
actions remain unsupported; never relabel position vectors as another action kind.

## Recording and replay

The policy CLI defaults to `--record-mode summary`. `CompactEpisodeRecorder`
saves only `summary.json`: context/model identity, event counts, latency aggregates,
one terminal Cartesian snapshot and bounded lifecycle/error/cleanup events. It
does not save images, chunks, action histories or replay arrays. `task_success`
remains null until an operator judges the real outcome; normal session shutdown
is not task success. Missing terminal events leave the receipt incomplete. A
complete receipt can still report failed execution or cleanup. This mode is not
input to `replay_observations`, training exporters or full-trace viewers.

For an explicitly requested detailed diagnostic, pass `--record-mode full`.
The existing Python `EpisodeRecorder` API remains full recording: it saves JSONL
events and lossless, non-pickle `.npy` sidecars. It
preserves context, observation histories/times, chunks, submitted targets and errors.
CLI Classic runs also record preflight/enable/disable events. Submitted targets are
not measured motor feedback. Storage errors terminate the session. Writes are
synchronous: slow disks may trip watchdogs; use adequate local storage.

```python
from openarm_policy import replay_observations
for observation in replay_observations("/path/to/episode"):
    output = policy.infer(observation)  # offline only: times belong to the old run
```

This is a platform trace, not a training dataset format. Exporters live in research
code. Shared-memory IPC and faster dataset writers can later replace implementation
details without changing Policy/Observation/ActionChunk.

## Validation boundary

Offline tests cover single/chunk policies, expired prefixes, future replacement,
invalid schemas/bounds, stale histories, missing cameras before enable, model
exceptions/hangs, storage failure, array replay, all-arm preflight and watchdog
cleanup. Installed-package simulation exercises the CLI. Separately, an
operator-authorized right-arm zero/one-cycle sine/return trial exercised the physical
backend with 50% and 100% gravity feedforward and verified disable of all eight
right-arm motors. See [physical results](gravity.md#physical-right-arm-trials).
This does not validate learned-model tracking, dual-arm policy motion, or arbitrary
trajectories. Camera/tactile-inclusive physical policy execution remains untested.
