# Physical gravity compensation

Install `openarm-runtime[gravity]` to add Pinocchio 4.1.0. Core imports remain
dependency-free; NumPy/Pinocchio load only when a gravity profile is constructed.
This implements static gravity feedforward, not inverse dynamics, an integral
controller, friction identification, or a gravity-only hand-guiding mode.

## Bilateral deployment configuration

New deployments use one `openarm.gravity.bimanual.v1` file, illustrated by
`examples/gravity-bimanual.example.json`. Model path/hash and root-frame gravity
are shared. Both `arms.left` and `arms.right` explicitly declare ordered joint
mapping, signs/offsets, all locked joints, mode, limits, slew, ramp, scale and
optional payload. No right-to-left sign guessing or implicit mass calibration is
performed. The example has nominal stock inertia, no added tactile payload, and
closed finger geometry; these assumptions must be recorded in deployment receipts.
Its J1/J2 feedforward caps are 12 Nm (rather than the historical 10 Nm): the pinned
stock model predicts about 10.9 Nm in the audited preparation/workspace samples.
These are feedforward caps, not total-effort or collision-safety guarantees;
firmware range preflight and combined PD/feedforward supervision still apply.

The Classic machine profile routes **both sides to the same file**:

```json
"gravity": {"left": "gravity-bimanual.json", "right": "gravity-bimanual.json"}
```

`Profile.make_gravity(side)` and offline `gravity --side left|right` select the
arm explicitly. Each worker has independent Pinocchio data, ramp and slew state;
configuring two arms never enables the unselected arm. Existing `ArmHold` applies
the same feedforward beneath hold, joint/Cartesian teleoperation and policy/fixed
preparation. Model/profile hashes and selected side are recorded with gravity events.

The loader rejects overlapping mappings, out-of-order or coupled branches and
payloads attached to the other arm. The fixed-base branch requirement is important:
locking the other arm is valid for this robot's independent static branches, not
for floating-base/coupled robots. Finger poses remain an explicit static assumption;
motor ID8 receives no gravity torque. There is no automatic payload identification.

Legacy single-arm files remain readable for old recordings, but are no longer the
recommended deployment format. Historical right-arm trials are not bilateral
hardware acceptance. Example `shadow` mode sends zero gravity torque, **but ordinary
hold/policy still enables PD**; only offline episode diagnostics are motion-free.

## Control calculation

At each control exchange, the latest measured motor angles are mapped into the
URDF: `q_model[i] = signs[i] * q_motor[i] + offsets_rad[i]`. Pinocchio computes
`g(q) = dU/dq`, the torque needed to oppose gravity. Virtual work maps it back:
`tau_motor[i] = signs[i] * g_model[i]`. **Do not negate g a second time.** The
motor's existing PD controller receives this as an additive MIT feedforward torque.
Targets and gains remain unchanged. No policy or leader adapter supplies gravity.

The mapping covers motor IDs 1..7. Motor ID8 receives zero feedforward: a stock
gripper transmission is not assumed to be a direct joint. Every remaining URDF
joint must have an explicit `locked_joints` value. The example fixes finger geometry
at the closed pose; this approximation must be assessed if aperture or attached
tools change COM appreciably. Fixed-base left/right branches are independent; a
locked other arm in a full dual URDF does not command or observe that physical arm.

The model uses inertial mass/COM, not visual meshes. Expanded URDF bytes are pinned
by SHA256. Missing driven-link inertials, unaccounted joints, invalid mappings and
nonfinite predictions reject startup. Measured angles are checked against model
limits with 0.1 degree endpoint readout tolerance; they are not silently clipped.
Mounting gravity is a vector in the **URDF root frame**, not necessarily base-link
or motor coordinates. Changing mount orientation changes this vector.

## Configuration

Copy `examples/gravity-bimanual.example.json` outside the checkout and set `urdf` relative to
that JSON file. It references the official OpenArm v2 visual model's expanded URDF
(description revision `2e27c793e10aa2045e7eb597b9b9dc08fb0a08`), already used by the
viewer. It is an initial **shadow** configuration, not a physically calibrated
dynamics model. Signs/offsets, gravity direction, mass/COM accuracy and limits must
match the actual machine. The example torque caps are commissioning settings, not
manufacturer continuous-torque ratings. The original zero calibration is not changed.

Add both side paths to the existing Classic machine JSON:

```json
"gravity": {"left": "gravity-bimanual.json", "right": "gravity-bimanual.json"}
```

These paths resolve relative to the machine profile. Python-created Profile objects
interpret paths relative to their process working directory. Omitting a side keeps
the legacy zero-feedforward controller on that side. A new model/data instance is
created for every arm session; Pinocchio mutable data is not shared across threads.

Modes:

- `shadow`: calculate and record model predictions, send zero feedforward. Running
  a hold/teleop/policy command still enables the normal PD controller; **shadow is
  not a read-only hardware command**.
- `active`: apply scaled, capped and ramped compensation. `scale` is in (0,1],
  `ramp_s` >=0.5, and per-joint `limits_nm`/`slew_nm_s` are positive seven-vectors.
  Startup begins with zero compensation. Saturation is explicit in every record.
  Caps above 80% of the discovered firmware encoding range reject before enable;
  this encoding range is not proof of a continuous hardware torque capability.

Optional attachment, in the named link's frame:

```json
"payload": {"link": "openarm_right_link7", "mass_kg": 0.15, "com_m": [0, 0, -0.06]}
```

Use an actual link name from your URDF. Payload mass must be additional to mass
already represented in the URDF; don't count the stock gripper twice. Measured
sensor/cable/tool mass and COM belong here or in a separately hash-pinned URDF.

## Offline inspection (never opens CAN)

```sh
openarm-runtime gravity --profile /path/gravity-bimanual.json --side left \
  --positions 0 0 0 0 0 0 0 0
```

This prints raw motor-space Nm and provenance; it does not apply configured caps or
pretend the result is measured torque. Existing episode observations can be passed
through `compensation.model.torques(...)` for offline comparison. Compare at the
same measured pose/time; moving PD effort and friction are not gravity ground truth.

```sh
openarm-runtime gravity --profile /path/gravity-bimanual.json \
  --episode /path/to/recorded-policy-episode --side right
```

The offline audit deduplicates feedback sequences and reports per-motor position,
predicted gravity and measured torque ranges, plus residual RMS. It opens no devices.
Residuals include dynamics, friction, sensor quantization and model error; do not
treat them as a calibrated mass estimate or an acceptance score.

## Firmware-range-correct encoding

The official SDK stays unchanged. Its MIT encoder uses a motor-type TMAX; firmware
can differ (known right ID5: 3 Nm versus SDK 10 Nm). For a requested firmware-space
torque `tau`, pass `tau * SDK_TMAX / firmware_TMAX` into the official encoder. Decode
the resulting 12-bit torque field using the firmware range and verify agreement
within one LSB. Zero retains the original midpoint 2047. This quantizes to a tiny
negative value on the symmetric range, as did the original zero command.

Nonfinite/out-of-range torque rejects before sending; no firmware registers are
rewritten. Fresh feedback, motor faults and temperature checks remain enforced.
Combined estimated PD plus feedforward effort is checked against 90% of firmware
range when gravity is configured. Update gaps >0.4 s abort; existing CAN/session
watchdogs remain independent. Every exit still uses verified motor disable; disable
is not a supported pose hold, so gravity may move the unpowered arm afterward.

## Integration and records

The same ArmHold integration is used by Python `hold_arm`, the panel, teleoperation
and Classic policy execution. Both arm model preflights finish before the shared
enable barrier in dual-arm paths. Hardware zero calibration deliberately retains
zero feedforward: its coordinate frame changes during calibration. Recheck gravity
mapping after calibration before activating compensation.

`gravity_config` records model hash, library version, mappings, payload and limits.
Every `gravity_sample` records input measured positions/receipt times, raw/scaled/
applied Nm, saturation and ramp, submitted targets, and the resulting motor states.
Input and resulting feedback times are distinct. Policy CAN workers enqueue these
records for the orchestration thread rather than writing to disk themselves.

## Validation and commissioning boundary

Offline tests check analytic multi-link potential-energy gradients, motor signs and
zero offsets, mounting gravity, added payloads, profile rejection, ramps/caps/slew,
shadow zero output, stale feedback, pre-enable failures and disable after model
failure. Firmware range tests include the 3/10 Nm mismatch and unchanged zero bits.

Physical commissioning is separate: inspect model-predicted signs across known
poses, compare stationary support effort in shadow, then test limited active
compensation on one arm and compare the same trajectories/gains. Motion logs from
the earlier zero-feedforward trials are not validation of active gravity control.
In particular, a CAD model's mass/COM and the physical mount must be verified before
claiming calibrated gravity cancellation or making active compensation a default.

2026-11-01 development verification: installed-package macOS suite 199 tests,
197 passed / 2 skipped; isolated Linux gravity/lifecycle suite 21 passed. On Linux,
the unchanged official SDK 1.4.1 encoded 28 offline packet cases over DM8009,
DM4340 and DM4310, including a 3 Nm firmware / 10 Nm SDK mismatch, within one
firmware torque LSB. No CAN socket was opened by that packet check. The pinned
OpenArm URDF loads on both platforms; a prior right-arm episode was evaluated
offline by the new CLI. Physical trials are recorded separately below.

## Physical right-arm trials

Operator-authorized trials on 2026-11-01 used the fixed official URDF above, no added
payload, default gains (J1 Kp=30, Kd=1.5), Classic 20 Hz execution, and right arm only.
Each first returned all right joints toward existing calibrated zero (<=3 degrees/s),
waited for all joints within 1 degree and low velocity for 0.5 seconds, then commanded
one J1 +/-20 degree sine cycle of 24 seconds. Other targets stayed zero. The episode
held zero after the cycle and ended at 45 seconds with verified eight-motor disable.

| Model compensation | Measured J1 minimum | Measured J1 maximum |
| --- | ---: | ---: |
| None (earlier baseline) | -12.53 degrees | +13.43 degrees |
| 50% | -15.40 degrees | +16.16 degrees |
| 100% | -18.22 degrees | +19.03 degrees |

Both active trials ended normally, without compensation saturation or cleanup
errors. Full compensation applied J1 feedforward between -3.72 and +3.75 Nm;
maximum absolute J1 torque feedback was 4.76 Nm. Its final enabled J1 feedback was
-1.10 degrees: returning the target to zero does not imply exact physical settling.
The left arm was not acquired or commanded. The 50% and 100% episode identifiers are
`right24-gravity50-001` and `right24-gravity100-001`; raw traces remain outside Git.

These single-run extrema support improved amplitude tracking under gravity
compensation, not a repeated accuracy measurement or proof of complete load-model
calibration. Remaining work includes multiple poses, repeatability, payload changes,
final settling, left/dual-arm trials and compensated teleoperation. Machine defaults
remain opt-in; the checked-in example remains shadow mode. The experimental runner
and machine-specific profiles stay outside the package and repository.
