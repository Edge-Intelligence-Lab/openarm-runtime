"""Read-only upright inference probe: existing cameras + disabled-arm feedback.

No Robot, PolicySession or action decoder is constructed. ReadOnlyBus has a
packet allowlist. Nominal geometry is explicitly labeled, never certified here.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time


def nominal_tcp(options, angles):
    import numpy as np
    import pinocchio as pin
    from scipy.spatial.transform import Rotation
    from openarm_runtime.cartesian import KinematicChain, pose
    chain = KinematicChain(**options["chain"])
    measured = np.asarray(angles, dtype=float)
    if measured.shape != (8,) or not np.isfinite(measured).all():
        raise ValueError("Expected eight finite measured motor positions")
    model = pin.buildModelFromXML(Path(options["chain"]["urdf"]).read_text())
    q = pin.neutral(model)
    for name, value in options["chain"]["locked"].items():
        q[model.joints[model.getJointId(name)].idx_q] = value
    for i, name in enumerate(options["chain"]["joints"]):
        q[model.joints[model.getJointId(name)].idx_q] = chain.signs[i]*measured[i] + chain.offsets[i]
    for name, sign in zip(options["finger_joints"], options["finger_signs"]):
        if not model.existJointName(name):
            raise ValueError("Unknown nominal finger joint")
        q[model.joints[model.getJointId(name)].idx_q] = sign*measured[7]
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    # FK observes encoders as they are, even if quantization/model mismatch puts
    # them outside nominal limits. Do not clip observations or reuse this path
    # for command IK, which retains its independent physical bounds checks.
    excess = np.maximum(model.lowerPositionLimit-q, q-model.upperPositionLimit)
    task_tcp = pose(options["task_from_base"]) @ (data.oMf[model.getFrameId(chain.frame)] * chain.tcp).homogeneous
    points = []
    for frame, point in zip(options["pad_frames"], options["pad_points_m"]):
        if not model.existFrame(frame):
            raise ValueError("Unknown nominal pad frame")
        points.append(data.oMf[model.getFrameId(frame)].act(np.asarray(point, dtype=float)))
    if len(points) != 2:
        raise ValueError("Two nominal pad points required")
    gap = float(np.linalg.norm(points[0]-points[1]))
    tcp = np.r_[task_tcp[:3, 3], Rotation.from_matrix(task_tcp[:3, :3]).as_quat(), gap]
    return tcp, dict(maximum_nominal_limit_excess_rad=float(max(0., excess.max())),
                     state_clipped=False, physical_pad_gap_measured=False)


def run(args):
    import numpy as np
    from openarm_policy import Context, ObservationBuffer, SensorSample, StreamSpec, HTTPVideoObservation
    from openarm_policy.models.http import NumpyHTTP
    from openarm_policy.models.tacpilot import UprightObservation
    from openarm_policy.recording import EpisodeRecorder
    from openarm_policy.robots import joint_spec
    from openarm_runtime.config import Profile
    from openarm_runtime.checks import snapshot

    if not 1 <= args.samples <= 100:
        raise ValueError("samples must be in [1, 100]")
    options = json.loads(args.kinematics.read_text())
    if options.get("calibration_status") != "nominal_model_not_hardware_calibrated":
        raise ValueError("This probe requires an explicitly labeled nominal model profile")
    if len(options["finger_joints"]) != 2 or len(options["finger_signs"]) != 2 or len(options["pad_frames"]) != 2:
        raise ValueError("Two nominal fingers/pad frames required")
    profile = Profile.load(args.config)
    source = HTTPVideoObservation(args.video_url, (args.overview, args.wrist))
    specs = source.streams + (StreamSpec(f"tcp/{args.side}", "tcp_pose_xyzw_opening_m", (8,),
                                         "m,xyzw,m", "nominal_task", .5),)
    context = Context(joint_spec((args.side,)), specs, 20, "observation-only upright probe")
    encoder = UprightObservation(f"camera/{args.overview}", f"camera/{args.wrist}", side=args.side)
    encoder.reset(context)
    predictor = NumpyHTTP(args.model_url, timeout_s=5, expected_model_sha256=args.model_sha256)
    predictor.reset(context)
    buffer = ObservationBuffer(specs, history=1)
    summaries = []
    with EpisodeRecorder(args.output) as recorder:
        recorder.record(dict(event="probe_context", time=time.monotonic(), camera=source.metadata,
                             model=predictor.metadata, kinematics=options,
                             model_state="measured_encoders_with_nominal_FK_and_pad_geometry",
                             synchronized=False, motion_commands_sent=False, context=asdict(context)))
        try:
            for index in range(args.samples):
                # This helper performs only allowlisted queries under a bus lease;
                # it fails if a controller owns the bus or motors are enabled.
                feedback = snapshot(profile, args.side)
                q = [feedback["states"][i]["position"] for i in range(1, 9)]
                tcp, geometry = nominal_tcp(options, q)
                samples = source.snapshot()
                samples[f"tcp/{args.side}"] = SensorSample(index, feedback["oldest_received_at"], tcp)
                observation = buffer.collect(samples)
                inputs = encoder.encode(observation)
                recorder.record(dict(event="observation", observation=observation))
                started = time.perf_counter()
                prediction = predictor.predict(inputs)
                latency = time.perf_counter()-started
                np.savez_compressed(args.output/f"probe-{index:03d}.npz", **inputs, raw_actions=prediction)
                recorder.record(dict(event="raw_prediction", time=time.monotonic(), index=index,
                                     feedback=feedback, geometry=geometry, raw_actions=prediction, inference_s=latency,
                                     motion_commands_sent=False))
                summaries.append(dict(index=index, inference_s=latency, state=inputs["state"].tolist(),
                                      raw_action_shape=list(prediction.shape),
                                      raw_action_min=float(prediction.min()), raw_action_max=float(prediction.max()),
                                      input_age_s=observation.captured_at-feedback["oldest_received_at"], geometry=geometry))
        finally:
            predictor.close()
    result = dict(schema="openarm.upright-observation-probe.v1", samples=summaries,
                  median_inference_s=float(np.median([s["inference_s"] for s in summaries])),
                  motion_commands_sent=False, calibration_status=options["calibration_status"],
                  scope="input and raw prediction inspection, not closed-loop hardware performance")
    (args.output/"summary.json").write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--kinematics", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--overview", default="head")
    parser.add_argument("--wrist", default="left")
    parser.add_argument("--video-url", default="http://127.0.0.1:15003")
    parser.add_argument("--model-url", default="http://127.0.0.1:18765/")
    parser.add_argument("--model-sha256", required=True)
    run(parser.parse_args())
