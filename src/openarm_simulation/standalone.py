"""Run an MJCF scene using only the local MuJoCo installation."""

import json
import math
from pathlib import Path


def base_scene_path() -> Path:
    """Return the installed OpenArm v2 base scene."""
    return Path(__file__).resolve().parent / "assets" / "openarm-v2" / "base.xml"


def load_scene(scene: Path | None = None, *, calibration: Path | None = None):
    """Compile a local scene and apply the head baseline to its declared arm origin."""
    import mujoco
    import numpy as np

    path = base_scene_path() if scene is None else scene.expanduser().resolve()
    if scene is not None and calibration is None:
        return mujoco.MjModel.from_xml_path(str(path))

    from openarm_runtime.camera_calibration import transform
    from openarm_runtime.camera_parameters import load_camera_parameters

    baseline = load_camera_parameters(path=calibration)
    camera = baseline["camera"]
    extrinsics = baseline["extrinsics"]
    if (camera["camera_model"] != "pinhole" or extrinsics["transform_name"] != "base_from_camera"
            or extrinsics["units"] != "m"
            or extrinsics["camera_axes"] != {"x": "right", "y": "down", "z": "forward"}):
        raise ValueError("Expected OpenCV pinhole coordinates and base_from_camera in metres")
    width, height = camera["image_size"]
    k = np.asarray(camera["matrix"], dtype=float)
    if (not all(type(size) is int and size > 0 for size in (width, height))
            or k.shape != (3, 3) or not np.isfinite(k).all()
            or min(k[0, 0], k[1, 1]) <= 0
            or not np.allclose(k[2], [0, 0, 1]) or not np.allclose([k[0, 1], k[1, 0]], 0)):
        raise ValueError("Expected finite, zero-skew camera intrinsics and positive image dimensions")
    base_from_camera = transform(extrinsics["base_from_camera"])

    spec = mujoco.MjSpec.from_file(str(path))
    if spec.camera("calibrated_head") is not None:
        raise ValueError("Scene already defines the calibrated_head camera")
    spec.worldbody.add_camera(
        name="calibrated_head", resolution=[width, height], sensor_size=[width, height],
        focal_length=[k[0, 0], k[1, 1]], principal_length=[width / 2 - k[0, 2], height / 2 - k[1, 2]],
    )
    spec.visual.global_.offwidth = max(spec.visual.global_.offwidth, width)
    spec.visual.global_.offheight = max(spec.visual.global_.offheight, height)
    model = spec.compile()
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "arm_origin")
    if site < 0 or model.body_weldid[model.site_bodyid[site]] != 0:
        raise ValueError("Head calibration requires a fixed arm_origin site at the robot base")
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    world_from_base = data.site_xmat[site].reshape(3, 3)
    world_from_camera = world_from_base @ base_from_camera[:3, :3] @ np.diag([1, -1, -1])
    head = model.camera("calibrated_head")
    head.pos[:] = data.site_xpos[site] + world_from_base @ base_from_camera[:3, 3]
    mujoco.mju_mat2Quat(head.quat, world_from_camera.ravel())
    return model


def run(scene: Path | None, *, steps: int, view: bool, calibration: Path | None = None) -> None:
    try:
        import mujoco
    except ModuleNotFoundError as exc:
        if exc.name != "mujoco":
            raise
        raise SystemExit("Install openarm-runtime[simulation] to run MuJoCo.") from exc

    path = scene.expanduser().resolve() if scene is not None else base_scene_path()
    model = load_scene(scene, calibration=calibration)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if view:
        from mujoco import viewer

        viewer.launch(model, data)
        return

    for _ in range(steps):
        mujoco.mj_step(model, data)
    elapsed = steps * model.opt.timestep
    if not math.isclose(data.time, elapsed, rel_tol=1e-9) or not all(
        math.isfinite(value) for state in (data.qpos, data.qvel) for value in state
    ):
        raise SystemExit("MuJoCo did not complete the requested steps with finite state.")
    print(json.dumps({
        "scene": str(path),
        "steps": steps,
        "time_s": data.time,
        "joints": model.njnt,
        "actuators": model.nu,
    }))
