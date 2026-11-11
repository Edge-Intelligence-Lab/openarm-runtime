"""Optional Pinocchio gravity model and bounded motor-space feedforward.

No hardware or optional scientific libraries are imported until construction.
Configuration is machine-owned: model zero is sign * motor radians + offset.
"""
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET


def vector(value, count, name, *, positive=False):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{name} needs {count} values")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           or (positive and v <= 0) for v in value):
        raise ValueError(f"Invalid {name}")
    return tuple(float(v) for v in value)


class GravityModel:
    """Fixed-base URDF inverse statics, mapped by virtual work into motor Nm."""
    def __init__(self, config, root):
        import numpy as np
        import pinocchio as pin

        self.np, self.pin = np, pin
        path = (Path(root) / config["urdf"]).expanduser().resolve()
        source = path.read_bytes()
        digest = hashlib.sha256(source).hexdigest()
        if digest != config["urdf_sha256"]:
            raise ValueError("Gravity URDF hash mismatch")
        xml = ET.fromstring(source)
        if xml.tag != "robot":
            raise ValueError("Expected expanded URDF robot, not xacro")
        self.names = tuple(config["joints"])
        if len(self.names) != 7 or len(set(self.names)) != 7 or any(not isinstance(n, str) for n in self.names):
            raise ValueError("Gravity joints must name motor IDs 1..7 exactly once")
        self.signs = vector(config["signs"], 7, "signs")
        if any(s not in (-1, 1) for s in self.signs):
            raise ValueError("Motor/model directions must be -1 or +1")
        self.offsets = vector(config["offsets_rad"], 7, "offsets_rad")
        gravity = vector(config["gravity_m_s2"], 3, "gravity_m_s2")
        if not 9 <= math.sqrt(sum(v*v for v in gravity)) <= 10:
            raise ValueError("Specify Earth's gravity in the URDF root frame")
        # All inertial data must be finite. Missing inertials on driven links are
        # rejected; massless fixed frames are normal URDF constructs.
        links = {link.get("name"): link for link in xml.findall("link")}
        joints = {joint.get("name"): joint for joint in xml.findall("joint")}
        for name in self.names:
            joint = joints.get(name)
            if joint is None or joint.get("type") != "revolute" or joint.find("mimic") is not None:
                raise ValueError(f"Expected bounded independent revolute joint: {name}")
            child = links[joint.find("child").get("link")]
            if child.find("inertial") is None:
                raise ValueError(f"Missing inertial data for {name}")
        for inertial in xml.findall("./link/inertial"):
            mass = float(inertial.find("mass").get("value"))
            if not math.isfinite(mass) or mass < 0:
                raise ValueError("Invalid URDF mass")
            for element in inertial:
                for raw in element.attrib.values():
                    if any(not math.isfinite(float(v)) for v in raw.split()):
                        raise ValueError("Nonfinite URDF inertial data")
        payload = config.get("payload")
        if payload is not None:
            if set(payload) != {"link", "mass_kg", "com_m"} or payload["link"] not in links:
                raise ValueError("Invalid payload attachment")
            mass = vector([payload["mass_kg"]], 1, "payload mass", positive=True)[0]
            com = vector(payload["com_m"], 3, "payload COM")
            if "runtime_payload" in links or "runtime_payload_joint" in joints:
                raise ValueError("Reserved payload name already in URDF")
            link = ET.SubElement(xml, "link", name="runtime_payload")
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "origin", xyz=" ".join(map(str, com)))
            ET.SubElement(inertial, "mass", value=str(mass))
            ET.SubElement(inertial, "inertia", ixx="1e-9", iyy="1e-9", izz="1e-9", ixy="0", ixz="0", iyz="0")
            joint = ET.SubElement(xml, "joint", name="runtime_payload_joint", type="fixed")
            ET.SubElement(joint, "parent", link=payload["link"])
            ET.SubElement(joint, "child", link="runtime_payload")
        self.model = pin.buildModelFromXML(ET.tostring(xml, encoding="unicode"))
        self.model.gravity.linear = np.array(gravity)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.indices = []
        locked = config["locked_joints"]
        if not isinstance(locked, dict):
            raise ValueError("locked_joints must explicitly set every unmapped joint")
        movable = set(self.model.names[1:])
        if set(self.names) & set(locked) or set(self.names) | set(locked) != movable:
            raise ValueError("Every model joint must be mapped or explicitly locked")
        for name in self.model.names[1:]:
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1:
                raise ValueError("Only bounded scalar joints are supported")
            if name in locked:
                value = vector([locked[name]], 1, "locked joint")[0]
                self.q[joint.idx_q] = value
        for name in self.names:
            joint = self.model.joints[self.model.getJointId(name)]
            self.indices.append((joint.idx_q, joint.idx_v))
        if any(not np.isfinite(i.mass) or i.mass < 0 for i in self.model.inertias):
            raise ValueError("Invalid model inertias")
        if sum(i.mass for i in self.model.inertias) <= 0:
            raise ValueError("Gravity model has no mass")
        self.metadata = dict(urdf_sha256=digest, pinocchio_version=pin.__version__,
                             joints=self.names, signs=self.signs, offsets_rad=self.offsets,
                             gravity_m_s2=gravity, locked_joints=dict(locked), payload=payload)

    def torques(self, positions):
        positions = vector(positions, 8, "motor positions")
        for (qi, _), sign, offset, angle in zip(self.indices, self.signs, self.offsets, positions):
            self.q[qi] = sign * angle + offset
        # Allow 0.1 degree of motor readout quantization at angular endpoints;
        # evaluate the measured angle, never clip it into a different pose.
        tolerance = self.np.zeros(self.model.nq)
        for qi, _ in self.indices:
            tolerance[qi] = math.radians(.1)
        if (self.np.any(self.q < self.model.lowerPositionLimit - tolerance)
                or self.np.any(self.q > self.model.upperPositionLimit + tolerance)):
            raise ValueError("Measured pose outside gravity model limits; check zero mapping")
        generalized = self.pin.computeGeneralizedGravity(self.model, self.data, self.q)
        result = tuple(sign * float(generalized[vi]) for (_, vi), sign in zip(self.indices, self.signs)) + (0.,)
        return vector(result, 8, "gravity torque")


class GravityCompensation:
    def __init__(self, model, *, mode, limits_nm, slew_nm_s, ramp_s=2.0, scale=1.0):
        if mode not in ("shadow", "active"):
            raise ValueError("Gravity mode must be shadow or active")
        self.model, self.mode = model, mode
        self.limits = vector(limits_nm, 7, "limits_nm", positive=True) + (0.,)
        self.slew = vector(slew_nm_s, 7, "slew_nm_s", positive=True) + (0.,)
        self.ramp_s = vector([ramp_s], 1, "ramp_s", positive=True)[0]
        self.scale = vector([scale], 1, "scale")[0]
        if not 0 < self.scale <= 1 or self.ramp_s < .5:
            raise ValueError("Expected scale in (0,1] and ramp_s >=0.5")
        self.started = self.previous = None
        self.applied = (0.,) * 8
        self.last = None

    def preflight(self, states, ranges):
        if any(self.limits[i-1] > .8 * ranges[i].torque for i in range(1, 8)):
            raise ValueError("Gravity cap exceeds 80% of firmware torque range")
        self.model.torques(tuple(states[i].position for i in range(1, 9)))

    def update(self, states, now=None):
        now = time.monotonic() if now is None else now
        if not all(states[i].fresh(now, .4) for i in range(1, 9)):
            raise RuntimeError("Gravity requires fresh measured feedback")
        raw = self.model.torques(tuple(states[i].position for i in range(1, 9)))
        if self.started is None:
            self.started = self.previous = now
        dt = now - self.previous
        if not 0 <= dt <= .4:
            raise RuntimeError("Gravity update deadline missed")
        ramp = min(1., (now - self.started) / self.ramp_s)
        desired = tuple(v * self.scale for v in raw)
        limited = tuple(max(-cap, min(cap, value)) for cap, value in zip(self.limits, desired))
        goal = tuple(value * ramp for value in limited) if self.mode == "active" else (0.,) * 8
        self.applied = tuple(old + max(-rate*dt, min(rate*dt, value-old))
                             for old, rate, value in zip(self.applied, self.slew, goal))
        self.previous = now
        self.last = dict(raw_nm=raw, desired_nm=desired, applied_nm=self.applied,
                         saturated=[i+1 for i, (a, b) in enumerate(zip(desired, limited)) if a != b],
                         ramp=ramp, mode=self.mode,
                         input_positions_rad={i: states[i].position for i in range(1, 9)},
                         input_received_at={i: states[i].received_at for i in range(1, 9)})
        return dict(enumerate(self.applied, 1))

    def metadata(self):
        return dict(self.model.metadata, mode=self.mode, limits_nm=self.limits,
                    slew_nm_s=self.slew, ramp_s=self.ramp_s, scale=self.scale)


def select_gravity_config(config, side):
    """Resolve a bilateral profile without importing scientific dependencies.

    Model identity/mounting is shared; mapping, payload and bounds are arm-owned.
    Legacy single-arm profiles remain readable for archived episode diagnostics.
    """
    if not isinstance(config, dict):
        raise ValueError("Expected gravity profile object")
    if "schema" not in config:
        return dict(config)
    shared = {"schema", "urdf", "urdf_sha256", "gravity_m_s2", "arms"}
    if set(config) != shared or config["schema"] != "openarm.gravity.bimanual.v1":
        raise ValueError("Invalid bilateral gravity schema or fields")
    if side not in ("left", "right"):
        raise ValueError("Bilateral gravity requires an explicit side")
    arms = config["arms"]
    if not isinstance(arms, dict) or set(arms) != {"left", "right"}:
        raise ValueError("Bilateral gravity requires both arms")
    required = {"joints", "signs", "offsets_rad", "locked_joints", "mode", "limits_nm", "slew_nm_s"}
    names = []
    for arm in arms.values():
        if not isinstance(arm, dict) or required - set(arm) or set(arm) - required - {"ramp_s", "scale", "payload"}:
            raise ValueError("Invalid bilateral arm fields")
        joints = arm["joints"]
        if not isinstance(joints, list) or len(joints) != 7 or any(not isinstance(n, str) for n in joints):
            raise ValueError("Each arm requires seven joint names")
        names.extend(joints)
    if len(set(names)) != 14:
        raise ValueError("Left/right gravity mappings must be disjoint")
    return {key: config[key] for key in ("urdf", "urdf_sha256", "gravity_m_s2")} | dict(arms[side])


def validate_arm_branch(config, bundle, side, root):
    """Require independent fixed-base branches, not a coupled serial mechanism."""
    xml = ET.fromstring((Path(root) / config["urdf"]).expanduser().read_bytes())
    parents = {j.find("child").get("link"): j for j in xml.findall("joint")}
    joints = {j.get("name"): j for j in xml.findall("joint")}

    def ancestors(link):
        result = []
        while link in parents:
            joint = parents[link]
            result.append(joint.get("name"))
            link = joint.find("parent").get("link")
        return result

    for arm_side, arm in bundle["arms"].items():
        branch = arm["joints"]
        for index, name in enumerate(branch):
            if name not in joints:
                raise ValueError(f"Unknown gravity joint: {name}")
            chain = ancestors(joints[name].find("child").get("link"))
            movable = [n for n in reversed(chain) if joints[n].get("type") != "fixed"]
            if movable != branch[:index+1]:
                raise ValueError(f"{arm_side} gravity must map an ordered independent fixed-base branch")
        payload = arm.get("payload")
        if payload is not None and (not isinstance(payload, dict)
                or branch[-1] not in ancestors(payload.get("link"))):
            raise ValueError(f"{arm_side} payload must attach downstream of its last arm joint")


def load_gravity(path, *, side=None):
    path = Path(path).expanduser().resolve()
    bundle = json.loads(path.read_text())
    config = select_gravity_config(bundle, side)
    required = {"urdf", "urdf_sha256", "joints", "signs", "offsets_rad", "gravity_m_s2",
                "locked_joints", "mode", "limits_nm", "slew_nm_s"}
    if not isinstance(config, dict) or required - set(config) or set(config) - required - {"ramp_s", "scale", "payload"}:
        raise ValueError("Invalid gravity profile keys")
    if "schema" in bundle:
        validate_arm_branch(config, bundle, side, path.parent)
    model = GravityModel(config, path.parent)
    if "schema" in bundle:
        model.metadata.update(schema=bundle["schema"], side=side,
                              profile_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return GravityCompensation(model, **{k: config[k] for k in
        ("mode", "limits_nm", "slew_nm_s", "ramp_s", "scale") if k in config})


def prepare_gravity(profile, side, bus, preflight, *, required=False):
    """Resolve and evaluate before multi-arm enable barriers."""
    compensation = profile.make_gravity(side, required=True) if required else profile.make_gravity(side)
    if compensation is not None:
        from .telemetry import MotorState
        states = {mid: MotorState(**entry["state"]) for mid, entry in preflight.items()}
        compensation.preflight(states, bus.ranges)
    return compensation


def diagnose_episode(compensation, directory, side):
    """Offline policy-trace audit; residual is not a gravity-model error label."""
    from openarm_policy.recording import replay_observations
    if side not in ("left", "right"):
        raise ValueError("Select recorded side explicitly")
    if compensation.model.metadata.get("side", side) != side:
        raise ValueError("Gravity model side does not match recorded side")
    rows = [[] for _ in range(8)]
    previous = None
    for observation in replay_observations(directory):
        history = observation.streams.get(f"robot/{side}", ())
        if not history:
            continue
        sample = history[-1]
        if sample.sequence == previous:
            continue
        previous = sample.sequence
        positions = tuple(row[0] for row in sample.data)
        predicted = compensation.model.torques(positions)
        for i, (state, gravity) in enumerate(zip(sample.data, predicted)):
            q, v, torque = vector(state, 3, "recorded joint state")
            rows[i].append((q, v, torque, gravity))
    if not rows[0]:
        raise ValueError("No recorded feedback for selected side")
    result = []
    for mid, samples in enumerate(rows, 1):
        residuals = [torque-g for q, v, torque, g in samples]
        result.append(dict(motor_id=mid, samples=len(samples),
                           position_range_rad=[min(s[0] for s in samples), max(s[0] for s in samples)],
                           gravity_range_nm=[min(s[3] for s in samples), max(s[3] for s in samples)],
                           measured_torque_range_nm=[min(s[2] for s in samples), max(s[2] for s in samples)],
                           residual_rms_nm=math.sqrt(sum(r*r for r in residuals)/len(residuals))))
    return dict(model=compensation.metadata(), side=side, motors=result,
                note="Measured torque minus gravity includes dynamics, friction, quantization and model error; not a calibration verdict.")
