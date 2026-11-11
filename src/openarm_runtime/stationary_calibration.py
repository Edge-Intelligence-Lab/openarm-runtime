"""Offline stop-and-sample eye-to-hand calibration; no devices or activation.

This is deliberately a separate contract from exposure-synchronized observations.
An acquisition interval must come from a qualified camera source (not a renamed
host receipt timestamp). Measured robot feedback must bracket that whole interval.
Numerical agreement does not certify the camera timing, FK or fixture rigidity.
"""
import hashlib
import json
import math
import re

from .camera_calibration import transform


def number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{name}: finite number required')
    return float(value)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def sha256(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def joint_vector(value):
    if not isinstance(value, (list, tuple)) or len(value) != 8:
        raise ValueError('Eight measured joints including fixed gripper required')
    return [number(v, 'joint state') for v in value]


def validate_stationary(sample):
    """Validate an auditable stationary bracket, never infer exposure from receipt.

    The source qualification binds a conservative acquisition interval to one PNG.
    Its producer is trusted runtime code, not browser-supplied timestamps. Trace
    times are per-joint feedback receipt on the same host monotonic clock. Each
    joint must independently bracket acquisition; oldest-joint aggregate time is
    insufficient. Retain this trace with the unmodified image for later review.
    """
    import numpy as np
    evidence = sample['stationary_evidence']
    schema = evidence.get('schema')
    if schema not in ('openarm.stationary-bracket.v1', 'openarm.stationary-bracket.v2'):
        raise ValueError('Stationary bracket contract required')
    acquisition = evidence['acquisition']
    if (acquisition.get('clock') != 'host_monotonic'
            or acquisition.get('semantics') != 'qualified_acquisition_interval'
            or acquisition.get('qualification') not in
                ('v4l2_monotonic_capture', 'verified_source_latency_bound', 'synthetic_test')
            or acquisition.get('png_sha256') != sample.get('png_sha256')):
        raise ValueError('Require source-qualified acquisition interval bound to original PNG')
    if not sha256(sample.get('png_sha256')):
        raise ValueError('Original image digest required')
    if not sha256(acquisition.get('qualification_evidence_sha256')):
        raise ValueError('Retain the source timing qualification evidence digest')
    start = number(acquisition['earliest_s'], 'acquisition earliest')
    end = number(acquisition['latest_s'], 'acquisition latest')
    received = number(acquisition['host_received_at_s'], 'host receipt')
    if not 0 <= start <= end <= received or end-start > .5:
        raise ValueError('Invalid or unbounded acquisition interval')
    trace = evidence['feedback']
    if not isinstance(trace, list) or not 12 <= len(trace) <= 2000:
        raise ValueError('Bounded continuous measured-feedback trace required')
    q, velocity, times = [], [], []
    sequence = -1
    for row in trace:
        if (type(row.get('sequence')) is not int or row['sequence'] <= sequence
                or row.get('source') != 'measured_feedback'
                or not isinstance(row.get('status'), list)
                or len(row['status']) != 8
                or any(type(v) is not int or v != 1 for v in row['status'])):
            raise ValueError('Fresh ordered enabled measured feedback required')
        sequence = row['sequence']
        q.append(joint_vector(row['positions_rad']))
        velocity.append(joint_vector(row['velocities_rad_s']))
        receipts = joint_vector(row['received_at_s'])
        observed = number(row['observed_at_s'], 'feedback observation time')
        if not max(receipts) <= observed <= min(receipts)+.15:
            raise ValueError('Joint feedback is future-dated or stale at observation')
        times.append(receipts)
    q, velocity, times = np.array(q), np.array(velocity), np.array(times)
    if (np.any(times < 0) or np.any(np.diff(times, axis=0) <= 0)
            or np.max(np.diff(times, axis=0)) > .15
            or np.max(times[0]) > start-.5 or np.min(times[-1]) < end+.2):
        raise ValueError('Every joint must continuously bracket acquisition with settling margins')
    # v1 retains its historical velocity rule. v2 uses the same tight measured
    # position envelope across fresh feedback; raw speed codes remain evidence,
    # not a zero-speed requirement below firmware quantization/bias.
    if np.max(np.ptp(q, axis=0)) > .003 or (schema.endswith('.v1') and np.max(np.abs(velocity)) > .01):
        raise ValueError('Robot or gripper moved during stationary acquisition bracket')
    # Use one real feedback row, not a commanded target or an invented mean state.
    index = int(np.argmin(np.abs(np.mean(times, axis=1)-(start+end)/2)))
    selected = joint_vector(sample['measured_joints_rad'])
    if selected != trace[index]['positions_rad']:
        raise ValueError('FK must use the selected original measured-feedback row')
    return dict(stationarity_method='position_envelope' if schema.endswith('.v2') else 'position_and_reported_velocity',
                selected_feedback_index=index, max_joint_span_rad=float(np.max(np.ptp(q, axis=0))),
                max_velocity_rad_s=float(np.max(np.abs(velocity))),
                acquisition_interval_s=[start, end],
                qualification=acquisition['qualification'],
                qualified_acquisition_interval=True, exposure_timestamp_known=False)


def fit_stationary(observations, chain):
    """Solve only fitting poses, then test every predeclared held-out pose.

    Compute measured FK again rather than trusting supplied pose matrices. Never
    call the synchronized fitter with fabricated matching image/robot timestamps.
    No outlier removal, threshold tuning, activation or original evidence edits.
    """
    import cv2
    import numpy as np
    from scipy.spatial.transform import Rotation
    data = observations
    if (data.get('schema') != 'openarm.stationary-head-observations.v1'
            or data.get('units') != 'm' or data.get('joint_state_source') != 'measured_feedback'
            or data.get('kinematics_sha256') != digest(chain.metadata)):
        raise ValueError('Metric stationary observations with exact FK mapping binding required')
    for key in ('camera_name', 'base_frame', 'tool_frame', 'intrinsics_sha256', 'route_sha256'):
        if not isinstance(data.get(key), str) or not data[key]:
            raise ValueError(f'Missing provenance: {key}')
    if not sha256(data['intrinsics_sha256']) or not sha256(data['route_sha256']):
        raise ValueError('Exact intrinsics and route digests required')
    samples = data['samples']
    if not isinstance(samples, list) or not 12 <= len(samples) <= 40:
        raise ValueError('At least eight fitting and four held-out observations required')
    route = data['route']
    search = route.get('schema') == 'openarm.calibration-search-route.v1'
    manual = route.get('schema') == 'openarm.calibration-manual-route.v1'
    if (route.get('schema') not in ('openarm.calibration-route.v1','openarm.calibration-search-route.v1','openarm.calibration-manual-route.v1') or route.get('side') != 'left'
            or digest(route) != data['route_sha256']):
        raise ValueError('Route identity changed after partition selection')
    expected = [(p['id'], p['role']) for p in route['waypoints']]
    if manual:
        protocol=route.get('protocol',{})
        slots=protocol.get('slots',[])
        if (protocol.get('schema')!='openarm.manual-head-protocol.v1'
                or data.get('manual_protocol_sha256')!=digest(protocol)
                or expected!=[(p['id'],p['role']) for p in slots]
                or [r for i,r in expected]!=['fit']*12+['validation']*4):
            raise ValueError('Manual captures require the original fixed 12/4 protocol')
    if search:
        declared=dict(expected)
        actual=[(s['waypoint_id'],s['role']) for s in samples]
        if (len(declared)!=len(expected) or len({i for i,r in actual})!=len(actual)
                or any(declared.get(i)!=r for i,r in actual)
                or data.get('accepted_waypoint_ids')!=[i for i,r in actual]
                or sum(r=='fit' for i,r in actual)!=12 or sum(r=='validation' for i,r in actual)!=4):
            raise ValueError('Search samples changed predeclared roles or acceptance identity')
        points={p['id']:p for p in route['waypoints']}
        selected_points=[points[s['waypoint_id']] for s in samples]
    else:
        selected_points=route['waypoints']
    if not search and [(s['waypoint_id'], s['role']) for s in samples] != expected:
        raise ValueError('Missing, reordered or reclassified route observations')
    if len(set(s['png_sha256'] for s in samples)) != len(samples):
        raise ValueError('Duplicate original image')
    roles = np.array([s['role'] for s in samples])
    if np.sum(roles == 'fit') < 8 or np.sum(roles == 'validation') < 4 or np.any(
            (roles != 'fit') & (roles != 'validation')):
        raise ValueError('Require predeclared fitting and validation partitions')
    source = samples[0]['camera_identity']
    if (not isinstance(source, dict) or source.get('camera_name') != data['camera_name']
            or type(source.get('simulation')) is not bool):
        raise ValueError('Camera identity and mode binding required')
    diagnostics, B, C = [], [], []
    sequences = []
    gripper = joint_vector(route['start_rad'])[7]
    if not search and len({p['id'] for p in route['waypoints']}) != len(samples):
        raise ValueError('Unique route identities required')
    for sample, waypoint in zip(samples, selected_points):
        if sample['camera_identity'] != source:
            raise ValueError('Camera source, mode or geometry changed')
        sequences.append(sample['frame_sequence'])
        if (type(sequences[-1]) is not int or sequences[-1] < 1
                or (len(sequences) > 1 and sequences[-1] <= sequences[-2])):
            raise ValueError('Fresh strictly increasing camera frames required')
        quality = number(sample['reprojection_rms_px'], 'reprojection RMS')
        if not 0 <= quality <= 1.5:
            raise ValueError('Poor board detection; no automatic outlier deletion')
        diagnostics.append(validate_stationary(sample))
        if source['simulation'] != (diagnostics[-1]['qualification'] == 'synthetic_test'):
            raise ValueError('Synthetic timing cannot qualify a physical camera')
        measured, target = joint_vector(sample['measured_joints_rad']), joint_vector(waypoint['joints_rad'])
        if max(abs(a-b) for a, b in zip(measured, target)) > .035:
            raise ValueError('Measured pose did not arrive at its declared waypoint')
        if abs(measured[7]-gripper) > .003 or abs(target[7]-gripper) > 1e-10:
            raise ValueError('Gripper changed during rigid-board calibration')
        B.append(transform(chain.forward(sample['measured_joints_rad'][:7])))
        C.append(transform(sample['camera_from_board']))
    B, C = np.array(B), np.array(C)
    selected = roles == 'fit'
    turns = Rotation.from_matrix(B[selected][0, :3, :3].T@B[selected, :3, :3]).as_rotvec()
    singular = np.linalg.svd(turns, compute_uv=False)
    if singular[1] < .2 or np.max(np.linalg.norm(turns, axis=1)) < .35:
        raise ValueError('Measured fitting rotations do not excite two axes')
    G = np.linalg.inv(B[selected])
    r, t = cv2.calibrateHandEye(list(G[:, :3, :3]), list(G[:, :3, 3]),
        list(C[selected, :3, :3]), list(C[selected, :3, 3]), method=cv2.CALIB_HAND_EYE_PARK)
    X = np.eye(4); X[:3, :3] = r; X[:3, 3] = t.ravel(); transform(X)
    constants = np.linalg.inv(B)@X@C
    mean = Rotation.from_matrix(constants[selected, :3, :3]).mean()
    Y = np.eye(4); Y[:3, :3] = mean.as_matrix()
    Y[:3, 3] = np.mean(constants[selected, :3, 3], axis=0)
    distances = np.linalg.norm(constants[:, :3, 3]-Y[:3, 3], axis=1)
    angles = (mean.inv()*Rotation.from_matrix(constants[:, :3, :3])).magnitude()
    metrics = {}
    for role in ('fit', 'validation'):
        mask = roles == role
        metrics[role] = dict(samples=int(np.sum(mask)),
            rms_translation_m=float(np.sqrt(np.mean(distances[mask]**2))),
            rms_rotation_rad=float(np.sqrt(np.mean(angles[mask]**2))),
            max_translation_m=float(np.max(distances[mask])),
            max_rotation_rad=float(np.max(angles[mask])))
        if metrics[role]['rms_translation_m'] > .005 or metrics[role]['rms_rotation_rad'] > .03:
            raise ValueError(f'{role}: inconsistent stationary calibration: {metrics[role]}')
    return dict(schema='openarm.stationary-head-estimate.v1',
        transform_name='base_from_camera', transform=X.tolist(), tool_from_board=Y.tolist(),
        camera_name=data['camera_name'], base_frame=data['base_frame'], tool_frame=data['tool_frame'],
        units='m', metrics=metrics, stationary_diagnostics=diagnostics,
        per_pose=[dict(waypoint_id=s['waypoint_id'], role=s['role'], translation_m=float(p),
                       rotation_rad=float(a)) for s, p, a in zip(samples, distances, angles)],
        kinematics_sha256=data['kinematics_sha256'], intrinsics_sha256=data['intrinsics_sha256'],
        route_sha256=data['route_sha256'], observations_sha256=digest(data),
        synthetic=any(d['qualification'] == 'synthetic_test' for d in diagnostics),
        timing_contract='stationary_bracket_not_exposure_synchronization',
        calibration_status='estimated_not_independently_verified', installed=False)
