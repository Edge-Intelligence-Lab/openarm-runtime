"""Offline metric head/hand-eye calibration. Never opens a camera or a robot.

Inputs must be measured board poses paired with measured-feedback robot FK.
Estimated extrinsics are not automatically installed or marked verified.
"""
import hashlib
import json
from pathlib import Path


def transform(value):
    import numpy as np
    a = np.asarray(value, dtype=float)
    if (a.shape != (4, 4) or not np.isfinite(a).all() or not np.allclose(a[3], [0, 0, 0, 1])
            or not np.allclose(a[:3, :3].T@a[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(a[:3, :3]), 1., atol=1e-6)):
        raise ValueError('Expected a finite rigid transform in metres')
    return a


def board_pose(corners, matrix, distortion, pattern, square_size_m):
    """Known inner-corner grid -> camera_from_board and reprojection RMS pixels."""
    import cv2
    import numpy as np
    if (len(pattern) != 2 or any(type(n) is not int or n < 3 for n in pattern)
            or not 0 < square_size_m <= .2):
        raise ValueError('Specify the measured metric board geometry')
    points = np.zeros((pattern[0]*pattern[1], 3), dtype=np.float64)
    points[:, :2] = np.mgrid[:pattern[0], :pattern[1]].T.reshape(-1, 2)*square_size_m
    pixels = np.asarray(corners, dtype=float).reshape(-1, 2)
    K, dist = np.asarray(matrix, dtype=float), np.asarray(distortion, dtype=float)
    if (pixels.shape != (len(points), 2) or K.shape != (3, 3)
            or not np.isfinite(pixels).all() or not np.isfinite(K).all() or not np.isfinite(dist).all()
            or K[0, 0] <= 0 or K[1, 1] <= 0):
        raise ValueError('Invalid board corners or camera intrinsics')
    ok, r, t = cv2.solvePnP(points, pixels, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or t[2, 0] <= 0:
        raise ValueError('Board pose failed or lies behind the camera')
    result = np.eye(4);result[:3, :3] = cv2.Rodrigues(r)[0];result[:3, 3] = t.ravel()
    projected = cv2.projectPoints(points, r, t, K, dist)[0].reshape(-1, 2)
    return result, float(np.sqrt(np.mean(np.sum((projected-pixels)**2, axis=1))))


def charuco_pose(image_rgb, matrix, distortion, board_spec):
    """Indexed ChArUco detection -> metric pose; no device or filesystem access.

    Unlike an unindexed chessboard, IDs preserve the same board frame across
    rotated images. Caller owns exposure/FK synchronization and provenance.
    Requires optional OpenCV aruco support (not loaded by ordinary runtime use).
    """
    import cv2
    import numpy as np
    required = {'squares_x', 'squares_y', 'square_size_m', 'marker_size_m', 'dictionary'}
    if set(board_spec) != required:
        raise ValueError('Specify exact indexed board geometry and dictionary')
    nx, ny = board_spec['squares_x'], board_spec['squares_y']
    square, marker = board_spec['square_size_m'], board_spec['marker_size_m']
    dictionary = board_spec['dictionary']
    if (type(nx) is not int or type(ny) is not int or min(nx, ny) < 4
            or isinstance(square, bool) or isinstance(marker, bool)
            or not 0 < marker < square <= .2
            or dictionary not in ('DICT_4X4_50', 'DICT_5X5_100', 'DICT_6X6_250')):
        raise ValueError('Invalid metric ChArUco board specification')
    image = np.asarray(image_rgb)
    K, dist = np.asarray(matrix, dtype=float), np.asarray(distortion, dtype=float)
    if (image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8
            or K.shape != (3, 3) or not np.isfinite(K).all() or not np.isfinite(dist).all()
            or K[0, 0] <= 0 or K[1, 1] <= 0):
        raise ValueError('Require RGB uint8 image and calibrated finite intrinsics')
    board = cv2.aruco.CharucoBoard((nx, ny), square, marker,
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)))
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(
        cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
    if ids is None or len(ids) < 8 or board.checkCharucoCornersCollinear(ids):
        raise ValueError('At least eight non-collinear indexed board corners required')
    objects, pixels = board.matchImagePoints(corners, ids)
    ok, r, t = cv2.solvePnP(objects, pixels, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or t[2, 0] <= 0:
        raise ValueError('Indexed board pose failed or lies behind camera')
    projected = cv2.projectPoints(objects, r, t, K, dist)[0].reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((projected-pixels.reshape(-1, 2))**2, axis=1))))
    if rms > 1.5:raise ValueError('Indexed board reprojection RMS exceeds 1.5px')
    result = np.eye(4);result[:3, :3] = cv2.Rodrigues(r)[0];result[:3, 3] = t.ravel()
    return dict(camera_from_board=result.tolist(), reprojection_rms_px=rms,
                corner_ids=ids.ravel().tolist(), board_spec=dict(board_spec))


def fit(observations):
    import cv2
    import numpy as np
    from scipy.spatial.transform import Rotation
    data = observations
    if (data.get('schema') != 'openarm.camera-calibration.observations.v1'
            or data.get('mode') not in ('wrist', 'head') or data.get('units') != 'm'
            or data.get('joint_state_source') != 'measured_feedback'):
        raise ValueError('Require metric measured-feedback head/wrist calibration observations')
    for key in ('base_frame', 'tool_frame', 'camera_name', 'robot_model_sha256', 'intrinsics_source'):
        if not isinstance(data.get(key), str) or not data[key]:
            raise ValueError(f'Missing calibration provenance: {key}')
    samples = data['samples']
    if not isinstance(samples, list) or len(samples) < 8:
        raise ValueError('At least eight diverse synchronized board/robot poses are required')
    B = np.array([transform(s['base_from_tool']) for s in samples])
    C = np.array([transform(s['camera_from_board']) for s in samples])
    skew = np.array([abs(float(s['image_captured_at_s'])-float(s['robot_received_at_s'])) for s in samples])
    if not np.isfinite(skew).all() or (skew > .05).any():
        raise ValueError('Calibration observations must be paired within 50ms on the same clock')
    reprojection = np.array([s['reprojection_rms_px'] for s in samples], dtype=float)
    if not np.isfinite(reprojection).all() or (reprojection < 0).any() or (reprojection > 1.5).any():
        raise ValueError('Reject poor board detections; no automatic outlier deletion')
    turns = Rotation.from_matrix(B[0, :3, :3].T@B[:, :3, :3]).as_rotvec()
    singular = np.linalg.svd(turns, compute_uv=False)
    if singular[1] < .2 or np.max(np.linalg.norm(turns, axis=1)) < .35:
        raise ValueError('Robot rotations do not excite two axes; hand-eye fit is underconstrained')
    # Wrist: board stationary in base, camera attached to tool.
    # Head: board attached to tool, camera stationary in base; invert robot poses.
    G = B if data['mode'] == 'wrist' else np.linalg.inv(B)
    r, t = cv2.calibrateHandEye(list(G[:, :3, :3]), list(G[:, :3, 3]),
                               list(C[:, :3, :3]), list(C[:, :3, 3]), method=cv2.CALIB_HAND_EYE_PARK)
    X = np.eye(4);X[:3, :3] = r;X[:3, 3] = t.ravel();transform(X)
    constants = B@X@C if data['mode'] == 'wrist' else np.linalg.inv(B)@X@C
    rotations = Rotation.from_matrix(constants[:, :3, :3]);mean = rotations.mean()
    position_error = np.linalg.norm(constants[:, :3, 3]-np.mean(constants[:, :3, 3], axis=0), axis=1)
    rotation_error = (mean.inv()*rotations).magnitude()
    p_rms = float(np.sqrt(np.mean(position_error**2)))
    r_rms = float(np.sqrt(np.mean(rotation_error**2)))
    if p_rms > .005 or r_rms > .03:
        raise ValueError(f'Inconsistent hand-eye observations: {p_rms:.6f}m / {r_rms:.6f}rad RMS')
    return dict(schema='openarm.camera-calibration.estimate.v1', mode=data['mode'],
                camera_name=data['camera_name'], base_frame=data['base_frame'], tool_frame=data['tool_frame'],
                transform_name='tool_from_camera' if data['mode'] == 'wrist' else 'base_from_camera',
                transform=X.tolist(), units='m', samples=len(samples),
                rms_translation_m=p_rms, rms_rotation_rad=r_rms,
                max_reprojection_rms_px=float(reprojection.max()),
                max_pair_skew_s=float(skew.max()),
                robot_model_sha256=data['robot_model_sha256'], intrinsics_source=data['intrinsics_source'],
                calibration_status='estimated_not_independently_verified',
                note='No automatic hardware activation; validate held-out poses, frame conventions and image intrinsics before use')


def run(args):
    raw = Path(args.observations).read_bytes()
    result = fit(json.loads(raw))
    result['observations_sha256'] = hashlib.sha256(raw).hexdigest()
    with Path(args.output).open('x') as f:f.write(json.dumps(result, indent=2)+'\n')
