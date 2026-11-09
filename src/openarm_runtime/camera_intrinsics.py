"""Offline ChArUco/checkerboard pinhole calibration; no device or file access."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Checkerboard:
    squares_x: int
    squares_y: int
    square_size_m: float

    def getChessboardCorners(self):
        """Per-image grid coordinates, NOT globally indexed physical corners."""
        import numpy as np
        points = np.zeros(((self.squares_x-1)*(self.squares_y-1), 3), np.float32)
        points[:, :2] = np.mgrid[0:self.squares_x-1, 0:self.squares_y-1].T.reshape(-1, 2)
        return points * self.square_size_m


def board(spec):
    import cv2
    if isinstance(spec, dict) and spec.get('type') == 'checkerboard':
        if set(spec) != {'type', 'squares_x', 'squares_y', 'square_size_m'}:
            raise ValueError('普通棋盘格只填写方格总数与实测格长；不填写 marker 或字典')
        nx, ny, square = (spec[k] for k in ('squares_x', 'squares_y', 'square_size_m'))
        if (type(nx) is not int or type(ny) is not int or not 4 <= nx <= 40 or not 4 <= ny <= 40
                or type(square) not in (int, float) or not math.isfinite(square) or not 0 < square <= .2):
            raise ValueError('无效的普通棋盘格规格（方格总数，格长单位为米）')
        return Checkerboard(nx, ny, float(square))
    if isinstance(spec, dict) and spec.get('type') == 'charuco':
        spec = {k:v for k,v in spec.items() if k != 'type'}
    if not isinstance(spec, dict) or set(spec) != {
            'squares_x', 'squares_y', 'square_size_m', 'marker_size_m', 'dictionary'}:
        raise ValueError('填写完整 ChArUco 板规格；不从图像猜测尺寸')
    nx, ny, square, marker, dictionary = (spec[k] for k in
        ('squares_x', 'squares_y', 'square_size_m', 'marker_size_m', 'dictionary'))
    if (type(nx) is not int or type(ny) is not int or not 4 <= nx <= 40 or not 4 <= ny <= 40
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in (square, marker))
            or not 0 < marker < square <= .2
            or dictionary not in ('DICT_4X4_50', 'DICT_5X5_100', 'DICT_6X6_250')):
        raise ValueError('无效的 ChArUco 板规格（单位为米）')
    if not hasattr(getattr(cv2, 'aruco', None), 'CharucoDetector'):
        raise RuntimeError('OpenCV 缺少 aruco.CharucoDetector；安装 calibration extra')
    return cv2.aruco.CharucoBoard((nx, ny), square, marker,
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)))


def detect(image_rgb, spec):
    import cv2
    import numpy as np
    image = np.asarray(image_rgb)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('需要 RGB uint8 原始尺寸图像')
    target = board(spec)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    if isinstance(target, Checkerboard):
        found, pixels = cv2.findChessboardCornersSB(gray,
            (target.squares_x-1, target.squares_y-1), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        expected = (target.squares_x-1)*(target.squares_y-1)
        if not found or pixels is None or len(pixels) != expected:
            raise ValueError(f'未检测到完整 {target.squares_x-1}×{target.squares_y-1} 内角点；请露出整块棋盘格')
        objects = target.getChessboardCorners()
        ids = np.arange(expected)
        frame_status = 'per_image_grid_ambiguous_for_extrinsics'
    else:
        corners, ids, _, _ = cv2.aruco.CharucoDetector(target).detectBoard(gray)
        if ids is None or len(ids) < 8 or target.checkCharucoCornersCollinear(ids):
            raise ValueError('至少需要八个不共线的 ChArUco 角点')
        objects, pixels = target.matchImagePoints(corners, ids)
        frame_status = 'indexed_charuco_board'
    xy = pixels.reshape(-1, 2)
    span = np.ptp(xy, axis=0)
    if min(span) < 20:
        raise ValueError('板在图像中过小，角点跨度不足 20 px')
    return dict(objects=objects.reshape(-1, 3).tolist(), pixels=xy.tolist(),
                corner_ids=ids.ravel().tolist(),
                board_frame_status=frame_status,
                center_normalized=(xy.mean(axis=0) / [image.shape[1], image.shape[0]]).tolist(),
                coverage_fraction=float(np.prod(span) / (image.shape[0]*image.shape[1])),
                sharpness=float(cv2.Laplacian(gray, cv2.CV_64F).var()))


def solve(samples, size):
    """Use only predeclared fit views; held-out images estimate pose, never K.

    Supports the ordinary pinhole + five-coefficient distortion model only.
    Held-out reprojection is an image check, NOT independent metric verification.
    """
    import cv2
    import numpy as np
    fit = [s for s in samples if s['role'] == 'fit']
    held = [s for s in samples if s['role'] == 'validation']
    if len(fit) < 8 or len(held) < 3:
        raise ValueError('至少需要 8 张拟合图像和 3 张预先指定的留出图像')
    def arrays(s):
        return (np.asarray(s['detection']['objects'], np.float32),
                np.asarray(s['detection']['pixels'], np.float32))
    objects, pixels = zip(*(arrays(s) for s in fit))
    rms, K, dist, rotations, translations = cv2.calibrateCamera(
        objects, pixels, tuple(size), None, None)
    if (not np.isfinite(K).all() or not np.isfinite(dist).all() or not math.isfinite(rms)
            or min(K[0, 0], K[1, 1]) <= 0):
        raise ValueError('内参求解无效')
    if any(not np.isfinite(t).all() or t[2, 0] <= 0 for t in translations):
        raise ValueError('拟合板位姿无效或位于相机背面')
    # Planar fronto-parallel / single-axis captures can fit a misleading K.
    normals = np.array([cv2.Rodrigues(r)[0][:, 2] for r in rotations])
    excitation = np.linalg.svd(normals - normals.mean(axis=0), compute_uv=False)
    if excitation[1] < .1:
        raise ValueError('姿态覆盖不足：请绕两个方向倾斜标定板')
    all_pixels = np.concatenate(pixels).reshape(-1, 2) / np.array(size)
    lower, upper = all_pixels.min(axis=0), all_pixels.max(axis=0)
    issues = []
    if (lower > .15).any() or (upper < .85).any():
        issues.append('拟合角点未充分覆盖图像边缘；畸变估计可能不可靠')
    errors = []
    for sample in samples:
        obj, pix = arrays(sample)
        if sample['role'] == 'fit':
            i = fit.index(sample)
            r, t = rotations[i], translations[i]
        else:
            ok, r, t = cv2.solvePnP(obj, pix, K, dist)
            if not ok or t[2, 0] <= 0:
                raise ValueError('留出图像板位姿求解失败')
        projected = cv2.projectPoints(obj, r, t, K, dist)[0].reshape(-1, 2)
        error = float(np.sqrt(np.mean(np.sum((projected-pix)**2, axis=1))))
        if not math.isfinite(error):
            raise ValueError('重投影检查无效')
        errors.append(dict(id=sample['id'], role=sample['role'], rms_px=error))
    validation_rms = float(np.sqrt(np.mean([e['rms_px']**2 for e in errors if e['role']=='validation'])))
    if rms > 1.5 or validation_rms > 1.5:
        issues.append('重投影 RMS 超过 1.5 px；检查图像、板规格和相机模型')
    return dict(schema='openarm.camera-intrinsics.estimate.v1', camera_model='pinhole',
                distortion_model='opencv_5', image_size=list(size), matrix=K.tolist(),
                distortion=dist.ravel().tolist(), fit_rms_px=float(rms),
                validation_rms_px=validation_rms,
                fit_corner_bounds_normalized=[lower.tolist(), upper.tolist()], quality_issues=issues,
                per_image=errors, fit_ids=[s['id'] for s in fit],
                validation_ids=[s['id'] for s in held],
                calibration_status='estimated_not_independently_verified',
                validation_kind='held_out_reprojection_with_independent_board_pose',
                note='留出重投影不是物理标定验收；无自动安装，无异常值自动删除')
