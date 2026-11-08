"""Pure geometry: undistorted pixels -> base-frame metres at a fixed plane."""
import math
import cv2
import numpy as np

def measure_polygon(pixels, homography):
    p = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    h = np.asarray(homography, dtype=np.float64)
    if len(p) < 4 or h.shape != (3, 3) or not np.isfinite(p).all() or not np.isfinite(h).all():
        raise ValueError('Invalid polygon or homography')
    if abs(np.linalg.det(h)) < 1e-12:
        raise ValueError('Singular homography')
    hp = np.column_stack([p, np.ones(len(p))]) @ h.T
    if np.any(np.abs(hp[:, 2]) < 1e-9):
        raise ValueError('Polygon crosses projection horizon')
    xy = (hp[:, :2] / hp[:, 2:]).astype(np.float32)
    rect = cv2.minAreaRect(xy)
    corners = cv2.boxPoints(rect)
    vectors = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(vectors, axis=1)
    index = int(np.argmin(lengths))
    width = float(lengths[index])
    if width <= 0:
        raise ValueError('Degenerate object outline')
    direction = vectors[index]
    return {'x': float(rect[0][0]), 'y': float(rect[0][1]),
            'width_m': width, 'length_m': float(max(lengths)),
            'closing_yaw_rad': math.atan2(float(direction[1]), float(direction[0]))}

def classify_brightness(bgr, mask, black_max=80, white_min=175):
    if not 0 <= black_max < white_min <= 255:
        raise ValueError('Brightness thresholds must be ordered')
    inner = cv2.erode(mask.astype(np.uint8), np.ones((5, 5), np.uint8))
    values = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)[inner > 0]
    if len(values) < 25:
        return 'uncertain'
    median = float(np.median(values))
    return 'black' if median <= black_max else 'white' if median >= white_min else 'uncertain'
