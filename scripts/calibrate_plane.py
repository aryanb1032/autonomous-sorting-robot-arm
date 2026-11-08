"""Fit an undistorted-pixel -> robot-base metre homography from measured points.
Input JSON: pixels, base_xy_m, image_size, camera_matrix, dist_coeffs.
Use points at box-top height with camera at the exact observation pose.
At least six points; reserve additional independent points to validate accuracy.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
p=argparse.ArgumentParser();p.add_argument('points');p.add_argument('output');args=p.parse_args()
c=json.loads(Path(args.points).read_text());src=np.asarray(c['pixels'],dtype=np.float64);dst=np.asarray(c['base_xy_m'],dtype=np.float64)
if src.shape!=dst.shape or src.ndim!=2 or src.shape[1]!=2 or len(src)<6 or not np.isfinite(src).all() or not np.isfinite(dst).all():raise SystemExit('Need at least six matching finite 2D points')
src=cv2.undistortPoints(src.reshape(-1,1,2),np.array(c['camera_matrix']),np.array(c['dist_coeffs']),P=np.array(c['camera_matrix'])).reshape(-1,2)
h,mask=cv2.findHomography(src,dst,cv2.RANSAC,0.002)
if h is None or mask.sum()<6:raise SystemExit('Calibration failed: need six consistent inliers')
pred=cv2.perspectiveTransform(src.reshape(-1,1,2),h).reshape(-1,2)
errors=np.linalg.norm(pred-dst,axis=1)
out={k:c[k] for k in ['image_size','camera_matrix','dist_coeffs']}
out.update(pixel_to_base_homography=h.tolist(),fit_errors_m=errors.tolist(),inliers=mask.ravel().tolist(),calibrated=False)
Path(args.output).write_text(json.dumps(out,indent=2)+'\n')
print('Fit errors in mm:',errors*1000)
print('Validate on independent points, then merge into vision.template.json. calibrated remains false.')
