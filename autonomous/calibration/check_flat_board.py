"""Check a held-out checkerboard frame in board-local coordinates; no robot registration."""
from pathlib import Path
import cv2,numpy as np,json
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
image=cv2.imread(str(HERE/'flat_board_reference.jpg'))
lens=json.loads((HERE/'captures/combined_batches_1_2_3/intrinsics_unvalidated.json').read_text())
gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
found,corners=cv2.findChessboardCorners(gray,(7,5))
if not found:raise SystemExit('Whole checkerboard grid not detected')
corners=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,30,.001))
k=np.asarray(lens['camera_matrix']);d=np.asarray(lens['dist_coeffs'])
uv=cv2.undistortPoints(corners,k,d,P=k).reshape(-1,2)
xy=np.mgrid[0:7,0:5].T.reshape(-1,2).astype(float)*25
train=np.arange(35)%2==0;test=~train
h,_=cv2.findHomography(uv[train],xy[train],0)
pred=cv2.perspectiveTransform(uv.reshape(-1,1,2),h).reshape(-1,2)
err=np.linalg.norm(pred-xy,axis=1)
report={'validated':False,'physical_control_enabled':False,'coordinate_frame':'checkerboard_local','square_size_mm_assumed_from_print':25,'frame_is_separate_from_lens_fit':True,'plane_height_above_floor_mm':None,'corners_detected':35,'fitting_points':int(train.sum()),'held_out_points':int(test.sum()),'held_out_max_error_mm':float(err[test].max()),'held_out_rms_error_mm':float(np.sqrt(np.mean(err[test]**2))),'undistorted_pixel_to_board_mm':h.tolist(),'limitations':['Within-image grid consistency, not independent robot positioning accuracy','Board placement, orientation and backing thickness not registered to robot','Cube-top height mapping not established','Lens model remains provisional']}
(HERE/'flat_board_check.json').write_text(json.dumps(report,indent=2)+'\n')
cv2.drawChessboardCorners(image,(7,5),corners,True)
cv2.imwrite(str(HERE/'flat_board_detected.jpg'),image)
print(json.dumps(report,indent=2))
