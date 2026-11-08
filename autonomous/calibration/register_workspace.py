"""Build a candidate robot mapping from surveyed workspace alignment.
All units millimetres. This does not activate calibration or move hardware.
"""
from pathlib import Path
import json,cv2,numpy as np
HERE=Path(__file__).resolve().parent
floor=json.loads((HERE/'workspace_floor_mapping_unvalidated.json').read_text())
lens=json.loads((HERE/'captures/combined_batches_1_2_3/intrinsics_unvalidated.json').read_text())
k=np.array(lens['camera_matrix'],float);d=np.array(lens['dist_coeffs'],float)
uv=np.array(floor['corners_raw_px'],float)
# x forward from base toward rectangle; y left looking forward, z up.
# Image A/D are nearest corners. A is +y, D is -y in this camera view.
world=np.array([[200,110,0],[350,110,0],[350,-110,0],[200,-110,0]],float)
ok,rv,tv=cv2.solvePnP(world,uv,k,d,flags=cv2.SOLVEPNP_ITERATIVE)
if not ok:raise RuntimeError('Camera pose fit failed')
r,_=cv2.Rodrigues(rv);camera=(-r.T@tv).ravel()
if camera[2]<=20 or np.min((r@world.T+tv)[2])<=0:raise RuntimeError('Invalid camera pose/plane orientation')
projected,_=cv2.projectPoints(world,rv,tv,k,d);errors=np.linalg.norm(projected.reshape(-1,2)-uv,axis=1)
h=k@np.column_stack([r[:,0],r[:,1],tv.ravel()+20*r[:,2]])
inverse=np.linalg.inv(h);inverse/=inverse[2,2]
result={**lens,'validated':False,'physical_control_enabled':False,'coordinate_frame':'robot_base','plane_height_mm':20,'pixel_to_plane_mm':inverse.tolist(),'robot_corner_coordinates_mm':world.tolist(),'corner_pixels_raw':uv.tolist(),'camera_position_robot_mm':camera.tolist(),'rvec':rv.ravel().tolist(),'tvec_mm':tv.ravel().tolist(),'floor_corner_fit_errors_px':errors.tolist(),'registration_basis':'User confirmed nearest 220 mm edge centred 200 mm forward of base-axis floor point; 150 mm rectangle depth','limitations':['Lens model provisional; no independent 20 mm-plane metric validation','Forward survey axis must be checked against physical arm/URDF forward direction','Camera mount/lens/resolution must remain unchanged','Bin coordinates unresolved; not enabled for motor execution']}
(HERE/'robot_top_plane_candidate.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'camera_position_robot_mm':camera.tolist(),'max_corner_fit_error_px':float(errors.max()),'candidate_only':True},indent=2))
