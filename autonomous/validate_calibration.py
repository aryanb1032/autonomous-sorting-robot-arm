#!/usr/bin/env python3
"""Validate a fitted camera mapping against independent, measured top-plane points.

Input points JSON: {"pixels": [[u,v],...], "plane_xy_mm": [[x,y],...]}
Use RAW/distorted image pixels at the calibration image resolution. Use points
not used to fit the mapping. This checks mapping accuracy, not robot motion.
"""
import argparse,json,sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'opencv_boxes'))
from box_vision import transform,load_calibration

def validate(c,pairs,limit=3.):
 if not np.isfinite(limit) or limit<=0:raise ValueError('Invalid error limit')
 pixels=np.asarray(pairs['pixels'],float);truth=np.asarray(pairs['plane_xy_mm'],float)
 if pixels.shape!=truth.shape or pixels.ndim!=2 or pixels.shape[1]!=2 or len(pixels)<4:raise ValueError('Need four or more independent point pairs')
 if not np.isfinite(pixels).all() or not np.isfinite(truth).all():raise ValueError('Nonfinite point')
 if np.linalg.matrix_rank(truth-truth.mean(axis=0))<2:raise ValueError('Validation points must span the workspace')
 k=np.asarray(c['camera_matrix'],float);d=np.asarray(c['dist_coeffs'],float)
 corrected=cv2.undistortPoints(pixels.reshape(-1,1,2),k,d,P=k).reshape(-1,2)
 predicted=transform(corrected,c['pixel_to_plane_mm'])
 errors=np.linalg.norm(predicted-truth,axis=1)
 return {'passed':bool(np.max(errors)<=limit),'errors_mm':errors.tolist(),'max_error_mm':float(np.max(errors)),
         'rms_error_mm':float(np.sqrt(np.mean(errors**2))),'count':len(errors),'limit_mm':limit}

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--calibration',type=Path,required=True);p.add_argument('--points',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--limit-mm',type=float,default=3.)
 a=p.parse_args();c=json.loads(a.calibration.read_text());pairs=json.loads(a.points.read_text())
 # Input fit may be unvalidated. Validate matrices with the detector's own checks.
 c['validated']=True
 import tempfile
 with tempfile.NamedTemporaryFile(mode='w',suffix='.json') as f:
  json.dump(c,f);f.flush();load_calibration(f.name)
 result=validate(c,pairs,a.limit_mm);print(json.dumps(result,indent=2))
 if not result['passed']:raise SystemExit('Independent error exceeds limit; output not written')
 c.update(validated=True,coordinate_frame='robot_base',plane_height_mm=20,independent_validation=result,
          validation_points=str(a.points.resolve()))
 a.output.write_text(json.dumps(c,indent=2)+'\n')
 print('Saved verified mapping. Stream latency, workspace, bins and aperture remain separate checks.')
if __name__=='__main__':main()
