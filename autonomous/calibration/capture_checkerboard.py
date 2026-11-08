#!/usr/bin/env python3
"""Automatically collect varied checkerboard views from the fixed phone.
No robot interface. Successful collection produces an UNVALIDATED lens fit only.
"""
import argparse,json,subprocess,sys,time
from pathlib import Path
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[2]

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--seconds',type=float,default=180)
 p.add_argument('--headless',action='store_true')
 a=p.parse_args()
 if not 0<a.seconds<=1800:p.error('seconds must be between 0 and 1800')
 cfg=json.loads((ROOT/'autonomous/camera_config.json').read_text())
 out=Path(__file__).resolve().parent/'captures'/time.strftime('%Y%m%d-%H%M%S');out.mkdir(parents=True)
 cap=cv2.VideoCapture(cfg['stream_url'],cv2.CAP_FFMPEG,[cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,5000,cv2.CAP_PROP_READ_TIMEOUT_MSEC,3000])
 if not cap.isOpened():raise SystemExit('Cannot open phone stream')
 saved=[];frames=0;size=None;start=time.monotonic();last_preview=0.;reason='time limit reached'
 try:
  while time.monotonic()-start<a.seconds:
   ok,im=cap.read()
   if not ok:reason='stream failed';break
   frames+=1
   if time.monotonic()-last_preview>=2:
    cv2.imwrite(str(out/'latest_capture.jpg'),im);last_preview=time.monotonic()
   if size is None:size=list(im.shape[1::-1])
   if list(im.shape[1::-1])!=size:reason='resolution changed';break
   gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
   found,corners=cv2.findChessboardCorners(gray,(7,5),cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_FAST_CHECK)
   if found:
    corners=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,30,.001))
    pts=corners.reshape(-1,2)
    # Reject repeated near-identical frames; this is not a calibration quality test.
    distinct=all(float(np.sqrt(np.mean((pts-old)**2)))>18 for old in saved)
    if distinct:
     saved.append(pts.copy());cv2.imwrite(str(out/f'view_{len(saved):02d}.png'),im)
     print(f'Accepted view {len(saved)}/15. Change board tilt, position and distance.',flush=True)
    cv2.drawChessboardCorners(im,(7,5),corners,found)
   if not a.headless:
    cv2.putText(im,f'Checkerboard views: {len(saved)}/15 - move BOARD, keep PHONE fixed - Q exits',(15,35),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2)
    cv2.imshow('Lens calibration capture',im)
    if cv2.waitKey(1)&255==ord('q'):reason='operator exit';break
   if len(saved)>=15:reason='collection complete';break
 finally:
  cap.release()
  if not a.headless:cv2.destroyAllWindows()
 report={'frames_checked':frames,'checkerboard_views_saved':len(saved),'image_size':size,'stream_url':cfg['stream_url'],'reason':reason,'validated':False,'physical_control_enabled':False}
 (out/'capture_report.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2),flush=True)
 if len(saved)<15:raise SystemExit('Need 15 varied views of the 7 x 5 inner-corner board. No calibration created.')
 subprocess.run([sys.executable,str(ROOT/'opencv_boxes/calibrate.py'),'intrinsics','--images',str(out/'view_*.png'),'--cols','7','--rows','5','--square-mm','25','--output',str(out/'intrinsics_unvalidated.json')],check=True)
 print('Lens fit saved for inspection. Robot/plane registration and independent checks still required. Active configuration unchanged.')
if __name__=='__main__':main()
