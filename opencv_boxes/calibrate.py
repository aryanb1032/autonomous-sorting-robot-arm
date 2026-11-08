"""Two-stage calibration for one stationary observation pose and box-top plane."""
import argparse
import glob
import json
from pathlib import Path
import cv2
import numpy as np

def intrinsics(args):
    if args.cols<2 or args.rows<2 or args.square_mm<=0:raise ValueError('Invalid checkerboard dimensions')
    grid=np.zeros((args.cols*args.rows,3),np.float32)
    grid[:,:2]=np.mgrid[0:args.cols,0:args.rows].T.reshape(-1,2)*args.square_mm
    objects=[];images=[];size=None
    for filename in sorted(glob.glob(args.images)):
        image=cv2.imread(filename)
        if image is None:continue
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        shape=gray.shape[::-1]
        if size is not None and shape!=size:raise ValueError('Mixed image resolutions')
        size=shape
        found,corners=cv2.findChessboardCorners(gray,(args.cols,args.rows))
        if not found:continue
        refined=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,.001))
        objects.append(grid.copy());images.append(refined)
    if len(images)<10:raise ValueError(f'Only {len(images)} checkerboards detected; capture at least 10 varied views')
    rms,k,d,rvecs,tvecs=cv2.calibrateCamera(objects,images,size,None,None)
    if not np.isfinite(rms) or not np.isfinite(k).all() or not np.isfinite(d).all():raise ValueError('Invalid calibration result')
    out=dict(image_size=list(size),camera_matrix=k.tolist(),dist_coeffs=d.ravel().tolist(),reprojection_rms_px=float(rms),views_used=len(images))
    Path(args.output).write_text(json.dumps(out,indent=2)+'\n')
    print('Reprojection RMS (pixels):',rms,'; inspect coverage and independent accuracy before use.')

def plane(args):
    lens=json.loads(Path(args.intrinsics).read_text());pairs=json.loads(Path(args.points).read_text())
    src=np.asarray(pairs['pixels'],dtype=np.float64);dst=np.asarray(pairs['plane_xy_mm'],dtype=np.float64)
    if src.shape!=dst.shape or src.ndim!=2 or src.shape[1]!=2 or len(src)<6 or not np.isfinite(src).all() or not np.isfinite(dst).all():
        raise ValueError('Need at least six matching finite pixel and plane coordinates')
    k=np.asarray(lens['camera_matrix']);d=np.asarray(lens['dist_coeffs'])
    corrected=cv2.undistortPoints(src.reshape(-1,1,2),k,d,P=k)
    h,inliers=cv2.findHomography(corrected,dst,cv2.RANSAC,1.)
    if h is None or abs(np.linalg.det(h))<1e-12 or int(inliers.sum())<6:raise ValueError('Insufficient consistent noncollinear calibration points')
    predicted=cv2.perspectiveTransform(corrected,h).reshape(-1,2)
    out={**lens,'pixel_to_plane_mm':h.tolist(),'fit_errors_mm':np.linalg.norm(predicted-dst,axis=1).tolist(),
         'inliers':inliers.ravel().tolist(),'validated':False,
         'plane_description':'Box-top plane, camera at observation pose; fill measured height and pose details'}
    Path(args.output).write_text(json.dumps(out,indent=2)+'\n')
    print('Saved unvalidated calibration. Errors (mm):',out['fit_errors_mm'])
    print('Check independent known lengths across the workspace before setting validated=true.')

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='mode',required=True)
    a=sub.add_parser('intrinsics');a.add_argument('--images',required=True);a.add_argument('--cols',type=int,required=True);a.add_argument('--rows',type=int,required=True);a.add_argument('--square-mm',type=float,required=True);a.add_argument('--output',required=True);a.set_defaults(run=intrinsics)
    a=sub.add_parser('plane');a.add_argument('--intrinsics',required=True);a.add_argument('--points',required=True);a.add_argument('--output',required=True);a.set_defaults(run=plane)
    args=p.parse_args();args.run(args)

if __name__=='__main__':main()
