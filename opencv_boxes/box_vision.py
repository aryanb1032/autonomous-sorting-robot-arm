"""Black/white box detection; optional calibrated plane measurements.

Designed for separated, matte boxes on a uniform mid-grey background.
No servo commands are sent. The camera must be at its calibrated observation pose.
"""
import argparse
import json
import math
import time
import urllib.request
from pathlib import Path
import cv2
import numpy as np

DEFAULTS = dict(black_max=70, white_min=180, white_max_saturation=65,
                min_area_px=100, max_area_fraction=.35, min_solidity=.92,
                expected_side_mm=20., side_tolerance_mm=2., roi=None)

def load_calibration(path):
    c=json.loads(Path(path).read_text())
    if c.get('validated') is not True:
        raise ValueError('Calibration must be independently checked and marked validated=true')
    for key,shape in [('camera_matrix',(3,3)),('pixel_to_plane_mm',(3,3))]:
        a=np.asarray(c[key],dtype=float)
        if a.shape!=shape or not np.isfinite(a).all() or abs(np.linalg.det(a))<1e-12:
            raise ValueError(f'Invalid {key}')
    d=np.asarray(c['dist_coeffs'],dtype=float).reshape(-1)
    if d.size not in (4,5,8,12,14) or not np.isfinite(d).all():raise ValueError('Invalid distortion coefficients')
    size=c['image_size']
    if len(size)!=2 or any(not isinstance(v,int) or v<=0 for v in size):raise ValueError('Invalid image_size')
    return c

def dimensions(points):
    rect=cv2.minAreaRect(np.asarray(points,dtype=np.float32).reshape(-1,2))
    corners=cv2.boxPoints(rect)
    edges=np.roll(corners,-1,axis=0)-corners
    lengths=np.linalg.norm(edges,axis=1);i=int(np.argmin(lengths))
    if lengths[i]<=0:raise ValueError('Degenerate contour')
    return dict(center=tuple(float(x) for x in rect[0]),
                width=float(lengths[i]),length=float(max(lengths)),
                closing_angle_deg=float(math.degrees(math.atan2(edges[i,1],edges[i,0]))%180))

def transform(points,h):
    p=np.asarray(points,dtype=float).reshape(-1,2)
    hp=np.column_stack((p,np.ones(len(p))))@np.asarray(h,dtype=float).T
    if not np.isfinite(hp).all() or np.any(np.abs(hp[:,2])<1e-9):
        raise ValueError('Invalid projection')
    if np.min(hp[:,2])*np.max(hp[:,2])<0:raise ValueError('Contour crosses projection horizon')
    return hp[:,:2]/hp[:,2:]

def detect(image, settings=None, calibration=None):
    cfg={**DEFAULTS,**(settings or {})}
    if not 0<=cfg['black_max']<cfg['white_min']<=255:raise ValueError('Thresholds must be ordered')
    if not 0<=cfg['white_max_saturation']<=255:raise ValueError('Invalid saturation threshold')
    if cfg['min_area_px']<=0 or not 0<cfg['max_area_fraction']<=1 or not 0<cfg['min_solidity']<=1:
        raise ValueError('Invalid area/shape thresholds')
    if cfg['expected_side_mm']<=0 or cfg['side_tolerance_mm']<0:raise ValueError('Invalid reference side')
    height,width=image.shape[:2]
    view=image.copy()
    if calibration:
        if [width,height]!=calibration['image_size']:raise ValueError('Image resolution differs from calibration')
        view=cv2.undistort(view,np.asarray(calibration['camera_matrix']),np.asarray(calibration['dist_coeffs']))
    x,y,w,h=cfg['roi'] or [0,0,width,height]
    if any(not isinstance(v,int) for v in [x,y,w,h]) or x<0 or y<0 or w<=0 or h<=0 or x+w>width or y+h>height:
        raise ValueError('ROI must fit inside the image')
    crop=view[y:y+h,x:x+w];hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    masks={'black':(hsv[:,:,2]<=cfg['black_max']).astype(np.uint8)*255,
           'white':((hsv[:,:,2]>=cfg['white_min'])&(hsv[:,:,1]<=cfg['white_max_saturation'])).astype(np.uint8)*255}
    objects=[];kernel=np.ones((3,3),np.uint8)
    for colour,raw in masks.items():
        mask=cv2.morphologyEx(raw,cv2.MORPH_OPEN,kernel)
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area=cv2.contourArea(contour)
            if not cfg['min_area_px']<=area<=w*h*cfg['max_area_fraction']:continue
            bx,by,bw,bh=cv2.boundingRect(contour)
            if bx<=1 or by<=1 or bx+bw>=w-1 or by+bh>=h-1:continue
            hull_area=cv2.contourArea(cv2.convexHull(contour))
            if hull_area<=0 or area/hull_area<cfg['min_solidity']:continue
            polygon=cv2.approxPolyDP(contour,.025*cv2.arcLength(contour,True),True)
            if len(polygon)!=4 or not cv2.isContourConvex(polygon):continue
            full=polygon.reshape(-1,2)+np.array([x,y])
            px=dimensions(full)
            obj={'colour':colour,'center_px':list(px['center']),
                 'width_px':px['width'],'length_px':px['length'],
                 'outline_px':full.tolist(),'measurement_unit':'px',
                 'width_mm':None,'length_mm':None,'center_plane_mm':None,
                 'closing_angle_plane_deg':None,'matches_20mm_side':None}
            if calibration:
                metric=dimensions(transform(full,calibration['pixel_to_plane_mm']))
                obj.update(measurement_unit='mm',width_mm=metric['width'],length_mm=metric['length'],
                           center_plane_mm=list(metric['center']),closing_angle_plane_deg=metric['closing_angle_deg'],
                           matches_20mm_side=min(abs(metric['width']-cfg['expected_side_mm']),abs(metric['length']-cfg['expected_side_mm']))<=cfg['side_tolerance_mm'])
            objects.append(obj)
    objects.sort(key=lambda o:(o['colour'],o['center_px'][0],o['center_px'][1]))
    counts={'black':0,'white':0}
    for o in objects:
        counts[o['colour']]+=1;o['frame_label']=f"{o['colour']}_{counts[o['colour']]}"
    report=dict(timestamp_unix=time.time(),calibrated=calibration is not None,
                expected_side_mm=cfg['expected_side_mm'],side_tolerance_mm=cfg['side_tolerance_mm'],
                counts=counts,objects=objects,
                note='Frame labels are not tracking IDs. Measurements require the calibrated pose and box-top plane.')
    return report,view,masks

def annotate(view,report):
    out=view.copy()
    for o in report['objects']:
        colour=(0,180,255) if o['colour']=='black' else (50,220,50)
        pts=np.asarray(o['outline_px'],np.int32)
        cv2.polylines(out,[pts],True,colour,2)
        if report['calibrated']:
            sizes=f"{o['width_mm']:.1f} x {o['length_mm']:.1f} mm"
            sizes+=' | side OK' if o['matches_20mm_side'] else ' | CHECK SIZE'
        else:sizes=f"{o['width_px']:.0f} x {o['length_px']:.0f} px"
        label=o['frame_label']+' '+sizes
        tx=max(5,min(int(pts[:,0].min()),out.shape[1]-10));ty=max(45,int(pts[:,1].min())-8)
        cv2.putText(out,label,(tx,ty),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,0,0),3,cv2.LINE_AA)
        cv2.putText(out,label,(tx,ty),cv2.FONT_HERSHEY_SIMPLEX,.42,colour,1,cv2.LINE_AA)
    text=f"Black: {report['counts']['black']}  White: {report['counts']['white']}  "+('CALIBRATED PLANE' if report['calibrated'] else 'PIXELS ONLY - CALIBRATION REQUIRED')
    cv2.rectangle(out,(0,0),(out.shape[1],29),(30,30,30),-1)
    cv2.putText(out,text,(8,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
    return out

def snapshot(url):
    with urllib.request.urlopen(url,timeout=3) as response:data=response.read(4*1024*1024)
    image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
    if image is None:raise ValueError('Camera did not return a valid image')
    return image

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    src=parser.add_mutually_exclusive_group(required=True)
    src.add_argument('--stream',help='Phone MJPEG/RTSP stream URL (not the camera web page)');src.add_argument('--image',help='Still image path');src.add_argument('--url',help='ESP32 HTTP JPEG /capture URL');src.add_argument('--camera',type=int,help='USB camera index')
    parser.add_argument('--settings');parser.add_argument('--calibration');parser.add_argument('--output',default='vision_results')
    parser.add_argument('--headless',action='store_true');parser.add_argument('--frames',type=int,default=0,help='Live frame limit; 0 until Ctrl+C or q')
    args=parser.parse_args()
    cfg=json.loads(Path(args.settings).read_text()) if args.settings else None
    cal=load_calibration(args.calibration) if args.calibration else None
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    (output/'latest.json').write_text(json.dumps({'status':'connecting','objects':[],'physical_control_enabled':False,'timestamp_unix':time.time()})+'\n')
    cap=cv2.VideoCapture(args.camera) if args.camera is not None else None
    if args.stream:
        cap=cv2.VideoCapture(args.stream,cv2.CAP_FFMPEG,[cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,5000,cv2.CAP_PROP_READ_TIMEOUT_MSEC,3000])
    if cap is not None and not cap.isOpened():raise SystemExit('Cannot open camera')
    processed=0
    try:
        while True:
            try:
                if args.image:image=cv2.imread(args.image)
                elif args.url:image=snapshot(args.url)
                else:
                    ok,image=cap.read()
                    if not ok:raise RuntimeError('Camera stream ended or capture failed')
                if image is None:raise ValueError('Cannot read input image')
                cv2.imwrite(str(output/'latest_raw.jpg'),image)
                report,view,masks=detect(image,cfg,cal)
                report.update(status='frame_processed',physical_control_enabled=False,frame_index=processed,timestamp_basis='PC processing time; camera capture time unknown')
                overlay=annotate(view,report)
                cv2.imwrite(str(output/'latest.jpg'),overlay)
                temp=output/'latest.tmp';temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');temp.replace(output/'latest.json')
                for colour,mask in masks.items():cv2.imwrite(str(output/f'{colour}_mask.png'),mask)
                print(json.dumps({'counts':report['counts'],'calibrated':report['calibrated']}),flush=True)
                processed+=1
                if not args.headless:
                    cv2.imshow('Black and white box sizes - q to quit',overlay)
                    if cv2.waitKey(1)&255==ord('q'):break
                if args.image or (args.frames>0 and processed>=args.frames):break
                if not args.stream:time.sleep(.2)
            except (OSError,RuntimeError,ValueError) as exc:
                # Do not leave a previous detection marked as current after capture fails.
                (output/'latest.json').write_text(json.dumps({'error':str(exc),'objects':[],'timestamp_unix':time.time()})+'\n')
                if args.image or args.stream:raise
                print('Capture failed:',exc,flush=True);time.sleep(1)
    except KeyboardInterrupt:pass
    finally:
        if cap is not None:cap.release()
        if not args.headless:cv2.destroyAllWindows()

if __name__=='__main__':main()
