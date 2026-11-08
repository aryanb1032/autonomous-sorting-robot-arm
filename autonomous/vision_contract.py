"""Validated observation interface; contains no robot/serial execution code."""
import json,math,time,sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"opencv_boxes"))
from box_vision import load_calibration

def issues(config):
 errors=[]
 for key in ['calibration_validated','stream_latency_validated','gripper_aperture_calibration_validated']:
  if config.get(key) is not True:errors.append(key+' is not validated')
 path=config.get('calibration_file')
 if not path or not Path(path).is_file():errors.append('camera calibration file missing')
 else:
  try:
   c=load_calibration(path)
   if not c.get('validated'):errors.append('calibration file not validated')
   if c.get('coordinate_frame')!='robot_base' or c.get('plane_height_mm')!=20:errors.append('calibration must map the 20 mm top plane into robot_base')
  except (OSError,ValueError,KeyError,TypeError):errors.append('invalid calibration file')
 e=config.get('measured_independent_error_mm')
 if not isinstance(e,(int,float)) or not math.isfinite(e) or not 0<=e<=config.get('max_independent_error_mm',3):errors.append('independent position error missing or excessive')
 poly=config.get('pickup_polygon_mm',[])
 if len(poly)<4 or any(len(p)!=2 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in p) for p in poly):errors.append('measured pickup polygon missing/invalid')
 for color in ['black','white']:
  b=config.get('bins',{}).get(color,{})
  center=b.get('center_mm')
  if not isinstance(center,list) or len(center)!=2 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in center):errors.append(color+' bin centre missing/invalid')
  for key in ['rim_height_mm','inside_diameter_mm']:
   v=b.get(key)
   if not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0:errors.append(color+' '+key+' missing/invalid')
 return errors

class Observations:
 def __init__(self,config):self.config=config;self.last_frame=-1
 def decode(self,report,now=None):
  now=time.time() if now is None else now
  if issues(self.config):raise ValueError('Calibration/configuration not ready')
  stamp=report.get('timestamp_unix')
  if not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or not 0<=now-stamp<=self.config['max_result_age_s']:raise ValueError('Stale/invalid observation timestamp')
  frame=report.get('frame_index')
  if not isinstance(frame,int) or frame<=self.last_frame:raise ValueError('Repeated/out-of-order frame; reset session on camera restart')
  if report.get('status')!='frame_processed' or report.get('calibrated') is not True or report.get('error'):raise ValueError('Uncalibrated/failed frame')
  self.last_frame=frame;result=[];poly=np.array(self.config['pickup_polygon_mm'],np.float32)
  for obj in report.get('objects',[]):
   xy=obj.get('center_plane_mm');w=obj.get('width_mm');h=obj.get('length_mm');angle=obj.get('closing_angle_plane_deg')
   if obj.get('colour') not in ('black','white') or not isinstance(xy,list) or len(xy)!=2:continue
   values=xy+[w,h,angle]
   if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):continue
   if abs(w-20)>2 or abs(h-20)>2:continue
   # Erode by the cube half-diagonal: reject candidates extending outside ROI.
   if cv2.pointPolygonTest(poly,tuple(xy),True)<math.sqrt(2)*10:continue
   # Do not turn already-sorted objects inside a fixed bin into new pickups.
   if any(math.hypot(xy[0]-b['center_mm'][0],xy[1]-b['center_mm'][1]) <= b['inside_diameter_mm']/2+math.sqrt(2)*10 for b in self.config['bins'].values()):continue
   result.append({'color':obj['colour'],'xyz_m':[xy[0]/1000,xy[1]/1000,.02],
                  'jaw_angle_rad':math.radians(angle),'frame_index':frame,
                  'measurement':'top face; grasp offset must be applied separately'})
  return result

if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=Path(__file__).with_name('camera_config.json'));a=p.parse_args()
 errors=issues(json.loads(a.config.read_text()))
 print(json.dumps({'ready_for_observations':not errors,'missing':errors,'motor_commands_sent':False},indent=2))
 raise SystemExit(1 if errors else 0)
