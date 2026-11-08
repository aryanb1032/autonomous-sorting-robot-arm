import json,sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from policy import OrderPolicy
from vision_contract import Observations,issues

def test_policy_update_and_reload(tmp_path):
 path=tmp_path/'q.json';p=OrderPolicy(path,True,4)
 items=[{'name':'b','color':'black','xyz':[.2,0,.01]},{'name':'w','color':'white','xyz':[.3,0,.01]}]
 item,decision=p.choose(items,[0,0]);p.update(decision,10,[],[0,0])
 loaded=OrderPolicy(path)
 assert loaded.updates==1 and loaded.q[decision[0]][decision[1]]==pytest.approx(2)
 assert item in items

def test_wrong_policy_revision(tmp_path):
 path=tmp_path/'q.json';path.write_text('{"revision":"old"}')
 with pytest.raises(ValueError):OrderPolicy(path)

def config(tmp_path):
 c=json.loads((ROOT/'camera_config.json').read_text());f=tmp_path/'cal.json'
 f.write_text(json.dumps({'validated':True,'coordinate_frame':'robot_base','plane_height_mm':20,'camera_matrix':[[1,0,0],[0,1,0],[0,0,1]],'pixel_to_plane_mm':[[1,0,0],[0,1,0],[0,0,1]],'dist_coeffs':[0,0,0,0,0],'image_size':[640,480]}))
 c.update(calibration_file=str(f),calibration_validated=True,stream_latency_validated=True,gripper_aperture_calibration_validated=True,measured_independent_error_mm=1,pickup_polygon_mm=[[0,0],[400,0],[400,400],[0,400]])
 for b in c['bins'].values():b.update(center_mm=[100,100],rim_height_mm=40,inside_diameter_mm=100)
 return c

def report():
 return {'timestamp_unix':100,'frame_index':1,'status':'frame_processed','calibrated':True,'objects':[{'colour':'black','center_plane_mm':[200,200],'width_mm':20,'length_mm':20,'closing_angle_plane_deg':30}]}

def test_unmeasured_config_is_locked():
 assert issues(json.loads((ROOT/'camera_config.json').read_text()))

def test_conversion_and_no_repeated_frames(tmp_path):
 reader=Observations(config(tmp_path));result=reader.decode(report(),100.1)
 assert result[0]['xyz_m']==[.2,.2,.02]
 with pytest.raises(ValueError):reader.decode(report(),100.2)

def test_stale_and_uncalibrated_rejected(tmp_path):
 for change,now in [({},102),({'calibrated':False},100.1),({'timestamp_unix':float('nan')},100.1)]:
  r=report();r.update(change)
  with pytest.raises(ValueError):Observations(config(tmp_path)).decode(r,now)

def test_both_cube_sides_and_roi_required(tmp_path):
 for key,value in [('length_mm',40),('center_plane_mm',[2,2]),('width_mm',float('nan'))]:
  r=report();r['objects'][0][key]=value
  assert Observations(config(tmp_path)).decode(r,100.1)==[]


def test_invalid_calibration_matrix_rejected(tmp_path):
 c=config(tmp_path);p=Path(c['calibration_file']);d=json.loads(p.read_text())
 d['pixel_to_plane_mm']=[[0,0,0]]*3;p.write_text(json.dumps(d))
 assert issues(c)


def test_independent_validation_detects_metric_error():
 from validate_calibration import validate
 c={'camera_matrix':[[1,0,0],[0,1,0],[0,0,1]],'dist_coeffs':[0]*5,'pixel_to_plane_mm':[[1,0,0],[0,1,0],[0,0,1]]}
 pairs={'pixels':[[0,0],[10,0],[10,10],[0,10]],'plane_xy_mm':[[0,0],[10,0],[10,10],[0,10]]}
 assert validate(c,pairs)['passed']
 c['pixel_to_plane_mm'][0][2]=5
 assert not validate(c,pairs)['passed']


def test_objects_in_fixed_bins_are_excluded(tmp_path):
 r=report();r['objects'][0]['center_plane_mm']=[100,100]
 assert Observations(config(tmp_path)).decode(r,100.1)==[]
