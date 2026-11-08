import copy,json,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from arm_sorter.jaw_mapping import JawMapping
from arm_sorter.bridge import Bridge
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'hardware'))
from prepare import validate,generate


def fixture_profile():
    p=json.loads((ROOT/'hardware/profile.template.json').read_text())
    p.update(calibration_confirmed=True,servo_power_verified=True,oe_wired_verified=True,servo_rail_voltage=6.0)
    for i,j in enumerate(p['joints']):
        j.update(channel=i,rad_low=-1.,rad_high=1.,pulse_at_low_us=1100.,pulse_at_high_us=1900.,initial_rad=0.,max_rad_per_sec=.2)
    p['joints'][-1].update(rad_low=0.,rad_high=1.,initial_rad=0.)
    p['gripper_width_table']=[[0.,.016],[.5,.04],[1.,.066]]
    return p

@pytest.mark.parametrize('table',[[[0,.016],[.5,.04],[1,.066]],[[0,.066],[.5,.04],[1,.016]]])
def test_jaw_round_trip(table):
    jaw=JawMapping(table)
    for q in np.linspace(0,1,21):assert jaw.to_servo(jaw.to_slide(q))==pytest.approx(q)
    with pytest.raises(ValueError):jaw.to_servo(-.01)
    with pytest.raises(ValueError):jaw.to_slide(2.)

@pytest.mark.parametrize('table',[[[0,.02],[1,.02]],[[0,.02],[0,.04]],[[0,.02],[1,float('nan')]]])
def test_bad_jaw_table(table):
    with pytest.raises(ValueError):JawMapping(table)

def test_unconfigured_profile_stays_locked(tmp_path):
    with pytest.raises(ValueError):generate(json.loads((ROOT/'hardware/profile.template.json').read_text()),tmp_path/'output')
    assert not (tmp_path/'output').exists()

@pytest.mark.parametrize('mutation',[lambda p:p.update(servo_rail_voltage=7.),lambda p:p['joints'][1].update(channel=0),lambda p:p['joints'][0].update(pulse_at_low_us=float('nan')),lambda p:p.update(oe_wired_verified=False)])
def test_bad_calibration(mutation):
    p=fixture_profile();mutation(p)
    with pytest.raises(ValueError):validate(p)

def test_generated_fingerprint_and_model(tmp_path):
    import xml.etree.ElementTree as E
    generate(fixture_profile(),tmp_path)
    cfg=json.loads((tmp_path/'bridge.json').read_text())
    assert cfg['config_id'] in (tmp_path/'arm_controller/hardware_config.h').read_text()
    robot=E.parse(tmp_path/'real_arm.urdf').getroot()
    assert robot.find('ros2_control') is None and robot.find('gazebo') is None
    assert float(robot.find("joint[@name='gripper_hinge_joint']/limit").get('upper'))==pytest.approx(np.arcsin(.025/.07))
    assert robot.find("joint[@name='joint_1']") is not None

def test_bridge_converts_slide_to_servo():
    cfg={'joint_names':['gripper_joint'],'limits':{'gripper_joint':[0,1]}}
    obj=SimpleNamespace(jaw=JawMapping([[0,.016],[1,.066]]),cfg=cfg)
    point=JointTrajectoryPoint(positions=[.0125]);point.time_from_start.sec=1
    trajectory=JointTrajectory(joint_names=['gripper_slide_joint'],points=[point])
    names,points=Bridge.decode(obj,trajectory)
    assert names==['gripper_joint'] and points[0][1][0]==pytest.approx(.5)
    trajectory.joint_names=['gripper_joint']
    with pytest.raises(ValueError):Bridge.decode(obj,trajectory)


def test_hinged_jaw_round_trip():
    jaw=JawMapping([[0,.016],[.5,.04],[1,.066]],.07)
    for q in np.linspace(0,1,31):assert jaw.from_joint(jaw.to_joint(q))==pytest.approx(q)
    with pytest.raises(ValueError):jaw.from_joint(.5)


def test_bridge_hinge_to_calibrated_servo():
    jaw=JawMapping([[0,.016],[1,.066]],.07)
    cfg={'joint_names':['gripper_joint'],'limits':{'gripper_joint':[0,1]},'gripper_model_joint':'gripper_hinge_joint'}
    obj=SimpleNamespace(jaw=jaw,cfg=cfg)
    point=JointTrajectoryPoint(positions=[jaw.to_joint(.5)]);point.time_from_start.sec=2
    trajectory=JointTrajectory(joint_names=['gripper_hinge_joint'],points=[point])
    names,points=Bridge.decode(obj,trajectory)
    assert names==['gripper_joint'] and points[0][1][0]==pytest.approx(.5)


def test_obsolete_geometry_profile_rejected():
    p=fixture_profile();p['geometry_revision']='old_model'
    with pytest.raises(ValueError,match='obsolete geometry'):validate(p)
