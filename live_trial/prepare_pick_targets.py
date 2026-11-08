#!/usr/bin/env python3
"""Offline targets for virtual-box / empty physical gripper rehearsal. No IO to robot."""
import json,math,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'model'))
from geometry import GEOMETRY,REVISION,forward

def inverse(angle,z,radius):
    # With wrist roll fixed at zero, M3->M4->M5 forms a single planar link.
    a=GEOMETRY['motor2_to_motor3']
    b=GEOMETRY['motor3_to_motor4']+GEOMETRY['motor4_to_motor5']
    t=GEOMETRY['motor5_to_gripper_motor']+GEOMETRY['gripper_motor_to_tip']
    h=GEOMETRY['ground_to_motor1']+GEOMETRY['motor1_to_motor2']
    candidates=[]
    for pitch in np.linspace(1.7,2.7,1001):
        x=radius-t*math.sin(pitch);y=z-h-t*math.cos(pitch)
        c=(x*x+y*y-a*a-b*b)/(2*a*b)
        if abs(c)>1:continue
        elbow=math.acos(c)
        shoulder=math.atan2(x,y)-math.atan2(b*math.sin(elbow),a+b*math.cos(elbow))
        wrist=pitch-shoulder-elbow
        q=[angle,shoulder,elbow,0.,wrist]
        # Intersection of user-reported servo travel and existing URDF limits.
        limits=[math.pi/2,1.4,math.pi/2,1.5,math.pi/2]
        if any(abs(v)>lim for v,lim in zip(q,limits)):continue
        target=[radius*math.cos(angle),radius*math.sin(angle),z]
        if np.linalg.norm(np.array(forward(q))-target)>1e-8:raise AssertionError('IK/FK mismatch')
        candidates.append((max(abs(shoulder),abs(elbow),abs(wrist)),q))
    if not candidates:raise ValueError('Target outside nominal servo travel')
    return min(candidates,key=lambda x:x[0])[1]

def main():
    layout=json.loads((ROOT/'sorting/layout.json').read_text())
    name='black_box_01';obj=layout['objects'][name];drop=layout['bins']['black']['slots'][0]
    angle=obj['source_angle'];radius=obj['source_radius']
    da=math.atan2(drop[1],drop[0]);dr=math.hypot(*drop[:2])
    steps=[]
    for label,q,event in [
        ('home',[0.]*5,'open'),
        ('approach',inverse(angle,.12,radius),None),
        ('pickup',inverse(angle,obj['xyz'][2],radius),'close_and_attach_virtual_box'),
        ('lift',inverse(angle,.12,radius),None),
        ('carry',inverse(da,.12,dr),None),
        ('drop',inverse(da,.07,dr),'open_and_detach_virtual_box'),
        ('retreat',inverse(da,.12,dr),None),
        ('home',[0.]*5,None)]:
        servos=[90+s*math.degrees(v) for s,v in zip([1,-1,1,1,1],q)]
        steps.append({'phase':label,'arm_rad':q,'nominal_servo_deg':servos,'virtual_grasp_event':event})
    data={'geometry_revision':REVISION,'mode':'virtual_box_empty_physical_gripper','box':name,'steps':steps,
          'status':'IK targets only; NOT collision-planned or executable',
          'installed_firmware_compatible':all(abs(v)<=math.pi/4 for st in steps for v in st['arm_rad']),
          'gripper_mapping':'pending simulated aperture to empty physical gripper command mapping',
          'physical_feedback':False}
    (ROOT/'live_trial/pick_targets.json').write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(data,indent=2))
if __name__=='__main__':main()
