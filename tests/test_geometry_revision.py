"""Check measured axis distances against expanded URDF, independent of analytic IK."""
from pathlib import Path
import sys,math,xml.etree.ElementTree as E
import numpy as np
import pytest,xacro
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'model'))
from geometry import forward,REVISION

def robot():return E.fromstring(xacro.process_file(str(ROOT/'ros2_ws/src/arm_simulation/urdf/provisional_arm.urdf.xacro')).toxml())

def test_final_axis_distances_and_hinges():
    r=robot()
    expected={'sim_joint_2':.019,'sim_joint_3':.120,'sim_joint_4':.110,'sim_joint_5':.060,'gripper_motor_fixed':.100,'tool_fixed':.170}
    for name,z in expected.items():
        xyz=list(map(float,r.find(f"joint[@name='{name}']/origin").get('xyz').split()))
        assert xyz==pytest.approx([0,0,z])
    for name in ['sim_finger_joint','sim_finger_mimic']:assert r.find(f"joint[@name='{name}']").get('type')=='revolute'
    assert float(r.find("joint[@name='tool_fixed']/origin").get('xyz').split()[2])-.100==pytest.approx(.070)

def test_analytic_tool_matches_urdf_chain():
    r=robot();chain=[f'sim_joint_{i}' for i in range(1,6)]+['tool_fixed']
    for q in [[0,0,0,0,0],[.5,.2,1.,.8,0],[-.4,.35,-.7,.35,1.2]]:
        T=np.eye(4)
        for i,name in enumerate(chain):
            j=r.find(f"joint[@name='{name}']");A=np.eye(4)
            A[:3,3]=list(map(float,j.find('origin').get('xyz').split()));T=T@A
            if j.get('type')!='fixed':
                axis=np.array(list(map(float,j.find('axis').get('xyz').split())))
                A=np.eye(4);A[:3,:3]=Rotation.from_rotvec(axis*q[i]).as_matrix();T=T@A
        assert T[:3,3]==pytest.approx(forward(q),abs=1e-12)
