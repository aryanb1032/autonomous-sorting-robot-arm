"""Shared geometry for offline kinematics and revision checks; values in metres."""
from pathlib import Path
import yaml, math
PATH=Path(__file__).resolve().parents[1]/'ros2_ws/src/arm_simulation/config/geometry.yaml'
GEOMETRY=yaml.safe_load(PATH.read_text())
REVISION=GEOMETRY['revision']
L1=GEOMETRY['motor2_to_motor3']; L2=GEOMETRY['motor3_to_motor4']
SHOULDER_HEIGHT=GEOMETRY['ground_to_motor1']+GEOMETRY['motor1_to_motor2']
TOOL_OFFSET=sum(GEOMETRY[k] for k in ['motor4_to_motor5','motor5_to_gripper_motor','gripper_motor_to_tip'])
FINGER_LENGTH=GEOMETRY['gripper_motor_to_tip']

def forward(q):
    """Tool position for base yaw, shoulder pitch, elbow pitch, wrist roll, wrist pitch."""
    import numpy as np
    R=np.eye(3);position=np.array([0.,0.,GEOMETRY['ground_to_motor1']])
    offsets=[0.,GEOMETRY['motor1_to_motor2'],L1,L2,GEOMETRY['motor4_to_motor5']]
    for angle,axis,offset in zip(q,['z','y','y','z','y'],offsets):
        position+=R@np.array([0.,0.,offset])
        c,s=math.cos(angle),math.sin(angle)
        rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]]) if axis=='z' else np.array([[c,0,s],[0,1,0],[-s,0,c]])
        R=R@rotation
    position+=R@np.array([0.,0.,GEOMETRY['motor5_to_gripper_motor']+FINGER_LENGTH])
    return position.tolist()
