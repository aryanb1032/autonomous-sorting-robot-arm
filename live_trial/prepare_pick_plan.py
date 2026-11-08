#!/usr/bin/env python3
"""Plan only: save one common, retimed, collision-checked no-box trajectory."""
from pathlib import Path
import sys,json,math,hashlib
import numpy as np
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints,JointConstraint
from moveit_msgs.srv import GetStateValidity
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'model'))
from geometry import REVISION
NAMES=[f'sim_joint_{i}' for i in range(1,6)]
rclpy.init();node=rclpy.create_node('prepare_common_trial')
def wait(f,timeout=30):
 rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
 if not f.done():raise TimeoutError('ROS request timed out')
 return f.result()
try:
 client=ActionClient(node,MoveGroup,'/move_action');valid=node.create_client(GetStateValidity,'/check_state_validity')
 if not client.wait_for_server(timeout_sec=10) or not valid.wait_for_service(timeout_sec=10):raise RuntimeError('MoveIt not ready')
 points=[[0.]*5]
 targets=json.loads((ROOT/'live_trial/pick_targets.json').read_text())['steps']
 phases=[]
 for step in targets[1:]:
  target=step['arm_rad']
  goal=MoveGroup.Goal();req=goal.request;req.group_name='arm';req.allowed_planning_time=5.;req.num_planning_attempts=5
  req.max_velocity_scaling_factor=.1;req.max_acceleration_scaling_factor=.1
  req.start_state.joint_state.name=NAMES+['sim_finger_joint','sim_finger_mimic']
  req.start_state.joint_state.position=points[-1]+[0.,0.]
  constraints=Constraints()
  for name,value in zip(NAMES,target):constraints.joint_constraints.append(JointConstraint(joint_name=name,position=value,tolerance_above=.001,tolerance_below=.001,weight=1.))
  req.goal_constraints=[constraints];goal.planning_options.plan_only=True;goal.planning_options.planning_scene_diff.is_diff=True
  handle=wait(client.send_goal_async(goal))
  if not handle.accepted:raise RuntimeError('Plan rejected')
  result=wait(handle.get_result_async()).result
  if result.error_code.val!=1:raise RuntimeError(f'Plan failed {result.error_code.val}')
  trajectory=result.planned_trajectory.joint_trajectory
  order=[trajectory.joint_names.index(n) for n in NAMES]
  for pt in trajectory.points:
   q=np.array([pt.positions[i] for i in order]);start=np.array(points[-1])
   count=max(1,math.ceil(float(np.max(abs(q-start)))/math.radians(2)))
   if np.max(abs(q-start))<1e-8:continue
   for k in range(1,count+1):points.append((start+(q-start)*k/count).tolist())
  # Include the exact named pose, not just the planner's tolerance endpoint.
  if np.max(abs(np.array(points[-1])-target))>1e-9:points.append(target)
  phases.append({'phase':step['phase'],'point_index':len(points)-1,'event':step['virtual_grasp_event']})
 for q in points:
  if not all(math.isfinite(v) and abs(v)<=math.pi/2 for v in q):raise ValueError('Path exceeds nominal 90-degree model range')
  request=GetStateValidity.Request();request.group_name='arm'
  request.robot_state.joint_state.name=NAMES+['sim_finger_joint','sim_finger_mimic'];request.robot_state.joint_state.position=q+[0.,0.]
  if not wait(valid.call_async(request),5).valid:raise RuntimeError('Collision found in sampled path')
 durations=[max(.4,float(np.max(abs(np.array(b)-a)))/.12) for a,b in zip(points,points[1:])]
 data={'geometry_revision':REVISION,'joint_names':NAMES,'points':points,'segment_seconds':durations,'signs':[1,-1,1,1,1],'gripper_servo_degrees':80,'mode':'virtual_pick_plan_only','phases':phases,'physical_feedback':False}
 raw=json.dumps(data,sort_keys=True);data['sha256']=hashlib.sha256(raw.encode()).hexdigest()
 (ROOT/'live_trial/pick_plan.json').write_text(json.dumps(data,indent=2)+'\n')
 print(f'PLAN ONLY: {len(durations)} segments, {sum(durations):.1f}s motion plus segment handshakes. No motion sent.')
 print('Joint extrema radians:',np.min(points,axis=0),np.max(points,axis=0))
finally:node.destroy_node();rclpy.shutdown()
