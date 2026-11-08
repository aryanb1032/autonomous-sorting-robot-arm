#!/usr/bin/env python3
"""Virtual grasp in Gazebo; optional empty real arm mirrors every joint segment."""
import argparse,json,math,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'sorting'),str(ROOT/'live_trial')]
from sort_boxes import Sorter,LAYOUT,inside_bin
from prepare_pick_targets import inverse
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints,JointConstraint
from moveit_msgs.srv import GetStateValidity
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
NAMES=[f'sim_joint_{i}' for i in range(1,6)]
# User's TPU grasp policy: command full closure on every physical grasp.
# Servo endpoint is commanded position, not measured jaw force or closure.
GRIPPER_HOME_DEG=90.
GRIPPER_FULL_CLOSE_DEG=0.

class Trial(Sorter):
 def __init__(self,physical):
  self.serial=None;self.active=None;self.physical=physical;self.grip_angle=GRIPPER_HOME_DEG;self.finger=0.
  self.stopfile=ROOT/'control/stop.request';self.status=ROOT/'control/active_run.json'
  super().__init__()
  self.arm=ActionClient(self.node,FollowJointTrajectory,'/sim_arm_controller/follow_joint_trajectory')
  self.valid=self.node.create_client(GetStateValidity,'/check_state_validity')
  if not self.arm.wait_for_server(timeout_sec=5) or not self.valid.wait_for_service(timeout_sec=5):raise RuntimeError('Controllers unavailable')
 def _state(self,msg):
  super()._state(msg)
  values=dict(zip(msg.name,msg.position))
  if 'sim_finger_joint' in values:self.finger=float(values['sim_finger_joint'])
 def connect_physical(self):
  if self.physical:
   self._fresh_state()
   if max(abs(self.q))>.02:raise RuntimeError('Gazebo not at upright home')
   import serial
   self.serial=serial.Serial(port=None,baudrate=115200,timeout=.02,write_timeout=.5,exclusive=True)
   self.serial.dtr=False;self.serial.rts=False
   self.serial.port='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
   self.serial.open();time.sleep(3);self.serial.reset_input_buffer()
   self.send('INFO');self.expect('CONFIG VIRTUAL_PICK_HOME90_V2')
   self.stopfile.unlink(missing_ok=True);self.record('active')
   self.send('INIT CONFIRM_INITIAL_POSE');self.expect('READY')
 def _wait(self,future,seconds):
  deadline=time.monotonic()+seconds;beat=0
  while not future.done() and time.monotonic()<deadline:
   if self.serial and self.serial.is_open:
    if self.stopfile.exists():raise InterruptedError('O STOP')
    if time.monotonic()-beat>.2:self.send('KEEPALIVE');self.record('active');beat=time.monotonic()
   rclpy.spin_once(self.node,timeout_sec=.02)
  if not future.done():raise TimeoutError('ROS reply missing')
  return future.result()
 def record(self,state):
  if self.physical:
   tmp=self.status.with_suffix('.tmp');tmp.write_text(json.dumps({'state':state,'time':time.time()}));tmp.replace(self.status)
 def send(self,s):self.serial.write((s+'\n').encode())
 def read(self):
  s=self.serial.readline(256).decode(errors='replace').strip()
  if s:print('MCU '+s,flush=True)
  if s.startswith(('ERR','ets ','rst:','ARM_V1 WAIT_INIT','BOOT HOME90','STOP LATCHED')):raise RuntimeError('Controller error/reset: '+s)
  return s
 def expect(self,text):
  end=time.monotonic()+4
  while time.monotonic()<end:
   if self.read()==text:return
  raise TimeoutError('Missing MCU '+text)
 def segment(self,start,end,seconds,finger_start=None,finger_end=None):
  client=self.arm if finger_end is None else self.grip
  names=NAMES if finger_end is None else ['sim_finger_joint']
  x=start if finger_end is None else [finger_start];y=end if finger_end is None else [finger_end]
  goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=names
  ms=math.ceil(seconds*1000);seconds=ms/1000
  goal.trajectory.points=[JointTrajectoryPoint(positions=list(x),time_from_start=Duration()),JointTrajectoryPoint(positions=list(y),time_from_start=Duration(sec=ms//1000,nanosec=(ms%1000)*1000000))]
  self.active=self._wait(client.send_goal_async(goal),5)
  if not self.active.accepted:raise RuntimeError('Segment rejected')
  future=self.active.get_result_async();ack=False;done=self.serial is None
  if self.serial:self.send('MOVE '+str(ms)+' '+' '.join(f'{v:.8f}' for v in list(end)+[math.radians(self.grip_angle)]))
  deadline=time.monotonic()+seconds+5;heartbeat=0
  while time.monotonic()<deadline:
   if hasattr(self,"check_stop"):self.check_stop()
   if self.serial:
    if self.stopfile.exists():raise InterruptedError('O STOP')
    if time.monotonic()-heartbeat>.2:self.send('KEEPALIVE');self.record('active');heartbeat=time.monotonic()
    s=self.read()
    if s=='ACK':ack=True
    if s=='DONE ESTIMATED' and ack:done=True
   rclpy.spin_once(self.node,timeout_sec=.01)
   if future.done() and done:break
  else:raise TimeoutError(f'Segment missing completion: gazebo={future.done()}, MCU_ACK={ack}, MCU_DONE={done}')
  result=future.result()
  if result.status!=4 or result.result.error_code!=0:raise RuntimeError('Gazebo execution failed '+result.result.error_string)
  self._fresh_state()
  if max(abs(self.q-np.asarray(end)))>.03:raise RuntimeError('Gazebo arm tracking mismatch')
  self.active=None
 def _move(self,q):
  goal=MoveGroup.Goal();req=goal.request;req.group_name='arm';req.allowed_planning_time=5.;req.num_planning_attempts=5
  req.max_velocity_scaling_factor=.1;req.max_acceleration_scaling_factor=.1;req.start_state.is_diff=True
  con=Constraints()
  for name,v in zip(NAMES,q):con.joint_constraints.append(JointConstraint(joint_name=name,position=float(v),tolerance_above=.001,tolerance_below=.001,weight=1.))
  req.goal_constraints=[con];goal.planning_options.plan_only=True;goal.planning_options.planning_scene_diff.is_diff=True
  h=self._wait(self.move.send_goal_async(goal),10)
  if not h.accepted:raise RuntimeError('Plan rejected')
  result=self._wait(h.get_result_async(),30).result
  if result.error_code.val!=1:raise RuntimeError('Planning failed '+str(result.error_code.val))
  traj=result.planned_trajectory.joint_trajectory;order=[traj.joint_names.index(n) for n in NAMES]
  points=[self.q.copy()]
  for pt in traj.points:
   target=np.array([pt.positions[i] for i in order]);start=points[-1]
   delta=max(abs(target-start))
   if delta<1e-6:continue
   count=max(1,math.ceil(delta/math.radians(2)))
   points.extend(start+(target-start)*k/count for k in range(1,count+1))
  points.append(np.array(q,dtype=float))
  for target in points:
   if max(abs(target))>math.pi/2:raise RuntimeError('Plan exceeds physical travel')
   check=GetStateValidity.Request();check.group_name='arm';check.robot_state.joint_state.name=NAMES+['sim_finger_joint','sim_finger_mimic'];check.robot_state.joint_state.position=target.tolist()+[self.finger]*2
   if not self._wait(self.valid.call_async(check),5).valid:raise RuntimeError('Sampled path collision')
  for start,target in zip(points,points[1:]):self.segment(start,target,max(.1,float(max(abs(target-start)))/.12))
  return True
 def gripper(self,value):
  # Physical TPU grasp always commands the full-close endpoint, independent of
  # simulated finger clearance. Keep this command during lift/transfer until release.
  # This is NOT an aperture/force calibration.
  old=self.grip_angle;self.grip_angle=GRIPPER_HOME_DEG if value>.15 else GRIPPER_FULL_CLOSE_DEG
  seconds=max(2.,abs(math.radians(self.grip_angle-old))/.18)
  self.segment(self.q,self.q,seconds,self.finger,value);self.finger=value
 def prepare_scene(self):
  if self.serial is not None:raise RuntimeError('Scene setup must precede physical connection')
  from gz.msgs10.empty_pb2 import Empty
  for obj in LAYOUT['objects']:
   deadline=time.monotonic()+.4
   while time.monotonic()<deadline:
    self.pubs[obj,'detach'].publish(Empty());rclpy.spin_once(self.node,timeout_sec=.03)
   self.attachment(obj,True);self.attachment(obj,False)
  self._move([0.]*5)
  self.gripper(.30);self.sync_scene()
  for obj,item in LAYOUT['objects'].items():
   if np.linalg.norm(self.object_pose(obj)[0]-item['xyz'])>.012:raise RuntimeError('Reset Gazebo: box is not at its initial position')
 def run_single(self):
  name='black_box_01';item=LAYOUT['objects'][name];bin=LAYOUT['bins']['black'];drop=bin['slots'][0]
  angle=item['source_angle'];radius=item['source_radius'];da=math.atan2(drop[1],drop[0]);dr=math.hypot(*drop[:2])
  self.move_to(inverse(angle,.12,radius),'Approach virtual box')
  self.move_to(inverse(angle,.01,radius),'Descend to virtual box')
  if np.linalg.norm(self.object_pose(name)[0]-self._position(self.q))>.008:raise RuntimeError('Virtual box alignment failed')
  self.gripper(.07);self.support_contact(name,True);self.attachment(name,True);self.attach_scene(name)
  self.move_to(inverse(angle,.12,radius),'Lift virtual box');self.support_contact(name,False)
  self.move_to(inverse(da,.12,dr),'Carry virtual box to bin')
  self.move_to(inverse(da,.07,dr),'Lower over virtual bin')
  self.gripper(.30);self.attachment(name,False);self.detach_scene(name)
  self.wait_until(lambda:inside_bin(*self.object_pose(name),bin),8,'Virtual box did not settle inside bin')
  self.move_to(inverse(da,.12,dr),'Retreat');self.move_to([0.]*5,'Return home')
  self.record('home_held');print('DONE virtual box delivered; real arm commands have no encoder feedback',flush=True)
 def abort(self):
  if self.active is not None:
   try:
    future=self.active.cancel_goal_async();rclpy.spin_until_future_complete(self.node,future,timeout_sec=2)
   except Exception:pass
  if self.serial:
   try:self.send('STOP');self.expect('DISABLED');self.record('disabled')
   except Exception:self.record('failed_check_power');print('STOP unconfirmed: cut servo power',flush=True)

def main():
 p=argparse.ArgumentParser();p.add_argument('--physical',action='store_true');p.add_argument('--confirm-supported-home',action='store_true');a=p.parse_args()
 if a.physical and not a.confirm_supported_home:raise SystemExit('Supported physical home confirmation required')
 rclpy.init();env=None
 try:
  env=Trial(a.physical);env.prepare_scene();env.connect_physical();env.run_single()
 except BaseException:
  if env:env.abort()
  raise
 finally:
  if env:
   if env.serial:env.serial.close()
   env.close()
  rclpy.shutdown()
if __name__=='__main__':main()
