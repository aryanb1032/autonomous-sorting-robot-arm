#!/usr/bin/env python3
"""Replay identical saved targets in Gazebo, optionally also on the physical arm."""
from pathlib import Path
import argparse,json,hashlib,math,time,sys
import numpy as np
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'model'))
from geometry import REVISION
p=argparse.ArgumentParser();p.add_argument('--physical',action='store_true');p.add_argument('--confirm-supported-home',action='store_true');a=p.parse_args()
if a.physical and not a.confirm_supported_home:raise SystemExit('Supported physical home confirmation required')
data=json.loads((ROOT/'live_trial/plan.json').read_text());digest=data.pop('sha256')
if hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()!=digest:raise SystemExit('Plan fingerprint mismatch')
if data['geometry_revision']!=REVISION:raise SystemExit('Plan model is obsolete')
points=data['points'];durations=data['segment_seconds'];names=data['joint_names']
if len(points)!=len(durations)+1 or names!=[f'sim_joint_{i}' for i in range(1,6)]:raise SystemExit('Invalid plan shape')
for q in points:
 if len(q)!=5 or any(not math.isfinite(v) or abs(v)>math.pi/4 for v in q):raise SystemExit('Invalid joint target')
for x,y,t in zip(points,points[1:],durations):
 if not .02<=t<=10 or max(abs(v-u) for u,v in zip(x,y))/t>.2:raise SystemExit('Invalid segment speed')
rclpy.init();node=rclpy.create_node('shared_no_box_trial');latest=[None,0.]
def observe(msg):
 d=dict(zip(msg.name,msg.position))
 if all(n in d for n in names):latest[:]=[[d[n] for n in names],time.monotonic()]
sub=node.create_subscription(JointState,'/joint_states',observe,10)
client=ActionClient(node,FollowJointTrajectory,'/sim_arm_controller/follow_joint_trajectory')
serial_port=None;handle=None;success=False
status=ROOT/'control/active_run.json';stop=ROOT/'control/stop.request'
def record(value):
 if not a.physical:return
 tmp=status.with_suffix('.tmp');tmp.write_text(json.dumps({'state':value,'time':time.time()}));tmp.replace(status)
def send(text):serial_port.write((text+'\n').encode())
def wait(f,seconds=5):
 deadline=time.monotonic()+seconds
 while not f.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
 if not f.done():raise TimeoutError('ROS reply missing')
 return f.result()
def expect(prefix,seconds=3):
 deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  line=serial_port.readline(256).decode(errors='replace').strip()
  if line.startswith('ERR'):raise RuntimeError(line)
  if line.startswith(prefix):return line
 raise TimeoutError(prefix)
try:
 if not client.wait_for_server(timeout_sec=10):raise RuntimeError('Gazebo controller unavailable')
 deadline=time.monotonic()+5
 while latest[0] is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
 if latest[0] is None or max(abs(u-v) for u,v in zip(latest[0],points[0]))>.02:raise RuntimeError('Gazebo is not at saved start pose')
 if a.physical:
  import serial
  serial_port=serial.Serial('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',115200,timeout=.03,write_timeout=.5,exclusive=True)
  serial_port.dtr=False;serial_port.rts=False;time.sleep(2);serial_port.reset_input_buffer()
  send('INFO')
  if expect('CONFIG ')!='CONFIG LIVE_TRIAL_HOME90_V2':raise RuntimeError('Wrong physical firmware')
  stop.unlink(missing_ok=True);record('active')
  send('INIT CONFIRM_INITIAL_POSE');expect('READY')
 print('START common saved trajectory '+('GAZEBO + PHYSICAL' if a.physical else 'GAZEBO ONLY'),flush=True)
 for index,(start,target,seconds) in enumerate(zip(points,points[1:],durations)):
  goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=names
  # Position-only points request linear interpolation, matching the ESP32 MOVE.
  goal.trajectory.points=[JointTrajectoryPoint(positions=start,time_from_start=Duration()),JointTrajectoryPoint(positions=target,time_from_start=Duration(sec=int(seconds),nanosec=int((seconds%1)*1e9)))]
  handle=wait(client.send_goal_async(goal))
  if not handle.accepted:raise RuntimeError('Gazebo rejected segment')
  result=handle.get_result_async();done=not a.physical;ack=False;last=0
  if a.physical:send('MOVE '+str(math.ceil(seconds*1000))+' '+' '.join(f'{v:.8f}' for v in target+[math.radians(90)]))
  deadline=time.monotonic()+seconds+5
  while time.monotonic()<deadline:
   if a.physical:
    if stop.exists():raise InterruptedError('O-key stop requested')
    if time.monotonic()-last>.2:send('KEEPALIVE');record('active');last=time.monotonic()
    line=serial_port.readline(256).decode(errors='replace').strip()
    if line.startswith('ERR'):raise RuntimeError(line)
    if line=='ACK':ack=True
    if line=='DONE ESTIMATED' and ack:done=True
   rclpy.spin_once(node,timeout_sec=.01)
   if result.done() and done:break
  else:raise TimeoutError('Segment completion missing')
  response=result.result()
  if response.status!=4 or response.result.error_code!=0:raise RuntimeError(f'Gazebo segment failed: {response.result.error_string}')
  if time.monotonic()-latest[1]>1 or max(abs(x-y) for x,y in zip(latest[0],target))>.03:raise RuntimeError('Gazebo target tracking mismatch')
  handle=None
  if index%10==0:print(f'Segment {index+1}/{len(durations)} complete',flush=True)
 success=True;record('home_held');print('DONE HOME: real angles remain commanded estimates.',flush=True)
finally:
 if not success:
  if handle is not None:
   try:wait(handle.cancel_goal_async(),2)
   except Exception:pass
  if serial_port is not None:
   try:
    send('STOP');expect('DISABLED',2);record('disabled')
   except Exception:record('failed_check_power');print('STOP not confirmed; cut servo power.',flush=True)
 if serial_port is not None:serial_port.close()
 node.destroy_node();rclpy.shutdown()
