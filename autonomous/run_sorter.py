#!/usr/bin/env python3
"""Autonomous Gazebo sorting with simulator observations; never opens a physical port."""
import argparse,json,math,sys,time,fcntl,signal
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'live_trial'),str(ROOT/'sorting')]
from single_box import Trial
from sort_boxes import LAYOUT,inside_bin
from prepare_pick_targets import inverse
from policy import OrderPolicy
import rclpy

class AutonomousSorter(Trial):
 def __init__(self,policy,folder):
  self.folder=folder;self.policy=policy;self.phase='STARTING';self.completed=[];self.blocked=[];self.placed={'black':0,'white':0}
  self.eventfile=folder/'events.jsonl';self.stop_path=ROOT/'autonomous/STOP'
  super().__init__(False)
 def emit(self,phase,**data):
  self.phase=phase;row={'time':time.time(),'phase':phase,**data}
  print(json.dumps(row),flush=True)
  with self.eventfile.open('a') as f:f.write(json.dumps(row)+'\n')
  tmp=self.folder/'status.tmp';tmp.write_text(json.dumps(row));tmp.replace(self.folder/'status.json')
 def check_stop(self):
  if self.stop_path.exists():raise InterruptedError('Operator STOP requested')
 def _wait(self,future,seconds):
  end=time.monotonic()+seconds
  while not future.done() and time.monotonic()<end:
   self.check_stop();rclpy.spin_once(self.node,timeout_sec=.02)
  if not future.done():raise TimeoutError('ROS response missing')
  return future.result()
 def segment(self,*args,**kwargs):
  self.check_stop();return super().segment(*args,**kwargs)
 def wait_until(self,predicate,timeout,message):
  end=time.monotonic()+timeout
  while not predicate():
   self.check_stop()
   if time.monotonic()>end:raise RuntimeError(message)
   rclpy.spin_once(self.node,timeout_sec=.03)
 def observe(self):
  rclpy.spin_once(self.node,timeout_sec=.05)
  items=[]
  for name,obj in LAYOUT['objects'].items():
   if name in self.completed or name in self.blocked:continue
   xyz,_=self.object_pose(name)
   color=obj['color'];slot=self.placed[color]
   if slot>=len(LAYOUT['bins'][color]['slots']):raise RuntimeError(f'{color} bin full')
   try:
    angle=math.atan2(xyz[1],xyz[0]);radius=math.hypot(*xyz[:2])
    inverse(angle,float(xyz[2]),radius);inverse(angle,.12,radius)
   except ValueError:
    self.blocked.append(name);self.emit('UNREACHABLE',object=name);continue
   items.append({'name':name,'color':color,'xyz':xyz.tolist()})
  return items
 def move_phase(self,phase,q):
  self.emit(phase);self.move_to(q,phase)
 def pick_place(self,item):
  name=item['name'];color=item['color'];xyz=np.array(item['xyz'])
  angle=math.atan2(xyz[1],xyz[0]);radius=math.hypot(*xyz[:2]);bin=LAYOUT['bins'][color]
  drop=bin['slots'][self.placed[color]];da=math.atan2(drop[1],drop[0]);dr=math.hypot(*drop[:2])
  # Retry only a planner rejection before grasp. Never retry uncertain motion/communication.
  approach=inverse(angle,.12,radius)
  for attempt in range(2):
   try:self.move_phase('APPROACH',approach);break
   except RuntimeError as e:
    if attempt or not str(e).startswith('Planning failed '):raise
    self.emit('REPLAN_APPROACH',object=name,attempt=attempt+1)
  if np.linalg.norm(self.object_pose(name)[0]-xyz)>.008:raise RuntimeError('Object moved before pickup')
  self.move_phase('DESCEND',inverse(angle,float(xyz[2]),radius))
  if np.linalg.norm(self.object_pose(name)[0]-self._position(self.q))>.008:raise RuntimeError('Pickup alignment failed')
  # Attachment models grasp; leave clearance for a rotated 20 mm cube.
  # This is not a friction/force or real gripper-aperture model.
  self.emit('GRASP',object=name);self.gripper(.14)
  self.support_contact(name,True);self.attachment(name,True);self.attach_scene(name)
  self.move_phase('LIFT',approach)
  if self.object_pose(name)[0][2]<xyz[2]+.035:raise RuntimeError('Lift not verified')
  self.support_contact(name,False)
  self.move_phase('TRANSFER',inverse(da,.12,dr))
  self.move_phase('LOWER',inverse(da,.07,dr))
  self.emit('RELEASE',object=name,bin=color);self.gripper(.30);self.attachment(name,False);self.detach_scene(name)
  good_since=None;deadline=time.monotonic()+8
  while time.monotonic()<deadline:
   self.check_stop();rclpy.spin_once(self.node,timeout_sec=.04)
   good=inside_bin(*self.object_pose(name),bin)
   if good:
    if good_since is None:good_since=time.monotonic()
    if time.monotonic()-good_since>=.6:break
   else:good_since=None
  else:raise RuntimeError('Bin placement not verified')
  self.completed.append(name);self.placed[color]+=1
  self.emit('PLACEMENT_VERIFIED',object=name,bin=color)
  self.move_phase('RETREAT',inverse(da,.12,dr))
  return drop
 def run(self):
  self.emit('INITIALIZE',observation_source='Gazebo ground truth',physical_control=False,learning_scope='sorting order only')
  self.prepare_scene();xy=[0.,0.]
  while True:
   self.check_stop();items=self.observe()
   if not items:break
   selected,decision=self.policy.choose(items,xy);self.emit('SELECT',object=selected['name'],color=selected['color'],strategy=decision[1])
   start=time.monotonic()
   try:drop=self.pick_place(selected)
   except BaseException:
    self.policy.update(decision,-5,[],xy);raise
   remaining=self.observe();reward=10.-.01*(time.monotonic()-start)
   self.policy.update(decision,reward,remaining,drop[:2]);xy=drop[:2]
  self.move_phase('HOME',[0.]*5)
  result={'mode':'simulation_only','observation_source':'Gazebo ground truth','grasp':'simulated attachment','sorted':self.completed,'blocked':self.blocked,'counts':self.placed,'policy_updates':self.policy.updates}
  (self.folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
  self.emit('DONE' if not self.blocked else 'DONE_WITH_UNREACHABLE',**result)

def main():
 p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--learn',action='store_true');p.add_argument('--seed',type=int,default=42);p.add_argument('--policy',type=Path,default=ROOT/'autonomous/models/order.json');a=p.parse_args()
 a.run_dir.mkdir(parents=True,exist_ok=True)
 lock=(ROOT/'reinforcement_learning/.training.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 rclpy.init();env=None
 def interrupted(*_):raise KeyboardInterrupt('Shutdown requested')
 signal.signal(signal.SIGTERM,interrupted)
 try:
  env=AutonomousSorter(OrderPolicy(a.policy,a.learn,a.seed),a.run_dir);env.run()
 except BaseException as e:
  if env:env.emit('STOPPED',reason=str(e));env.abort()
  raise
 finally:
  if env:env.close()
  rclpy.shutdown()
if __name__=='__main__':main()
