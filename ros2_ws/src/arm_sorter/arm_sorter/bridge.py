"""Position-only FollowJointTrajectory adapter; mock mode is the default.

Piecewise linear interpolation, velocity limits checked. Reported states are
commanded estimates. Use ros2_control with real feedback for production control.
"""
import json
import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from .validation import validate_points
from .jaw_mapping import JawMapping

class Bridge(Node):
    def __init__(self):
        super().__init__('serial_trajectory_bridge')
        self.cfg=json.loads(Path(self.declare_parameter('config','').value).read_text())
        self.mock=self.declare_parameter('mock',True).value
        self.names=self.cfg['joint_names']; self.q=list(self.cfg['initial_rad'])
        self.jaw=JawMapping(self.cfg['gripper_width_table'],self.cfg.get('gripper_finger_length_m')) if self.cfg.get('gripper_width_table') else None
        self.lock=threading.Lock(); self.busy=False; self.port=None
        if len(self.q)!=len(self.names): raise ValueError('Initial position count mismatch')
        validate_points(self.names,[(0,self.q)],self.cfg)
        if not self.mock:
            if not self.cfg.get('calibrated'): raise ValueError('Hardware calibration required')
            import serial
            self.port=serial.Serial(self.cfg['port'],115200,timeout=0.2,write_timeout=0.5,exclusive=True)
            time.sleep(2);self.port.reset_input_buffer()
            if not self.cfg.get('config_id'):raise ValueError('Generate a hardware profile with a config fingerprint')
            self.port.write(b'INFO\n')
            if self.port.readline().decode().strip()!='CONFIG '+self.cfg['config_id']:
                raise RuntimeError('Uploaded firmware calibration does not match PC configuration')
            if self.declare_parameter('confirm_initial_pose',False).value:
                self.port.write(b'INIT CONFIRM_INITIAL_POSE\n')
                if self.port.readline().decode().strip()!='READY':
                    raise RuntimeError('Firmware initialization rejected')
            self.port.write(b'STATE\n')
            line=self.port.readline().decode().strip().split()
            if len(line)!=len(self.names)+2 or line[:2]!=['STATE','1']:
                raise RuntimeError('Firmware must be initialized at its calibrated pose first; opening serial may reset ESP32')
            self.q=[float(x) for x in line[2:]]
            validate_points(self.names,[(0,self.q)],self.cfg)
        self.pub=self.create_publisher(JointState,'/joint_states',10)
        self.create_timer(0.05,self.publish)
        group=ReentrantCallbackGroup()
        self.servers=[ActionServer(self,FollowJointTrajectory,f'/{name}/follow_joint_trajectory',self.execute,
                                  goal_callback=self.goal,cancel_callback=lambda _:CancelResponse.ACCEPT,
                                  callback_group=group)
                      for name in ['arm_controller','gripper_controller']]
        self.get_logger().warning('MOCK execution' if self.mock else 'OPEN LOOP: joint_states are estimates, not measured feedback')
    def publish(self):
        m=JointState();m.header.stamp=self.get_clock().now().to_msg()
        if self.jaw:
            slide=self.jaw.to_joint(self.q[self.names.index('gripper_joint')])
            joint=self.cfg.get('gripper_model_joint','gripper_slide_joint')
            m.name=self.names[:-1]+[joint,self.cfg.get('gripper_mimic_joint','gripper_slide_mimic')]
            m.position=self.q[:-1]+[slide,slide]
        else:m.name=self.names;m.position=self.q.copy()
        self.pub.publish(m)
    def decode(self,trajectory):
        names=list(trajectory.joint_names)
        points=[(p.time_from_start.sec+p.time_from_start.nanosec/1e9,list(p.positions)) for p in trajectory.points]
        if self.jaw:
            joint=self.cfg.get('gripper_model_joint','gripper_slide_joint')
            if 'gripper_joint' in names:raise ValueError('Use configured gripper model joint, not raw actuator radians')
            if joint in names:
                i=names.index(joint);names[i]='gripper_joint'
                for _,values in points:
                    if len(values)!=len(names):raise ValueError('Position count mismatch')
                    values[i]=self.jaw.from_joint(values[i])
        validate_points(names,points,self.cfg)
        return names,points

    def goal(self,request):
        try:
            if request.multi_dof_trajectory.points: raise ValueError('Multi-DOF trajectories unsupported')
            if request.path_tolerance or request.goal_tolerance: raise ValueError('Cannot guarantee measured tolerances without feedback')
            if request.trajectory.header.stamp.sec or request.trajectory.header.stamp.nanosec: raise ValueError('Only immediate trajectories supported')
            self.decode(request.trajectory)
            with self.lock:
                if self.busy:return GoalResponse.REJECT
                self.busy=True
            return GoalResponse.ACCEPT
        except ValueError as exc:
            self.get_logger().error(str(exc));return GoalResponse.REJECT
    def execute(self,handle):
        result=FollowJointTrajectory.Result()
        try:
            trajectory=handle.request.trajectory
            names,points=self.decode(trajectory)
            indices=[self.names.index(n) for n in names]
            previous_time=0.;previous=self.q.copy()
            # Validate every segment before sending any movement.
            segments=[]
            for seconds,positions in points:
                dt=seconds-previous_time;target=previous.copy()
                for i,q in zip(indices,positions):target[i]=q
                if dt==0:
                    if max(abs(a-b) for a,b in zip(target,previous))>0.01:raise ValueError('First point disagrees with current estimate')
                else:
                    if dt<0.02:raise ValueError('Segments must be at least 20 ms')
                    for name,a,b in zip(self.names,previous,target):
                        if abs(b-a)/dt>self.cfg['max_velocity'][name]:raise ValueError('Velocity limit exceeded')
                    segments.append((dt,target))
                previous=target;previous_time=seconds
            for dt,target in segments:
                origin=self.q.copy();steps=max(1,int(np.ceil(dt/0.1)))
                for step in range(1,steps+1):
                    if handle.is_cancel_requested:
                        if self.port:self.port.write(b'STOP\n')
                        handle.canceled();result.error_string='Canceled; holding last commanded pose';return result
                    q=[a+(b-a)*step/steps for a,b in zip(origin,target)]
                    seconds=dt/steps
                    if self.mock:time.sleep(seconds)
                    else:
                        ms=max(20,round(seconds*1000))
                        self.port.write(('MOVE '+str(ms)+' '+' '.join(f'{v:.6f}' for v in q)+'\n').encode())
                        deadline=time.monotonic()+seconds+2
                        done=False
                        while time.monotonic()<deadline:
                            reply=self.port.readline().decode().strip()
                            if reply.startswith('ERR'):raise RuntimeError(reply)
                            if reply=='DONE ESTIMATED':done=True;break
                        if not done:raise TimeoutError('ESP32 completion timeout')
                    self.q=q
                    feedback=FollowJointTrajectory.Feedback();feedback.joint_names=trajectory.joint_names
                    feedback.desired.positions=[self.jaw.to_joint(self.q[i]) if self.jaw and name==self.cfg.get('gripper_model_joint','gripper_slide_joint') else self.q[i] for name,i in zip(trajectory.joint_names,indices)]
                    # actual deliberately omitted: no encoder feedback available.
                    handle.publish_feedback(feedback)
            handle.succeed();result.error_code=result.SUCCESSFUL
            result.error_string='Command sequence completed; physical success unverified'
        except Exception as exc:
            if self.port:self.port.write(b'STOP\n')
            handle.abort();result.error_code=result.PATH_TOLERANCE_VIOLATED;result.error_string=str(exc)
        finally:
            with self.lock:self.busy=False
        return result

def main():
    rclpy.init();node=Bridge();executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    finally:
        if node.port:node.port.write(b'STOP\n');node.port.close()
        executor.shutdown();node.destroy_node();rclpy.shutdown()
