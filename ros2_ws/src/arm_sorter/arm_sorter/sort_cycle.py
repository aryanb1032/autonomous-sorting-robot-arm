"""Observation-pose sorting coordinator. Dry run by default.

Real execution requires configured MoveIt, planning scene, calibrated gripper,
and a /sorting/verify_pick Trigger service backed by a real check.
"""
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

class Sorter(Node):
    def __init__(self):
        super().__init__('sort_coordinator')
        self.cfg=json.loads(Path(self.declare_parameter('config','').value).read_text())
        self.dry=self.declare_parameter('dry_run',True).value
        self.latest=None
        self.observe=self.create_publisher(Bool,'/sorting/observe',10)
        self.create_subscription(String,'/sorting/detections',self.receive,10)
        if not self.dry:
            if not self.cfg.get('execution_verified'):raise ValueError('Complete calibration and scene validation first')
            from moveit_msgs.action import MoveGroup
            from control_msgs.action import FollowJointTrajectory
            self.move=ActionClient(self,MoveGroup,'/move_action')
            self.grip=ActionClient(self,FollowJointTrajectory,'/gripper_controller/follow_joint_trajectory')
            self.verify=self.create_client(Trigger,'/sorting/verify_pick')
            if not self.verify.wait_for_service(timeout_sec=3):raise RuntimeError('Pickup verification service required')
    def receive(self,msg):
        try:self.latest=json.loads(msg.data)
        except (ValueError,TypeError):self.latest=None
    def wait_future(self,future,timeout):
        rclpy.spin_until_future_complete(self,future,timeout_sec=timeout)
        if not future.done():raise TimeoutError('ROS operation timed out')
        return future.result()
    def action(self,client,goal):
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Action server unavailable')
        handle=self.wait_future(client.send_goal_async(goal),5)
        if not handle.accepted:raise RuntimeError('Goal rejected')
        try:result=self.wait_future(handle.get_result_async(),60)
        except Exception:
            self.wait_future(handle.cancel_goal_async(),5);raise
        if result.status!=GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val!=1:
            raise RuntimeError('MoveIt execution failed')
    def move_to(self,label,position=None,yaw=None):
        if self.dry:
            self.get_logger().info(f'DRY RUN move {label}: {position}, closing yaw={yaw}');return
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        goal=MoveGroup.Goal();req=goal.request
        req.group_name=self.cfg['planning_group'];req.allowed_planning_time=5.;req.num_planning_attempts=3
        req.max_velocity_scaling_factor=0.1;req.max_acceleration_scaling_factor=0.1;req.start_state.is_diff=True
        constraint=Constraints()
        if position is None:
            for name,q in self.cfg['poses'][label].items():
                j=JointConstraint();j.joint_name=name;j.position=float(q);j.tolerance_above=0.01;j.tolerance_below=0.01;j.weight=1.;constraint.joint_constraints.append(j)
        else:
            p=PositionConstraint();p.header.frame_id='base_link';p.link_name=self.cfg['tool_link'];p.weight=1.
            shape=SolidPrimitive();shape.type=SolidPrimitive.SPHERE;shape.dimensions=[0.003]
            pose=Pose();pose.orientation.w=1.;pose.position.x=position[0];pose.position.y=position[1];pose.position.z=position[2]
            p.constraint_region.primitives=[shape];p.constraint_region.primitive_poses=[pose];constraint.position_constraints=[p]
            # qz(yaw + tool offset) * configured downward tool quaternion.
            x,y,z,w=self.cfg['downward_quaternion_xyzw'];a=(yaw+self.cfg['jaw_yaw_offset_rad'])/2;c=math.cos(a);s=math.sin(a)
            o=OrientationConstraint();o.header.frame_id='base_link';o.link_name=self.cfg['tool_link'];o.weight=1.
            o.orientation.x=c*x-s*y;o.orientation.y=c*y+s*x;o.orientation.z=c*z+s*w;o.orientation.w=c*w-s*z
            o.absolute_x_axis_tolerance=0.1;o.absolute_y_axis_tolerance=0.1;o.absolute_z_axis_tolerance=0.1
            constraint.orientation_constraints=[o]
        req.goal_constraints=[constraint];goal.planning_options.plan_only=False;goal.planning_options.planning_scene_diff.is_diff=True
        self.action(self.move,goal)
    def jaw(self,width):
        if self.cfg.get('gripper_command_units') in ('slide_metres','hinge_radians'):
            lo,hi=self.cfg['jaw_opening_limits_m']
            if not lo<=width<=hi:raise ValueError('Box exceeds measured jaw range')
            q=(width-lo)/2
            if self.cfg.get('gripper_command_units')=='hinge_radians':
                q=math.asin(q/self.cfg['gripper_finger_length_m'])
                if q>.4:raise ValueError('Width exceeds hinge model range')
        else:
            q=self.jaw_servo_angle(width)
        if self.dry:self.get_logger().info(f'DRY RUN jaw {width:.4f} m -> joint target {q:.4f}');return
        self.send_jaw(q)
    def jaw_servo_angle(self,width):
        pairs=np.asarray(self.cfg['jaw_calibration_width_rad'],dtype=float)
        if len(pairs)<2 or np.any(np.diff(pairs[:,0])<=0):raise ValueError('Jaw width calibration must increase')
        if not pairs[0,0]<=width<=pairs[-1,0]:raise ValueError('Box exceeds jaw range')
        return float(np.interp(width,pairs[:,0],pairs[:,1]))
    def send_jaw(self,q):
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=[self.cfg['gripper_joint']]
        point=JointTrajectoryPoint();point.positions=[q];point.time_from_start.sec=2;goal.trajectory.points=[point]
        if not self.grip.wait_for_server(timeout_sec=5):raise RuntimeError('Gripper unavailable')
        handle=self.wait_future(self.grip.send_goal_async(goal),5)
        if not handle.accepted:raise RuntimeError('Gripper rejected goal')
        try:r=self.wait_future(handle.get_result_async(),10)
        except Exception:
            self.wait_future(handle.cancel_goal_async(),5);raise
        if r.status!=GoalStatus.STATUS_SUCCEEDED or r.result.error_code!=0:raise RuntimeError('Gripper failed')
    def cycle(self,box):
        x,y,width=box['x'],box['y'],box['width_m'];yaw=box['closing_yaw_rad'];colour=box['colour']
        if colour not in ['black','white']:raise ValueError('Unknown colour')
        if not all(math.isfinite(v) for v in [x,y,width,yaw]):raise ValueError('Nonfinite detection')
        xmin,xmax,ymin,ymax=self.cfg['workspace_bounds_m']
        if not (xmin<=x<=xmax and ymin<=y<=ymax):raise ValueError('Target outside workspace')
        z=self.cfg['grasp_tool_z_m'];above=z+self.cfg['approach_clearance_m']
        self.jaw(width+self.cfg['jaw_clearance_m'])
        self.move_to('approach',[x,y,above],yaw)
        self.move_to('grasp',[x,y,z],yaw)
        self.jaw(width-self.cfg['grip_compression_m'])
        self.move_to('lift',[x,y,above],yaw)
        if not self.dry:
            if not self.wait_future(self.verify.call_async(Trigger.Request()),5).success:
                raise RuntimeError('Pickup verification failed; stop for inspection')
        # Scene must include conservative payload clearance, or attach geometry via
        # a project-specific scene adapter before enabling execution_verified.
        self.move_to(colour+'_bin')
        self.jaw(self.cfg['release_width_m'])
        self.move_to('observation')
    def run(self):
        sample=self.declare_parameter('demo_target',False).value
        if sample:
            if not self.dry:raise ValueError('Demo targets are dry-run only')
            self.cycle(dict(x=.20,y=0.,width_m=.04,closing_yaw_rad=0.,colour='black'));return
        self.move_to('observation')
        while rclpy.ok():
            self.observe.publish(Bool(data=False));self.latest=None
            # No capture is requested until the motion has completed and settled.
            end=time.monotonic()+2
            while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=0.1)
            start=self.get_clock().now().nanoseconds;self.latest=None
            while rclpy.ok():
                self.observe.publish(Bool(data=True));rclpy.spin_once(self,timeout_sec=0.2)
                if self.latest and self.latest.get('frame_id')=='base_link' and self.latest.get('stamp_ns',0)>start:
                    age=self.get_clock().now().nanoseconds-self.latest['stamp_ns']
                    if 0<=age<2e9 and self.latest.get('boxes'):break
            self.observe.publish(Bool(data=False))
            if not rclpy.ok():return
            self.cycle(self.latest['boxes'][0])

def main():
    rclpy.init();node=Sorter()
    try:node.run()
    except Exception as exc:node.get_logger().error(f'Sorting stopped: {exc}')
    finally:node.observe.publish(Bool(data=False));node.destroy_node();rclpy.shutdown()
