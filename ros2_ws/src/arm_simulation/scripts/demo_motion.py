#!/usr/bin/env python3
"""Simulation-only trajectory exercise. Uses dedicated sim_* joint/controller names."""
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
if os.environ.get('ROS_DOMAIN_ID')!='42':raise SystemExit('Use simulation environment: ROS_DOMAIN_ID=42')
rclpy.init();node=Node('simulation_motion_test')
client=ActionClient(node,FollowJointTrajectory,'/sim_arm_controller/follow_joint_trajectory')
try:
    if not client.wait_for_server(timeout_sec=20):raise RuntimeError('Simulation controller unavailable')
    goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=[f'sim_joint_{i}' for i in range(1,6)]
    for seconds,values in [(3,[0,.25,-.5,.25,0]),(6,[.5,.25,-.5,.25,0]),(9,[-.5,.25,-.5,.25,0]),(12,[0,0,0,0,0])]:
        point=JointTrajectoryPoint();point.positions=values;point.time_from_start.sec=seconds;goal.trajectory.points.append(point)
    f=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,f,timeout_sec=10)
    if not f.done() or not f.result().accepted:raise RuntimeError('Trajectory rejected')
    result=f.result().get_result_async();rclpy.spin_until_future_complete(node,result,timeout_sec=30)
    if not result.done() or result.result().result.error_code!=0:raise RuntimeError('Simulation trajectory failed')
    print('PASS: simulated arm trajectory completed; no physical hardware used.')
finally:node.destroy_node();rclpy.shutdown()
