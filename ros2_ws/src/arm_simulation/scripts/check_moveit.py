#!/usr/bin/env python3
"""Plan and execute a joint goal through MoveIt in the isolated simulation domain."""
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints,JointConstraint
if os.environ.get('ROS_DOMAIN_ID')!='42':raise SystemExit('Use simulation environment: ROS_DOMAIN_ID=42')
rclpy.init();node=Node('simulation_moveit_test');client=ActionClient(node,MoveGroup,'/move_action')
try:
    if not client.wait_for_server(timeout_sec=20):raise RuntimeError('MoveIt unavailable')
    goal=MoveGroup.Goal();req=goal.request;req.group_name='arm';req.allowed_planning_time=5.;req.num_planning_attempts=3
    req.max_velocity_scaling_factor=.2;req.max_acceleration_scaling_factor=.2;req.start_state.is_diff=True
    c=Constraints()
    for i,q in enumerate([0.,.25,-.5,.25,0.],1):
        j=JointConstraint();j.joint_name=f'sim_joint_{i}';j.position=q;j.tolerance_above=.01;j.tolerance_below=.01;j.weight=1.;c.joint_constraints.append(j)
    req.goal_constraints=[c];goal.planning_options.planning_scene_diff.is_diff=True
    future=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,future,timeout_sec=10)
    if not future.done() or not future.result().accepted:raise RuntimeError('MoveIt rejected goal')
    handle=future.result();future=handle.get_result_async();rclpy.spin_until_future_complete(node,future,timeout_sec=45)
    if not future.done():handle.cancel_goal_async();raise RuntimeError('MoveIt timed out')
    result=future.result().result
    if result.error_code.val!=1:raise RuntimeError(f'MoveIt error {result.error_code.val}')
    print('PASS: MoveIt planned and executed simulation goal.')
finally:node.destroy_node();rclpy.shutdown()
