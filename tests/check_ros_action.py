"""Integration check against a running MOCK trajectory_bridge only."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
rclpy.init();node=Node('mock_action_test');client=ActionClient(node,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
def wait(future):
    rclpy.spin_until_future_complete(node,future,timeout_sec=8)
    assert future.done(),'Action timed out'
    return future.result()
def goal(name,position,seconds):
    g=FollowJointTrajectory.Goal();g.trajectory.joint_names=[name]
    p=JointTrajectoryPoint();p.positions=[position];p.time_from_start.sec=seconds;g.trajectory.points=[p];return g
try:
    assert client.wait_for_server(timeout_sec=5)
    assert not wait(client.send_goal_async(goal('unknown_joint',0.,1))).accepted
    h=wait(client.send_goal_async(goal('joint_1',.1,1)));assert h.accepted
    result=wait(h.get_result_async());assert result.status==GoalStatus.STATUS_SUCCEEDED
    h=wait(client.send_goal_async(goal('joint_1',.2,3)));assert h.accepted
    assert wait(h.cancel_goal_async()).goals_canceling
    assert wait(h.get_result_async()).status==GoalStatus.STATUS_CANCELED
    print('PASS: invalid joint rejection, mock trajectory execution, cancellation')
finally:node.destroy_node();rclpy.shutdown()
