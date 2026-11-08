"""Gazebo-only reaching environment. No serial or real-hardware backend."""
import os
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"model"))
from geometry import REVISION,GEOMETRY
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import GetParameters
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionFK
from visualization_msgs.msg import Marker

NAMES = [f'sim_joint_{i}' for i in range(1, 6)]
TASK_REVISION = 'gazebo_reach_m4roll_m5pitch_v2'
HOME = np.array([0., .35, -.7, 0., .35])
LOW = np.array([-.65, .15, -1.0])
HIGH = np.array([.65, .65, -.35])


def candidate(q, action):
    """Restrict exploration to a small region; keep wrist pitch compensated."""
    action = np.asarray(action, dtype=float)
    if action.shape != (3,) or not np.isfinite(action).all():
        raise ValueError('Action must contain three finite values')
    result = np.array(q, dtype=float).copy()
    result[:3] = np.clip(result[:3] + .06 * np.clip(action, -1, 1), LOW, HIGH)
    result[3] = 0.
    result[4] = -result[1] - result[2]
    return result


class ReachEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, max_steps=30):
        if os.environ.get('ROS_DOMAIN_ID') != '42':
            raise RuntimeError('Source simulation_env.sh first; domain 42 required')
        self.max_steps = max_steps
        self.action_space = spaces.Box(-1., 1., (3,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (11,), dtype=np.float32)
        self.node = Node('rl_simulation_reaching')
        self.q = None
        self.state_received = 0.
        self.clock = None
        self.node.create_subscription(JointState, '/joint_states', self._state, qos_profile_sensor_data)
        self.node.create_subscription(Clock, '/clock', self._clock, qos_profile_sensor_data)
        self.marker = self.node.create_publisher(Marker, '/rl_target', 10)
        self.move = ActionClient(self.node, MoveGroup, '/move_action')
        self.fk = self.node.create_client(GetPositionFK, '/compute_fk')
        params = self.node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        if not params.wait_for_service(timeout_sec=10):
            raise RuntimeError('Simulation robot_state_publisher unavailable')
        req = GetParameters.Request(names=['robot_description'])
        xml = self._wait(params.call_async(req), 10).values[0].string_value
        if 'gz_ros2_control/GazeboSimSystem' not in xml or 'name="provisional_arm"' not in xml:
            raise RuntimeError('Requires this project Gazebo model, not mock or hardware')
        import xml.etree.ElementTree as E
        tree=E.fromstring(xml)
        expected={'sim_joint_2':GEOMETRY['motor1_to_motor2'],'sim_joint_3':GEOMETRY['motor2_to_motor3'],'sim_joint_4':GEOMETRY['motor3_to_motor4'],'sim_joint_5':GEOMETRY['motor4_to_motor5']}
        for name,z in expected.items():
            actual=float(tree.find(f"joint[@name='{name}']/origin").get('xyz').split()[2])
            if abs(actual-z)>1e-9:raise RuntimeError('Running robot geometry is obsolete; rebuild and restart Gazebo')
        for name,axis in [('sim_joint_4',[0.,0.,1.]),('sim_joint_5',[0.,1.,0.])]:
            actual=list(map(float,tree.find(f"joint[@name='{name}']/axis").get('xyz').split()))
            if actual!=axis:raise RuntimeError('Running wrist axes are obsolete; restart simulation')
        if not self.move.wait_for_server(timeout_sec=10) or not self.fk.wait_for_service(timeout_sec=10):
            raise RuntimeError('MoveIt unavailable')
        self._fresh_state()
        first = self.clock
        deadline = time.monotonic() + 5
        while (first is None or self.clock == first) and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.1)
            if first is None:
                first = self.clock
        if self.clock is None or self.clock == first:
            raise RuntimeError('Gazebo clock is not advancing; unpause simulation')

    def _state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values for n in NAMES):
            q = np.array([values[n] for n in NAMES])
            if np.isfinite(q).all():
                self.q = q
                self.state_received = time.monotonic()

    def _clock(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _wait(self, future, seconds):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=seconds)
        if not future.done():
            raise RuntimeError('ROS request timed out; training stopped')
        return future.result()

    def _fresh_state(self):
        start = time.monotonic()
        while self.state_received <= start:
            if time.monotonic() - start > 5:
                raise RuntimeError('No fresh joint feedback; training stopped')
            rclpy.spin_once(self.node, timeout_sec=.1)
        return self.q.copy()

    def _position(self, q):
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names = ['tool0']
        req.robot_state.joint_state.name = NAMES
        req.robot_state.joint_state.position = list(q)
        result = self._wait(self.fk.call_async(req), 5)
        if result.error_code.val != 1:
            raise RuntimeError('Forward kinematics failed')
        p = result.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z])

    def _move(self, q):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'arm'
        req.allowed_planning_time = 2.
        req.num_planning_attempts = 2
        req.max_velocity_scaling_factor = .3
        req.max_acceleration_scaling_factor = .3
        req.start_state.is_diff = True
        constraint = Constraints()
        for name, value in zip(NAMES, q):
            j = JointConstraint(joint_name=name, position=float(value),
                                tolerance_above=.005, tolerance_below=.005, weight=1.)
            constraint.joint_constraints.append(j)
        req.goal_constraints = [constraint]
        goal.planning_options.planning_scene_diff.is_diff = True
        handle = self._wait(self.move.send_goal_async(goal), 10)
        if not handle.accepted:
            raise RuntimeError('MoveIt action rejected')
        try:
            result = self._wait(handle.get_result_async(), 30).result
        except (RuntimeError, KeyboardInterrupt):
            self._wait(handle.cancel_goal_async(), 5)
            raise
        code = result.error_code.val
        # Planning failures are penalized; execution/communication failures halt training.
        if code not in (1, -1, -2, -10, -12, -31):
            raise RuntimeError(f'MoveIt execution failed ({code}); training stopped')
        self._fresh_state()
        return code == 1

    def _observation(self):
        self.position = self._position(self.q)
        self.distance = float(np.linalg.norm(self.target - self.position))
        return np.concatenate([self.q, self.target, self.target - self.position]).astype(np.float32)

    def _show_target(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns = 'rl_reach_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, self.target)
        marker.scale.x = marker.scale.y = marker.scale.z = .025
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, 1., .2, 1.
        self.marker.publish(marker)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if not self._move(HOME):
            raise RuntimeError('Could not reset to the ready pose')
        start_position = self._position(self.q)
        # Exclude targets already within the 15 mm success radius at reset.
        for _ in range(100):
            target_q = HOME.copy()
            target_q[:3] += self.np_random.uniform([-.4, .10, -.20], [.4, .25, -.08])
            target_q[3] = 0.
            target_q[4] = -target_q[1] - target_q[2]
            self.target = self._position(target_q)
            if np.linalg.norm(self.target - start_position) >= .03:
                break
        else:
            raise RuntimeError('Could not sample a nontrivial reaching target')
        self.steps = 0
        self.ended = False
        obs = self._observation()
        self._show_target()
        return obs, {'distance_m': self.distance}

    def step(self, action):
        if self.ended:
            raise RuntimeError('Reset before stepping a finished episode')
        old = self.distance
        accepted = self._move(candidate(self.q, action))
        self.steps += 1
        obs = self._observation()
        success = self.distance < .015
        reward = 30. * (old - self.distance) - self.distance - .01
        reward -= .002 * float(np.square(np.clip(action, -1, 1)).sum())
        if not accepted:
            reward -= 1.
        if success:
            reward += 5.
        truncated = self.steps >= self.max_steps and not success
        self.ended = success or truncated
        self._show_target()
        return obs, float(reward), success, truncated, {
            'is_success': success, 'distance_m': self.distance,
            'planning_accepted': accepted,
        }

    def close(self):
        self.node.destroy_node()
