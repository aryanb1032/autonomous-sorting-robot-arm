"""Hardware-model commissioning: mock by default, explicit gates for physical output."""
from pathlib import Path
import json,os,yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
ROOT=Path(__file__).resolve().parents[1]

def setup(context):
    if os.environ.get('ROS_DOMAIN_ID')!='44':raise ValueError('Use hardware_env.sh: domain 44 required')
    folder=Path(LaunchConfiguration('config_dir').perform(context));mock=LaunchConfiguration('mock').perform(context)=='true'
    confirm=LaunchConfiguration('confirm_initial_pose').perform(context)=='true'
    p=json.loads((folder/'profile.json').read_text())
    current=yaml.safe_load((ROOT/'ros2_ws/src/arm_simulation/config/geometry.yaml').read_text())
    if p.get('geometry_revision')!=current['revision']:raise ValueError('Obsolete generated hardware model: update profile and regenerate')
    cfg=json.loads((folder/'bridge.json').read_text())
    if not mock:
        if not confirm:raise ValueError('Support the arm at calibrated initial pose, then explicitly confirm it')
        for k in ['calibration_confirmed','servo_power_verified','oe_wired_verified','model_verified','workspace_scene_verified']:
            if p.get(k) is not True:raise ValueError(k+' must be verified before physical execution')
        if not cfg['port']:raise ValueError('Set the actual USB serial port in the profile and regenerate')
    def read(path):return yaml.safe_load(path.read_text())
    config=ROOT/'ros2_ws/src/arm_simulation/config'
    description={'robot_description':(folder/'real_arm.urdf').read_text()}
    params=[description,{'robot_description_semantic':(folder/'real_arm.srdf').read_text()},
        {'robot_description_kinematics':read(config/'kinematics.yaml')},
        {'robot_description_planning':read(folder/'joint_limits.yaml')},
        {'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':read(config/'ompl.yaml')},
        read(folder/'moveit_controllers.yaml'),{'use_sim_time':False,'publish_robot_description':True,'publish_robot_description_semantic':True,
        'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True}]
    nodes=[Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[description],output='screen'),
            Node(package='arm_sorter',executable='trajectory_bridge',parameters=[{'config':str(folder/'bridge.json'),'mock':mock,'confirm_initial_pose':confirm}],output='screen'),
            Node(package='moveit_ros_move_group',executable='move_group',parameters=params,output='screen')]
    if LaunchConfiguration('rviz').perform(context)=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(config/'simulation.rviz')],parameters=params,output='screen'))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_dir',default_value=str(ROOT/'hardware/generated')),
        DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('mock',default_value='true'),DeclareLaunchArgument('confirm_initial_pose',default_value='false'),OpaqueFunction(function=setup)])
