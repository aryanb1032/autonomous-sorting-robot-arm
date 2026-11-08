from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    root=Path(get_package_share_directory('arm_simulation'))
    backend=LaunchConfiguration('backend').perform(context)
    if backend not in ('mock','gazebo'):raise ValueError('backend must be mock or gazebo')
    sim=backend=='gazebo'
    sorting=LaunchConfiguration('sorting').perform(context)=='true'
    if sorting and not sim: raise ValueError('Sorting needs Gazebo')
    gui=LaunchConfiguration('gui').perform(context)=='true'
    rviz=LaunchConfiguration('rviz').perform(context)=='true'
    controllers=str(root/'config/controllers.yaml')
    description={'robot_description':xacro.process_file(str(root/'urdf/provisional_arm.urdf.xacro'),mappings={'gazebo':str(sim).lower(),'controllers':controllers,'sorting':str(sorting).lower()}).toxml()}
    def read(name):return yaml.safe_load((root/'config'/name).read_text())
    move=[description,{'robot_description_semantic':(root/'config/provisional_arm.srdf').read_text()},
          {'robot_description_kinematics':read('kinematics.yaml')},
          {'robot_description_planning':read('joint_limits.yaml')},
          {'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':read('ompl.yaml')},
          read('moveit_controllers.yaml'),
          {'use_sim_time':sim,'publish_robot_description':True,'publish_robot_description_semantic':True,
           'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True}]
    nodes=[LogInfo(msg='PROVISIONAL SIMULATION ONLY: assumed axes, masses and limits; no hardware connected.'),
           Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[description,{'use_sim_time':sim}],output='screen')]
    if sim:
        nodes += [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
                   launch_arguments={'gz_args':('-r ' if gui else '-r -s ')+str(root/('worlds/sorting_workcell.sdf' if sorting else 'worlds/workcell.sdf'))+' --physics-engine gz-physics-bullet-featherstone-plugin'}.items()),
                  Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],output='screen')]
        create=Node(package='ros_gz_sim',executable='create',arguments=['-name','provisional_arm','-topic','robot_description','-allow_renaming','false'],output='screen')
        nodes.append(TimerAction(period=2.,actions=[create]))
    else:
        nodes.append(Node(package='controller_manager',executable='ros2_control_node',parameters=[controllers],output='screen'))
    js=Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','--controller-manager-timeout','90'],output='screen')
    arm=Node(package='controller_manager',executable='spawner',arguments=['sim_arm_controller','--controller-manager-timeout','90'],output='screen')
    grip=Node(package='controller_manager',executable='spawner',arguments=['sim_gripper_controller','--controller-manager-timeout','90'],output='screen')
    nodes.append(TimerAction(period=4. if sim else 1.,actions=[js]))
    def after_success(actions):
        def callback(event,context):
            if event.returncode==0:return actions
            return [LogInfo(msg='Controller startup failed; shutting down simulation.'),EmitEvent(event=Shutdown(reason='Controller failed'))]
        return callback
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=js,on_exit=after_success([arm]))))
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=arm,on_exit=after_success([grip]))))
    planning=[Node(package='moveit_ros_move_group',executable='move_group',parameters=move,output='screen')]
    if rviz:planning.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'config/simulation.rviz')],parameters=move,output='screen'))
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=grip,on_exit=after_success(planning))))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('backend',default_value='mock'),DeclareLaunchArgument('sorting',default_value='false'),DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=setup)])
