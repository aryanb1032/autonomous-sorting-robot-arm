#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
missing=()
for package in moveit_ros_move_group moveit_setup_assistant moveit_kinematics moveit_planners_ompl moveit_ros_visualization moveit_simple_controller_manager ros_gz_sim ros_gz_bridge gz_ros2_control controller_manager joint_state_broadcaster joint_trajectory_controller robot_state_publisher xacro rviz2; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || missing+=("ros-jazzy-${package//_/-}")
done
if ((${#missing[@]})); then
  sudo apt-get update
  sudo apt-get install -y "${missing[@]}"
fi
python3 -c 'import psutil' || sudo apt-get install -y python3-psutil
echo 'Simulation dependencies available. Run scripts/build.sh, then scripts/start_simulation.sh gazebo.'
