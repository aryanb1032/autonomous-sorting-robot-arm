#!/usr/bin/env bash
source "$(dirname "${BASH_SOURCE[0]}")/activate.sh"
export ROS_DOMAIN_ID=42
export GZ_PARTITION=robot_arm_provisional_sim
echo 'SIMULATION ONLY: ROS domain 42; dedicated sim_* joints and controllers.'
