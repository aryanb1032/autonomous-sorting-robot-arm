#!/usr/bin/env bash
source "$(dirname "${BASH_SOURCE[0]}")/activate.sh"
export ROS_DOMAIN_ID=44
unset GZ_PARTITION
export ROS_LOCALHOST_ONLY=1
echo 'Hardware commissioning domain 44. Firmware/calibration gates remain in effect.'
