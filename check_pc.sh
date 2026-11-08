#!/usr/bin/env bash
source "$(dirname "${BASH_SOURCE[0]}")/activate.sh"
missing=0
for pkg in moveit_ros_move_group moveit_setup_assistant controller_manager joint_trajectory_controller gz_ros2_control camera_calibration rviz2; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "OK: $pkg"
  else
    echo "MISSING: $pkg"
    missing=1
  fi
done
python -c 'import rclpy, cv_bridge, cv2, serial, ultralytics, torch; print("Python imports OK; torch:", torch.__version__, "CUDA:", torch.cuda.is_available())' || missing=1
arduino-cli --config-file "$ARDUINO_CONFIG_FILE" core list
if ! arduino-cli --config-file "$ARDUINO_CONFIG_FILE" core list | grep -q '^esp32:esp32'; then
  echo 'MISSING: ESP32 board support'; missing=1
fi
python -c 'from serial.tools.list_ports import comports; print("Serial devices:", [(p.device,p.description) for p in comports()])'
exit "$missing"
