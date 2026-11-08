# Validation record

Performed locally on 26 October 2026:

- colcon build --symlink-install --packages-select arm_sorter: PASS.
- 13 pure-Python tests: PASS. Includes rotated and perspective-transformed box
  width, singular calibration rejection, uncertain colour, malformed trajectories,
  nonfinite positions, joint limits and non-increasing times.
- Sorting demonstration sequence, dry_run=true: PASS, no hardware contacted.
- ROS FollowJointTrajectory mock integration: PASS. Unknown joint rejected,
  valid trajectory succeeded, cancellation produced canceled status.
- Python syntax compilation: PASS.
- Vision dependency check and pretrained YOLOv8 CPU smoke inference: PASS.

Not yet verified:

- ESP32 compilation: toolchain installation still running at this checkpoint.
- Physical USB, PWM pulse widths, motor direction, joint tracking and gripper force.
- Camera board firmware/pinout and actual image stream.
- Metric calibration and black/white accuracy on the actual boxes.
- Real MoveIt execution and Gazebo: actual URDF/SRDF/model not supplied yet.
- Real autonomous sorting and pickup verification.

No firmware was flashed and no motors were moved by these checks.
