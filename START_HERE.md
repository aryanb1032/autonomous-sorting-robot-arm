> **2026-11-02 update:** Use [the autonomous simulation guide](autonomous/README.md)
> for the one-command Gazebo/RViz sorting runner and phone-vision preparation.
> The older tables below describe the original starter, not the latest validated
> simulation. Real camera-guided sorting still requires measured calibration and
> physical integration; do not enable an older execution flag to bypass that work.

> Current model revision: **2026-10-29-final-dimensions**. See [final geometry notes](model/FINAL_GEOMETRY.md). Previous validation reports and RL checkpoints apply only to their original geometry.

# Your reusable robot-arm project

## What is ready, and what is still specific to your robot

This is a working software starter, not a physically validated autonomous robot.
It contains executable camera/perception nodes, a mock or serial trajectory
action server, a dry-run/MoveIt sorting coordinator, calibration utilities,
ESP32 firmware, and build/upload scripts. Hardware-specific unknowns are locked
or left blank. Do not substitute demonstration dimensions or limits for calibration.

| Piece | Current readiness |
|---|---|
| ROS package | Built locally |
| Geometry and input validation | 13 automated tests passed |
| Sorting sequence | Dry run passed; no physical motion |
| Serial check firmware | Source ready; ESP32 core installation must finish before compilation/upload |
| PCA9685 firmware | Source prepared; output locked until calibrated |
| Camera firmware | Copy official example after core installation; actual board pinout and Wi-Fi still needed |
| MoveIt configuration | Controller mapping provided; actual URDF/SRDF and kinematics still required |
| Gazebo | Controller YAML provided; actual model, inertias, collision geometry and launch configuration still required |
| Vision measurements | Code ready; lens and observation-plane calibration still required |
| Box detection | Controlled-background prototype or custom YOLO segmentation weights |
| Real sorting | Requires calibrated targets, scene, controller and a pickup-verification service |

## 1. Open the project environment

In a new bash terminal:

```bash
source /path/to/robot-arm-project/activate.sh
bash check_pc.sh
```

The activation script selects ROS Jazzy, the Python vision environment and our
built ROS package. It does not flash firmware or command movement.
Keep the project and its adjacent work directory in place.

## 2. Run a complete dry sorting sequence now

```bash
ros2 run arm_sorter sort_cycle --ros-args \
  -p config:="$ROBOT_PROJECT/ros2_ws/src/arm_sorter/config/sort_demo.json" \
  -p dry_run:=true -p demo_target:=true
```

Read the printed sequence: open, approach, descend, close, lift, bin, release,
observation. The target, gripper mapping and heights are demonstration values.
No motors are contacted; this is a workflow test, not Gazebo simulation.

## 3. Test ROS trajectory communication without hardware

Terminal A, after activation:

```bash
ros2 run arm_sorter trajectory_bridge --ros-args \
  -p config:="$ROBOT_PROJECT/ros2_ws/src/arm_sorter/config/bridge_demo.json" \
  -p mock:=true
```

Terminal B, after activation:

```bash
python tests/check_ros_action.py
```

Run this integration test only against the mock controller. It tests an invalid
joint, a valid movement, and cancellation. Stop the mock bridge with Ctrl+C.

## 4. Verify USB communication after the ESP32 compiler installation

Connect ESP32 USB with servo power off. Confirm the device using:

```bash
python -m serial.tools.list_ports -v
```

Use its actual port, replacing /dev/ttyUSB0 if different:

```bash
bash scripts/upload.sh serial_check /dev/ttyUSB0
python scripts/ping.py /dev/ttyUSB0
```

Uploading replaces the ESP32's current program. The test sketch only responds
to PING and never initializes servo outputs. Close serial monitors before
running the Python test. Some Dev Modules require holding BOOT while connecting.
Do not choose a port based solely on its example name.

## 5. Complete hardware calibration before motor firmware

Reported wiring: 3.3 V -> VCC, common GND, GPIO21 -> SDA, GPIO22 -> SCL.
User wrote PCA9865; confirm PCA9685 on the actual board before using this code.
Servo power goes to the board's servo supply terminal, not logic VCC.

The reported 7 V exceeds Tower Pro's MG996R published 4.8–6.6 V range.
Use an appropriately regulated voltage within the actual servo ratings; confirm
the DEAO model's rating too. Determine current capacity from simultaneous load
and stall requirements, rather than assuming a small USB supply is sufficient.
Source: https://torqpro.com/product/mg996r/

Fill firmware/arm_controller/hardware_config.h with:

- Each channel and joint order, base to wrist, then gripper.
- Safe joint limits in radians, corresponding PWM pulse widths, and direction.
- Conservative maximum joint speeds.
- A known supported initial pose matching the physical arm.

The all-zero arrays and -1 channels are placeholders, not calibration defaults.
Only set HARDWARE_CALIBRATED=true after recording real values. The firmware
checks limits, moves linearly, and holds its last commanded position on STOP or
communication timeout. It cannot measure actual angle, load, or gripping force.
A power cutoff may allow a gravity-loaded arm to fall; support it appropriately.

Create a separate bridge_real.json using the same joint names, bounds, initial
pose and velocity limits. Set calibrated=true and the actual port there.
After validating low-risk individual movement, the real bridge can be started:

```bash
ros2 run arm_sorter trajectory_bridge --ros-args \
  -p config:=/ABSOLUTE/PATH/bridge_real.json \
  -p mock:=false -p confirm_initial_pose:=true
```

This command can energize the calibrated initial pose. Do not run it until the
arm is physically supported at that pose and servo power/wiring is verified.
Opening the serial port can reset the ESP32. Initialization therefore occurs
over that same connection. No auto-initialization occurs by default.

## 6. Create the actual robot model and MoveIt configuration

Reported lengths (19, 120, 110, 60, 100, 70 mm) are not sufficient to reconstruct all
joint origins, axes and tool offsets. Measure joint-centre distances, axis
directions, mechanical zeros and tool-tip/camera offsets. Six motors including
the gripper imply five arm axes unless another motor was omitted.

Create URDF/Xacro from those measurements and your CAD. Include visual meshes,
conservative collision geometry and correct limits. For Gazebo, also provide
physically valid masses/inertias. Check joint direction in RViz before execution.

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Load the actual URDF; create an arm planning group and gripper group; select
the actual tool link; review self-collision pairs; configure an appropriate IK
solver; save observation and bin configurations. Generate the MoveIt package.
With five axes, arbitrary 6D poses may be unreachable. Test the top-down grasp
orientation across the entire pickup region; do not silently drop constraints.

config/moveit_controllers.yaml is the mapping for the provided serial bridge.
It uses FollowJointTrajectory for both arm and gripper. Do not also launch
ros2_control controllers with the same action names in this mode.

Add the table and bins to MoveIt's planning scene. For a held box, attach its
collision object or validate a conservative envelope covering its dimensions.
The coordinator does not yet build that scene automatically. Approach/retreat
uses planned pose goals, not guaranteed vertical Cartesian segments. Validate
the entire path in RViz before enabling real sorting.

## 7. Gazebo is a separate execution mode

Use the real URDF with gz_ros2_control position command interfaces and state
interfaces. Load config/gazebo_controllers.yaml, spawn the model in a Harmonic
world, and activate the joint-state broadcaster and the two controllers.
All simulation nodes need use_sim_time=true and a consistent simulation clock.
The provided serial bridge must be off in this mode. MoveIt's controller action
names remain the same, so the planner can target simulated joints instead.

Gazebo is not configured end to end yet: it needs your actual robot model. A
fake generic arm would test a different mechanism and give misleading confidence.
Reference: https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html

## 8. Camera firmware and observations

```bash
python scripts/prepare_camera_example.py
```

This copies the official CameraWebServer example from the installed Espressif
core. Select the actual camera board and configure Wi-Fi locally. OV3660 is a
sensor name; it does not determine the board's GPIO pin mapping. Use the FQBN
appropriate to the camera board, not automatically the arm's ESP32 Dev Module.
The upload.sh helper intentionally supports only the arm's two sketches.

After the camera serves snapshots, run:

```bash
ros2 run arm_sorter camera --ros-args \
  -p capture_url:=http://CAMERA_IP/capture
```

This publishes /camera/image_raw. The timestamp is PC receive time, not sensor
exposure time. CameraInfo publication is not included yet; intrinsics are read
from the vision JSON. This is enough for the fixed-observation prototype.

## 9. Calibrate before trusting millimetres

Use a checkerboard to calibrate intrinsics at the actual image resolution.
Return the arm to a repeatable observation pose. Place at least six measured
reference points on a plane at the future box-top height. Record pixel points
and base-frame XY coordinates in metres, and run scripts/calibrate_plane.py.
Its docstring describes the JSON input. Validate errors on different points.

Merge the resulting intrinsics and homography into vision.template.json; fill
workspace bounds and expected widths, then mark calibrated=true only after
checking accuracy. This mapping is valid only at that observation pose and
height. It is not a general eye-in-hand calibration for moving-camera vision.

The controlled_background detector needs a uniform middle-brightness surface,
matte boxes and stable lighting. For YOLO mode, collect images of your actual
boxes, annotate segmentation masks, split training/validation/test images, train
YOLOv8n-seg, and set weights to your best.pt. Generic COCO weights have not been
trained for your black_box/white_box classes.

```bash
yolo segment train model=models/yolov8n-seg.pt data=/ABSOLUTE/PATH/dataset.yaml epochs=50 imgsz=640 device=cpu
```

This is a starting training command, not a guaranteed accuracy target. CPU
training can be slow. Validate detection and metric width errors on real boxes.

## 10. Assemble autonomous operation

Use sort_real.template.json for real values: observation/bin joint positions,
tool height, workspace, downward quaternion, jaw-axis offset and measured jaw
opening versus servo angle. Record gripper calibration from narrow to wide.
Small width compression is an open-loop approximation, not force control.

The coordinator requires /sorting/verify_pick (std_srvs/Trigger) for real motion.
Implement it using a real contact/force sensor or validated visual check.
There is deliberately no always-success verifier supplied for physical use.
On failure it stops the cycle; automatic retries/bin-full monitoring are future
work. Release success is not sensed by the current code.

Start the real bridge, robot_state_publisher and MoveIt package, then camera
and calibrated perception. Finally run sort_cycle with the real config and
dry_run=false only after stage-by-stage verification. Set execution_verified
only after the model, scene, calibration and controller are validated together.

## Learn the reusable architecture

Read docs/LEARN_AND_REUSE.md next. For any new project, preserve the interfaces
and change hardware configuration, robot geometry, perception, and task logic
one layer at a time. Keep a mock mode so each layer can be tested independently.

## Mixed-box sorting and learning to build your own arm

Run `scripts/start_sorting_demo.sh` for a fresh randomized batch of three black
and three white 20 mm cubes. See `sorting/README.md` for options and limitations.
The step-by-step construction guide is `docs/BUILD_YOUR_OWN_ARM.md`.


## Connecting the assembled arm

Read [the detailed real-robot setup guide](docs/REAL_ROBOT_SETUP.md). Start with `bash scripts/hardware_preflight.sh`; keep servo V+ disconnected during diagnostic uploads. Channels remain unspecified, and 7 V must not directly power MG996R servos. Camera PCB identification and physical calibration are still required.
