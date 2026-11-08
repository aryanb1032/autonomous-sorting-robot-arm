> Current model revision: **2026-10-29-final-dimensions**. See [final geometry notes](../model/FINAL_GEOMETRY.md). Previous validation reports and RL checkpoints apply only to their original geometry.

# How this works, and how to reuse it

## The complete command journey

1. Camera firmware converts sensor frames to JPEG and serves them over Wi-Fi.
2. camera.py requests a fresh JPEG and publishes a ROS Image message.
3. perception.py finds masks, classifies brightness and transforms an outline
   into base-frame position, width and jaw-closing orientation.
4. sort_cycle.py decides what happens next: observe, approach, grip, lift, sort.
5. MoveIt solves inverse kinematics and plans a collision-aware trajectory using
   your URDF/SRDF, joint limits and planning scene.
6. bridge.py receives a FollowJointTrajectory action. It checks positions and
   speeds, then sends small timed joint-position segments over USB serial.
7. ESP32 firmware converts joint radians into calibrated pulse widths.
8. PCA9685 generates PWM signals. Servo electronics drive motors toward their
   requested positions. The separate servo power supply provides motor current.

The camera does not directly command a servo. Each layer converts information
into a more specific representation, and each can reject invalid input.

## Why these interfaces matter

| Concept | Meaning here | Reuse in another project |
|---|---|---|
| Topic | Repeated observations such as images or joint states | Replace a sensor while preserving its message type |
| Service | A short request and response, e.g. pickup verification | Replace a contact sensor with a vision check |
| Action | A long operation with result and cancellation | Swap mock, simulated and physical trajectory execution |
| Parameter | Settings such as port, model or config path | Reconfigure without editing algorithm code |
| URDF | Physical links, joint axes, origins and limits | Describe a different robot |
| SRDF | Planning groups and semantic robot configuration | Define a new tool/group without changing camera code |
| TF | Relationships between coordinate frames | Transform camera observations into robot coordinates |

## Units and frames

Use metres and radians in ROS. Convert mm to m by dividing by 1000; degrees to
radians by multiplying by pi/180. PWM pulse width is in microseconds. Servo
electrical angle, mechanical joint angle and gripper opening are different things.

Pixel width alone is not physical width. Perspective and camera distance affect
it. The observation-plane homography handles a calibrated plane; lens correction
comes first. A different camera pose, image crop/resolution or box height changes
that mapping. General moving-camera use requires hand-eye calibration and reliable
base-to-tool pose at the image timestamp.

## Commanded versus measured

This hardware has no confirmed position telemetry. joint_states from the bridge
are therefore estimates of commanded positions. DONE ESTIMATED means firmware
finished its schedule, not that a servo reached its target. A stalled or slipping
servo can disagree with that estimate. Encoders or servo telemetry are needed for
measured trajectory tolerances; contact/force feedback helps verify gripping.

## Why the gripper needs its own calibration

A hinged linkage converts rotation to linear travel, but linkage clearances,
gear geometry and finger arrangements affect actual opening. Measure the jaw gap
at several safe positions and interpolate. Opening to box width plus clearance
allows approach; closing slightly below measured width can generate contact but
cannot regulate force. Equal box weight does not remove these uncertainties.

## Read and experiment in this order

1. serial_check.ino and scripts/ping.py: explain every byte of PING/PONG.
2. geometry.py: run the rotated-box test and change the synthetic rotation.
3. bridge.py: run mock action tests; observe /joint_states with ros2 topic echo.
4. sort_cycle.py: run a dry sequence; change the demo colour to white in a copy.
5. camera.py and perception.py: inspect image output and reject uncertain masks.
6. hardware_config.h: explain why channel, polarity and limits are independent.
7. MoveIt setup: compare a joint target with a Cartesian tool pose target.
8. Gazebo: replace the serial executor with simulated controllers using the same
   action names. Compare what simulation can and cannot validate.

## Adapting this to your next project

| New project | Keep | Replace or recalibrate |
|---|---|---|
| Sort red/blue parts | Controller, robot model, task structure | Colour method/training data and bin mapping |
| Pick different box sizes | ROS messages and planner | Jaw limits, model clearance, measurement validation |
| Another printed arm | Camera transport, perception concepts | URDF, axes, limits, IK, controller mapping, all spatial calibration |
| Conveyor pickup | Some perception and planning infrastructure | Tracking, timestamps, prediction and synchronized motion |
| Mobile robot | ROS topic/action concepts | Robot model, navigation/control stack, task logic |
| Different motor controller | MoveIt action interface | Hardware transport, firmware and feedback implementation |

Start every adaptation with an interface contract: input units, output units,
frame, valid range, freshness, failure behaviour, and what success actually means.
Then test sensing, planning and execution separately before combining them.

## Build, run and upload are different operations

- Install downloads tools and dependencies onto the PC.
- Build translates source into executable code or prepares ROS entry points.
- Source selects a shell's ROS and Python environment.
- Launch/run starts a PC process; it does not necessarily upload anything.
- Compile turns an Arduino sketch into ESP32 machine code.
- Upload writes that binary into ESP32 flash, replacing its previous program.
- Calibrate measures the relationship between software units and the real world.

Compilation proves syntax and toolchain compatibility. Mock tests prove selected
software behaviours. Gazebo tests a model. Only carefully staged physical tests
can validate the real arm, power supply, repeatability and grasp reliability.

## References

- MoveIt 2 installation: https://moveit.ai/install-moveit2/binary/
- Trajectory actions: https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
- Gazebo controls: https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
- Ultralytics: https://docs.ultralytics.com/tasks/segment/
- Espressif camera example: https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer
- PCA9685 datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
