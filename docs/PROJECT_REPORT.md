# Project report: autonomous colour-sorting robotic arm

## Summary

This project built a low-cost robotic arm and software stack to detect 20 mm black and white cubes, pick them from a mixed area, and sort them into colour-specific bins. The reported BOM is ₹5,500. The system combines a six-DOF servo arm with an adaptive TPU gripper whose fingers are driven by a rack-and-pinion mechanism.

The work covered mechanical integration, ROS 2 and MoveIt configuration, simulation, embedded control, computer vision, camera calibration, and reinforcement-learning experiments. Simulation was completed and tested on the development laptop. The owner later reported a successful autonomous physical sorting run after making minor changes on another laptop; that adapted version is not included in this repository snapshot.

## Mechanical system

- 3D-printed six-DOF arm inspired by the GrabCAD 6-DOF Robot Arm V1.1 reference.
- Six servo channels: base, shoulder, elbow, wrist joints, and gripper actuation.
- Rack-and-pinion gripper moves the paired fingers; TPU finger material provides compliant contact.
- Recorded link dimensions (revision `2026-11-01-wrist-roll-pitch`): ground to motor 1 70 mm; motor 1 to 2 19 mm; motor 2 to 3 120 mm; motor 3 to 4 110 mm; motor 4 to 5 60 mm; motor 5 to gripper motor 100 mm; gripper motor to fingertip 70 mm.
- Servo hardware reported: three 20 kg servos and three MG996R servos. PCA9685 channels: 0, 1, 2, 5, 6, 7.
- Project-reported total BOM: ₹5,500.

These are the dimensions and component details recorded during this project; the URDF is a working model and should be checked against the assembled hardware before reuse.

## Software architecture

1. **Perception:** OpenCV processes a fixed phone-camera stream and classifies dark and light cube candidates. Image calibration and a planar mapping workflow were prepared for image-to-workspace coordinates.
2. **Task logic:** the autonomous state machine chooses a cube, plans approach and pickup, closes the simulated gripper, lifts, transfers, releases, checks placement, and returns to home.
3. **Motion and simulation:** ROS 2 Jazzy, MoveIt 2, `ros2_control`, Gazebo and RViz provide planning, controller execution, visualization, and a virtual sorting workcell.
4. **Embedded control:** ESP32 communicates with the PC over USB serial and commands PCA9685 servo PWM outputs. Firmware includes startup home behavior and a persistent STOP path.
5. **Learning experiments:** tabular Q-learning records outcomes and selects sorting order in simulation. A separate reaching-policy experiment was also exercised; it did not establish learned physical grasping or end-to-end vision control.

## Simulation evidence

The saved six-cube episode `autonomous/runs/20261102-110359/episode-001/result.json` records a simulation-only run that sorted three black and three white cubes, reported zero blocked objects, and completed nine policy updates. The run uses Gazebo ground-truth cube identity, colour, and position plus a simulated detachable grasp. It therefore validates the simulation flow, not camera-based perception or real contact dynamics.

The two-cube and STOP integration results are recorded in `autonomous/VALIDATION.md`. The documented policy updates concern sorting order. They do not show that the arm learned servo control, grasp force, or visual perception.

## Vision and calibration work

A DroidCam stream from a fixed Redmi Note 13 was used during camera experiments. A printed 25 mm checkerboard was used for lens calibration; multiple image batches and a planar workspace mapping were recorded. The final candidate mapping remained unvalidated for robot control. The independent cube-position comparison recorded about 6.9 mm error, above the project's 3 mm acceptance threshold. The camera stream and workspace also had visibility limitations. The current configuration appropriately leaves physical execution disabled.

## Physical controller evidence in this snapshot

A read-only serial check detected the ESP32 CP2102 serial bridge and identified firmware `VIRTUAL_PICK_HOME90_V2`. The board reported its estimated commanded home state. The check sent `INFO` and `STATE`; it did not send movement commands. Servos have no position encoders in this setup, so a reported command is not measured joint position.

The readiness record `autonomous/final_run_readiness.json` documents the snapshot's gaps: no complete camera-observation-to-physical-motion sorter, bin centres not registered in robot-base coordinates, and calibration independent-position error of about 6.9 mm. Accordingly, this snapshot did not run a camera-guided physical sorting task.

## Later owner-reported result

After this development snapshot, the project owner reported that another instructor/user made minor changes on a different laptop and the physical robot sorted autonomously as intended. This is an owner-reported outcome and useful project history. The changed files, exact operating conditions, and independent video/log evidence were not available when this repository snapshot was assembled. Do not attribute that later physical result to the unchanged files in this snapshot without adding the actual revised source and evidence.

## Reproduction

Use a compatible Ubuntu 24.04 / ROS 2 Jazzy installation. Build and launch using the scripts in the root README. For an autonomous simulated run, use `bash autonomous/start.sh --learn`. Keep the system in simulation; the scripts and firmware do not imply physical readiness. Read `autonomous/README.md`, `hardware-details.md`, and the safety notes before connecting a real arm.

## Main limitations at the snapshot date

- The robot description and kinematics are approximations of a custom printed arm.
- The successful simulation uses simulator ground truth and a virtual grasp attachment.
- The camera mapping had not passed independent physical accuracy validation.
- Camera-to-physical sorting orchestration and robot-base bin coordinates were absent.
- No servo encoders or independent servo-rail monitor provide physical feedback.
- The limited RL evidence concerns action ordering in simulation and a separate reaching experiment, not end-to-end learned sorting.
