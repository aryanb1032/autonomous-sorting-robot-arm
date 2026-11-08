# Final user dimensions — 29 October 2026

These dimensions supersede the earlier 200/180 mm links and rack-and-pinion description.

| Axis-to-axis distance | Metres in the model | Servo at starting axis |
|---|---:|---|
| Motor 1 base → motor 2 shoulder | 0.019 | DEAO 20 kg-class |
| Motor 2 shoulder → motor 3 elbow | 0.120 | DEAO 20 kg-class |
| Motor 3 elbow → motor 4 wrist pitch | 0.110 | DEAO 20 kg-class |
| Motor 4 wrist pitch → motor 5 wrist rotation | 0.060 | MG996R |
| Motor 5 wrist rotation → gripper motor | 0.100 | MG996R |
| Gripper motor → end of finger tip | 0.070 | MG996R |

The source of truth is `../ros2_ws/src/arm_simulation/config/geometry.yaml`. Xacro loads it when creating the robot description. `geometry.py` reads the same values for analytic sorting IK and version checks. All translations are along each link's local Z in the straight configuration; real joint-axis directions and lateral offsets still need verification.

Updated 31 October: the user measured ground-to-motor-1 axis height as 70 mm. It is separate from the new motor1-to-motor2 distance of 19 mm. Thus the shoulder axis is 89 mm above the model ground, and the zero-pose tip is 549 mm above ground. The wrist-pitch-axis-to-tip length is 230 mm (60 + 100 + 70). These derived dimensions are not additional measurements.

Motor roles remain base yaw, shoulder pitch, elbow pitch, wrist pitch, wrist roll and gripper. This is five arm joints plus one gripper actuator. Servo torque labels are not calibrated URDF torque limits; all dynamic masses/inertias and simulation effort limits remain approximations.

## Pictured gripper

The new screenshot supersedes the earlier rack-and-pinion description. The simulation now has mirrored revolute jaws, represented by 70 mm rigid fingers. The gripper-motor frame is 100 mm beyond motor 5, with the closed-tip reference 70 mm further on. The 70 mm motor-to-tip measurement does not establish each four-bar link length or the true finger hinge axis: using it as a proxy finger length is an explicit approximation.

This is **not an exact four-bar linkage or CAD replica**. The screenshot lacks pivot coordinates, linkage-bar lengths, finger profiles, mechanical transmission and camera mount dimensions. GrabCAD returned HTTP 403 during this update; no meshes were downloaded. The visuals/collisions remain simplified bodies. A closed kinematic loop needs a suitable linkage model/constraint implementation once the missing mechanical data or CAD files are available.

The gripper's model command is now an angle in radians, not sliding travel in metres. Simulation uses `sim_finger_joint` and its mirror; generated hardware uses `gripper_hinge_joint` and `gripper_hinge_mimic`. The measured servo-angle/opening table maps this virtual angle to actuator angle. The proxy uses opening = minimum opening + 2 × 0.070 × sin(hinge angle); this is an aperture approximation, not a claim about the pictured mechanism's transmission. The tool reference stays at the closed-tip location; opened fingers retract axially in the proxy.

All real jaw widths must be measured again. Finger collision geometry, moving contact point and actual linkage transmission must be checked before `model_verified` becomes true. Servo PWM limits, channels and force cannot be inferred from link dimensions.

## What was updated

- Xacro joint offsets, rigid-body visual/collision lengths and dimension-dependent box inertias.
- Explicit gripper-motor frame and revised tool reference.
- Mirrored revolute fingers; angular controller limits/tolerances.
- Ground-sort analytic IK with height-dependent pitch; it uses the shared dimensions.
- Hardware configuration template, generated URDF/SRDF/controller names, bridge and physical sorter gripper conversions.
- Servo assignments: motors 1–3 DEAO 20 kg-class, 4–6 MG996R.
- RL run directories and checkpoint geometry guards. Old models are retained but cannot be resumed/evaluated as current models.
- Guides and regression tests. Old sorting results are retained as `last_result_pre_2026_10_29.json`, not current validation.

Box size stays 20 mm and black/white classification is unchanged. Camera firmware does not encode arm link lengths, so it does not need a geometry-driven re-upload. Camera-to-tool mounting, observation pose, pixel-to-base mapping, bin reachability and physical collision scenes require recalibration for this arm. Vision templates remain uncalibrated; physical execution remains gated.

The motor firmware takes joint angles and calibrated pulses, not link lengths. Generate it again only from an updated, measured hardware profile; do not reuse old calibration flags. New profiles carry `geometry_revision: 2026-10-31-base70`. No motor firmware was uploaded during the model update.

## Open and test

From the robot-arm project directory:

```bash
source simulation_env.sh
bash scripts/build.sh
bash scripts/start_simulation.sh gazebo
```

For a fresh ground sorting batch, stop the first simulation, then:

```bash
bash scripts/start_sorting_demo.sh --seed 42
```

The sorting demo remains scripted and uses simulated attachment grasps; it is not a vision-driven learned sorting policy. Consult `VALIDATION.md` here for results for this revision.

The current physical trial contains no boxes. Earlier sorting validation is historical for the old base height and has not been repeated for this revision. Base collision geometry is still a rectangular proxy, not an exact model of the photographed stand.

## Wrist order correction — 1 November 2026

User clarified M4/channel5 spins the gripper and M5/channel6 bends it. The actual
chain is base Z, shoulder Y, elbow Y, wrist Z, wrist Y, with previous measured
axis-to-axis distances retained. Revision: 2026-11-01-wrist-roll-pitch.
In the chosen home coordinate frame +X points toward the left of the marked
photo and +Z upward. The user-observed increasing-servo direction signs are
[+1,-1,+1,+1,+1]. These observations establish a nominal mapping, not encoder
calibration. Real servo transmission scale and shape offsets remain approximate.
The gripper stays at80 for the requested no-box trial.

URDF axes, named poses, shared forward kinematics, and observation records were
updated. Old planar sorting IK is explicitly blocked because it assumes M4 pitch
and M5 roll; it must be revised before any future sorting run. Old sorting/RL
results are historical, not validation of this revised chain. No new physical
motor commands were sent as part of the model correction.
