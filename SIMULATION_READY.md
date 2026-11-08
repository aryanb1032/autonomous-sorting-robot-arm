> Current model revision: **2026-10-29-final-dimensions**. See [final geometry notes](model/FINAL_GEOMETRY.md). Previous validation reports and RL checkpoints apply only to their original geometry.

# RViz, MoveIt 2 and Gazebo: ready for provisional simulation

## Start

```bash
bash /path/to/robot-arm-project/scripts/start_simulation.sh gazebo
```

This opens Gazebo Harmonic and RViz2, spawns the provisional robot, activates its
controllers, and starts MoveIt 2. Bullet Featherstone supplies physics with mimic
joint support. It does not access USB or the ESP32. Do not
start a second copy while the first is running. Stop the launch with Ctrl+C.

To use RViz/MoveIt with ideal mock joints and no physics:

```bash
bash /path/to/robot-arm-project/scripts/start_simulation.sh mock
```

In a second terminal:

```bash
source /path/to/robot-arm-project/simulation_env.sh
ros2 run arm_simulation check_moveit.py
```

This plans and executes a joint goal through MoveIt. For a preprogrammed
controller exercise (no MoveIt collision planning):

```bash
ros2 run arm_simulation demo_motion.py
```

The workcell now contains two 20 mm cubes. Do not run the direct controller
exercise near objects; it does not perform collision planning.
All simulation uses ROS_DOMAIN_ID=42, GZ_PARTITION=robot_arm_provisional_sim and
sim_* joint/controller names. Keep hardware on its separately configured domain.

## Using RViz

The MotionPlanning display should load the robot. Select the arm group and a
named goal such as ready, left_bin or right_bin, then Plan. Inspect the preview
before Execute. The planning controls may be below the Displays tree; expand
the MotionPlanning display if necessary. Gazebo displays the corresponding
physics motion; RViz displays the robot state and planned trajectory.

The five-axis model uses position-only KDL IK for exploration. This does not
guarantee an arbitrary wrist orientation. Named JOINT goals are the verified
path. Real top-down picking requires confirming orientation reachability.

## What the model represents

Source: ros2_ws/src/arm_simulation/urdf/provisional_arm.urdf.xacro.
An expanded provisional_arm.urdf is also provided. Edit the Xacro, then rebuild;
the expanded file is a snapshot and is not the launch input.

The Xacro properties `shoulder_to_elbow = 0.120` and `elbow_to_wrist = 0.110`
are in metres. Each drives the joint offset, link visual/collision length,
centre of mass and box inertia. All six final distances now come from config/geometry.yaml.
The three October 26 reference screenshots show the intended appearance but
cannot establish the remaining exact dimensions; this model remains a box proxy.

| Element | Basis |
|---|---|
| Base height 35 mm | User report |
| Shoulder-to-elbow joint-centre distance 120 mm | User final correction October 29 |
| Elbow-to-wrist joint-centre distance 110 mm | User final correction October 29 |
| Wrist1–wrist2 60 mm; wrist2–gripper motor 100 mm; gripper motor–tip 70 mm | User report |
| Five arm axes plus gripper | Inferred from six motors including gripper; confirm |
| Axis order Z/Y/Y/Y/Z | Provisional assumption |
| M1–M2 19 mm, full wrist/tool chain | User final correction October 29; linkage shape remains approximate |
| Link masses, inertias, limits, speeds, colours | Simulation assumptions |
| Two mirrored hinged fingers | Simplified hinged-linkage proxy |
| Geometry | Boxes, not downloaded STL meshes |

The simulation gripper joint is now in RADIANS (mirrored hinges). It is not the same as the physical
servo-angle joint in the earlier arm_sorter package. Their names deliberately
differ. Do not connect that sorting configuration to this model without an
explicit jaw-width/servo-angle mapping and compatible group names.

## GrabCAD access and replacement

Reference: https://grabcad.com/library/6-dof-robot-arm-v1-1
Title: 6 DOF robot arm V1; author Amrith; uploaded November 3, 2022.
The page lists STL/rendering files. Download files opens Sign In or Create Account.
No CAD archive or meshes were downloaded, and no mesh dimensions were measured.

To make a faithful model, download the archive through your GrabCAD account and
provide the STL files, preferably the original assembly/STEP if available. We
must identify which parts belong to each rigid link, units, mesh origins, joint
centres and axes, and differences from your modified hinged-linkage gripper.
STL alone does not encode kinematic joints or servo limits.

## Verified locally

- All required runtime packages for these launch files resolve in ROS Jazzy.
- arm_simulation builds with colcon.
- Xacro expands into URDF and robot_state_publisher loads it.
- Mock backend: MoveIt planned AND executed the test joint goal.
- Gazebo backend: model spawned; arm, gripper and joint-state controllers active.
- Gazebo backend: MoveIt planned AND executed the test joint goal.
- Gazebo gripper action: target 0.015 m completed with feedback 0.0149999 m.
- Arm and gripper goal tolerances enabled; success requires tracking the target.
- RViz2 and Gazebo GUI processes launched; RViz OpenGL initialization succeeded.

No physical robot motion or real-camera calibration was performed.

## Known limitations and log messages

The world now has one black and one white 20 mm cube (see BOX_SIMULATION.md).
They are not yet synchronized into the MoveIt planning scene. This is not a
completed sorting-cell simulation.
Box contact, grip retention, bin collisions and the real payload are unvalidated.
The initial DART run did not move the gripper correctly. The final configuration
uses Bullet Featherstone, and actual simulated finger position was checked.
Do not infer realistic hinged-linkage contact mechanics from this proxy.

MoveIt may log that no 3D sensor plugin or /recognize_objects action is available.
This project uses no depth sensor or object-recognition action; those messages do
not prevent the tested joint planning/execution. RViz may log a plugin factory
namespace warning while still loading successfully. MoveIt processes segfaulted
during shutdown after successful tests in this installation. Runtime motion
tests passed. The launcher includes child-process cleanup and a duplicate-launch
lock to prevent orphan Gazebo processes from conflicting on restart.

## Files to learn from

- urdf/: geometry, joint axes and ros2_control hardware selection.
- config/provisional_arm.srdf: groups, named poses and adjacent collision pairs.
- config/kinematics.yaml and ompl.yaml: IK and planning algorithms.
- config/controllers.yaml: simulated position trajectory controllers.
- config/moveit_controllers.yaml: planner-to-controller action mapping.
- config/simulation.rviz: visualization layout.
- launch/simulation.launch.py: starts components and orders controller loading.
- worlds/workcell.sdf: Gazebo ground, light and physics plugins.

After editing, run scripts/build.sh from the robot-arm project and restart the
simulation. Never transfer these provisional limits, masses or axes to hardware.

## Reinforcement learning

A simulation-only SAC reaching trainer is available in `reinforcement_learning/`.
See `reinforcement_learning/README.md` for training, resuming and evaluation.
It saves policy and replay experience after every completed attempt. It does not
yet learn box grasping or sorting. Do not command the arm from RViz while training.

## Fixed-bin sorting demo

The separately selected sorting scene now supports a verified six-box mixed-colour scripted
pick-and-place demonstration. Run `scripts/start_sorting_demo.sh` after stopping
other simulation instances. See `sorting/README.md` for fixed bin positions,
collision-scene integration, and the explicit attachment-based grasp abstraction.
This is separate from the reaching-only reinforcement-learning policy.
