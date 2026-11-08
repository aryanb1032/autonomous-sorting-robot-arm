# Mixed black/white boxes → two fixed bins

The default demo now uses **six 20 mm cubes: three black and three white**.
Colours are shuffled and pickup positions/yaws are randomized within a reachable annular sector,
with at least 65 mm centre spacing. Both bin positions stay fixed. Each bin has three drop slots.

## Start a fresh batch

Stop any existing simulation or trainer first, then run:

```bash
bash /path/to/robot-arm-project/scripts/start_sorting_demo.sh
```

This generates a new layout, builds the ROS package, opens Gazebo and RViz, waits
for MoveIt, and sorts all six objects. The windows stay open when sorting ends.
Press Ctrl+C in its terminal to close them before starting another batch.

Reproduce the verified layout:

```bash
bash /path/to/robot-arm-project/scripts/start_sorting_demo.sh --seed 42
```

Use two of each colour:

```bash
bash /path/to/robot-arm-project/scripts/start_sorting_demo.sh --per-color 2 --seed 17
```

Supported counts are 1–3 per colour. The seed is saved in `layout.json`. The
randomization is constrained to an accessible arc with clearance between objects;
it is not unrestricted random clutter or a pile. Do not regenerate files while
a simulation is active. MoveIt considers other boxes, the ground and bin walls.
Do not send competing RViz Execute or controller commands during a batch.

## Fixed bins and randomized sources

All positions use the world frame, in metres. Black bin centre:
`[0.136079, -0.267362, 0.015]`; white bin centre:
`[0.136079, +0.267362, 0.015]`. The Z coordinate is the resting box centre, not
the top of a bin wall. Each bin is 100 × 100 mm outside, 90 × 90 mm inside,
with a 5 mm floor and 40 mm walls. Separate slots keep released boxes apart.

Pickup boxes sit directly on the ground with resting centres at Z=10 mm. Their radial
positions are 275–335 mm, with angles within ±0.72 radians and small randomized yaw rotations.
There are no pickup pedestals.
Each box has an assumed mass of 10 g. The 20 mm cube height is a demo assumption.

## What is actually tested

For every box: approach, descend, check alignment, partially close gripper,
attach, lift, verify following, carry, lower, open, detach, verify settling inside
the assigned bin, retreat. The arm finally returns to ready. Actual Gazebo poses
supply verification; a commanded destination alone does not count as success.
Every corner of the cube must fit inside the assigned bin and the box must remain
near its resting height for 0.6 seconds. `last_result.json` records the completed
batch. Failure stops the controller rather than marking a box sorted.

MoveIt tracks the carried object as an attached collision object. Contact between
that object and its own support is allowed only during lift-off, then disabled.
Grasp pitch varies with target height to accommodate the revised long wrist/tool.
The final axis distances are loaded from config/geometry.yaml; fingers are hinged proxies.
An exclusive lock prevents the reaching trainer and sorting controller running
together.

This is **scripted sorting with known simulation identities**, not camera-based
colour recognition or a learned sorting policy. Gazebo's DetachableJoint plugin
represents a grasp; realistic finger friction and grip force are not validated.
The plugin is detached for release, after which gravity settles the box. No
serial/physical-arm backend is used. The separate SAC reaching policy is unchanged.

## Learn and customize

Read `../docs/BUILD_YOUR_OWN_ARM.md` for the complete file-by-file tutorial,
commands, URDF/Xacro explanation, Gazebo/MoveIt setup, CAD mesh use, and adapting
a different arm in an independent workspace.

`build_workcell.py` generates the world, layout and attachment plugins together.
`sort_boxes.py` controls and verifies the task. Its analytic IK contains this
arm's dimensions: changing only the URDF is not sufficient for a different arm.

Plugin documentation: https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DetachableJoint.html
