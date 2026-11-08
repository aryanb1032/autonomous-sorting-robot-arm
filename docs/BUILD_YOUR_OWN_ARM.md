> Current model revision: **2026-10-29-final-dimensions**. See [final geometry notes](../model/FINAL_GEOMETRY.md). Previous validation reports and RL checkpoints apply only to their original geometry.

# Build, understand and change a ROS 2 robotic arm

This guide uses your actual project, Ubuntu 24.04, ROS 2 Jazzy, MoveIt 2 and
Gazebo Harmonic. It explains both the existing files and how to make a separate
arm project. Commands in a block should be run in order. Keep the terminal that
runs a simulator open. Stop a running process with Ctrl+C.

## 1. What is implemented, and what is assumed

The model has five revolute arm joints and one commanded translating gripper
joint. The second finger mimics the first. Six servos including the gripper does
not mean six independent arm axes. Your confirmed shoulder–elbow distance is
120 mm and elbow–wrist distance is 110 mm. Other geometry and physical properties
remain provisional. The model was built from code using simple shapes; it was
not imported as an exact GrabCAD CAD assembly. Screenshots do not provide enough
information to recover joint axes, all dimensions or accurate collision meshes.

The sorting demonstration uses several black/white 20 mm cubes. Cube height and
10 g mass are assumptions. It reads known simulation identities and measured
Gazebo poses, not camera images. A temporary attachment joint represents a grasp.
MoveIt plans the motion. This is different from the separate SAC reaching trainer;
we have not trained a colour-sorting policy or validated a real friction grasp.

## 2. How I opened and edited the files

I used terminal commands and Python/text edits directly on files in your workspace.
I did not click an Import URDF button in Gazebo, and did not download an exact
GrabCAD mesh. The launch file reads and expands the Xacro at startup, then sends
the resulting robot description to ROS and Gazebo.

Your project directory is:

```text
/path/to/robot-arm-project
```

Open Ubuntu's Files application, press Ctrl+L, paste that path and press Enter.
Open `ros2_ws/src/arm_simulation/urdf/provisional_arm.urdf.xacro` with a text editor.
For terminal editing:

```bash
cd /path/to/robot-arm-project
nano ros2_ws/src/arm_simulation/urdf/provisional_arm.urdf.xacro
```

In nano: Ctrl+O, Enter saves; Ctrl+X exits. Make a copy before experimenting:

```bash
cp ros2_ws/src/arm_simulation/urdf/provisional_arm.urdf.xacro /tmp/arm-before-edit.xacro
```

`pwd` shows your folder; `ls` lists files; `cd` changes folder; `cat FILE` prints
its contents. Editing source does not update a robot that is already running.
Rebuild when needed, stop the old simulation, and launch again.

## 3. The complete data flow

```text
Xacro source ──expand──> URDF /robot_description
                           ├──> robot_state_publisher ──> /tf ──> RViz
                           ├──> Gazebo robot entity
                           └──> MoveIt robot model

Gazebo world SDF ──> ground, pickup supports, boxes and bins

Sorting controller ──> MoveIt /move_action ──> planned trajectory
    ↑                                             ↓
Gazebo object poses                 /sim_arm_controller/follow_joint_trajectory
                                                  ↓
                                  ros2_control + gz_ros2_control
                                                  ↓
                                           Gazebo joints
                                                  ↓
                                  /joint_states ──> ROS/MoveIt/RViz
```

RViz displays ROS data and planned motion; it does not simulate gravity. Gazebo
simulates physics; it does not automatically understand your sorting task.
MoveIt plans paths and checks its own collision scene; it does not automatically
copy every Gazebo obstacle. Our sorting controller explicitly adds those obstacles.

## 4. Where each file belongs

All paths below are relative to the project directory above.

| File/folder | Purpose |
|---|---|
| `ros2_ws/src/arm_simulation/package.xml` | Package identity and dependencies |
| `ros2_ws/src/arm_simulation/CMakeLists.txt` | Installs resources and executable scripts |
| `.../urdf/provisional_arm.urdf.xacro` | Editable robot geometry, joints and control interfaces |
| `.../urdf/provisional_arm.urdf` | Expanded snapshot; NOT the file the launch reads |
| `.../urdf/sorting_attachments.xacro` | Generated object-specific grasp plugins |
| `.../config/provisional_arm.srdf` | Planning groups, named poses and collision exemptions |
| `.../config/kinematics.yaml` | IK solver configuration |
| `.../config/ompl.yaml` | Motion planning pipeline |
| `.../config/joint_limits.yaml` | Planning velocity/acceleration limits |
| `.../config/controllers.yaml` | Simulated arm/gripper controller definitions |
| `.../config/moveit_controllers.yaml` | MoveIt action-to-controller mapping |
| `.../config/simulation.rviz` | Saved RViz display and camera settings |
| `.../worlds/workcell.sdf` | Ordinary simulation world |
| `.../worlds/sorting_workcell.sdf` | Generated mixed-box sorting world |
| `.../launch/simulation.launch.py` | Starts the system and orders controller startup |
| `sorting/build_workcell.py` | Randomized layout and attachment generator |
| `sorting/layout.json` | Generated seed, object identities, positions, supports and bins |
| `sorting/sort_boxes.py` | Sorting state machine, scene updates and placement checks |
| `sorting/run_demo.py` | Starts a fresh scene and runs one sorting batch |
| `reinforcement_learning/reach_env.py` | Separate learning environment for reaching |
| `scripts/build.sh` | Builds both existing ROS packages |
| `simulation_env.sh` | Loads ROS, Python environment, package overlay and simulation domain |

`src/` is where you edit. `build/`, `install/` and `log/` are generated by colcon.
Do not treat `install/` as your master source folder.

## 5. Describe a robot before writing code

Create a measurement sheet. For every joint record:

- Parent link and child link.
- Joint type: fixed, revolute, continuous or prismatic.
- Position of the joint centre relative to the parent link frame.
- Rotation axis in the joint frame and direction of positive motion.
- Zero pose, lower/upper limits and maximum speed.
- Link dimensions, mass and centre of mass.
- Gripper linkage and the frame where a grasp target should be expressed.
- Camera mounting transform, when relevant.

Use metres, radians, kilograms and seconds in the model. Thus 120 mm = 0.120 m;
180 degrees = pi radians. Draw coordinate frames at joint centres before exporting
CAD. CAD mesh origins and robot joint origins often differ.

## 6. URDF links: shape is not a joint

A link is one rigid body. Its three descriptions have different jobs:

```xml
<link name="upper_arm">
  <visual>
    <origin xyz="0 0 0.060"/>
    <geometry><box size="0.035 0.035 0.120"/></geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0.060"/>
    <geometry><box size="0.035 0.035 0.120"/></geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0.060"/>
    <mass value="0.3"/>
    <inertia ixx="0.000390625" ixy="0" ixz="0"
             iyy="0.000390625" iyz="0" izz="0.00006125"/>
  </inertial>
</link>
```

This is an explanation of one link, not a complete robot file. The link frame is
at one end; its 120 mm box has its centre 60 mm along local Z. `visual` controls
appearance. `collision` is the shape used for contacts/planning. `inertial`
controls dynamics; it is not decorative. For a uniform box with dimensions x,y,z:
Ixx=m(y²+z²)/12, Iyy=m(x²+z²)/12, Izz=m(x²+y²)/12. Use CAD mass properties for a
real assembly rather than assuming a solid uniform block.

## 7. URDF joints: how links connect

Your elbow connection is represented by:

```xml
<joint name="sim_joint_3" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 0.120" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="2.0" effort="20" velocity="0.6"/>
</joint>
```

The elbow centre sits 120 mm along the upper-arm frame. It rotates about local Y.
The limits are radians, effort is Nm for this revolute joint, and velocity is rad/s.
For a prismatic joint, position is metres, effort is N, and velocity is m/s.
The numbers above are existing simulation assumptions, not verified servo ratings.

A visual origin moves the drawing. A joint origin moves the kinematic connection.
Changing only the drawing length does not change where the elbow rotates. This is
why the same length parameter must drive joint offset, geometry and inertia.

## 8. Xacro: variables and reusable parts

URDF is the expanded XML description. Xacro generates that XML using variables,
expressions, macros and conditional sections. In your model:

```xml
<xacro:property name="shoulder_to_elbow" value="0.120"/>
<xacro:property name="elbow_to_wrist" value="0.110"/>
```

The geometry uses `${shoulder_to_elbow}`, its centre uses
`${shoulder_to_elbow / 2}`, and the elbow joint offset uses the same variable.
To test a different measured upper-arm length, change the property, not every
literal number separately. However, this alone does not adapt the task-specific
IK or calibration code—see section 17.

Expand and inspect a model manually:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/robot-arm-project
xacro ros2_ws/src/arm_simulation/urdf/provisional_arm.urdf.xacro > /tmp/my_arm.urdf
```

The launch does the equivalent through Python's `xacro.process_file(...)` and
passes its XML output as the `robot_description` parameter. `gazebo:=true` selects
the simulated hardware plugin; `sorting:=true` adds generated grasp attachments.
Normally, edit the Xacro and leave the generated URDF alone.

## 9. Replacing boxes with CAD meshes

Export each rigid assembly as a separate mesh. Keep moving links separate.
Put meshes in a package `meshes/` directory and install it through CMake:

```cmake
install(DIRECTORY urdf meshes config launch worlds DESTINATION share/${PROJECT_NAME})
```

For a mesh exported in millimetres:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://my_arm_description/meshes/upper_arm.stl"
          scale="0.001 0.001 0.001"/>
  </geometry>
</visual>
```

Use `scale="1 1 1"` if the mesh is already in metres. Verify scale with a known
dimension. Align the mesh relative to the joint frame using `origin`, not by
moving the joint merely to make the picture look right. Use simplified collision
shapes/meshes where practical; detailed visual meshes can be expensive for collision
checking. STL does not provide joint axes, motion limits or a complete URDF.

## 10. Add controllers and connect Gazebo

In a Gazebo model the ros2_control block selects:

```xml
<ros2_control name="SimulationOnly" type="system">
  <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
  <joint name="sim_joint_1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- Repeat for each independently commanded joint. -->
</ros2_control>
```

The Gazebo plugin loads `controllers.yaml`. The joint-state broadcaster publishes
measured simulated positions. The arm trajectory controller receives timed joint
goals; the gripper has its own trajectory controller. A mimic finger has state
interfaces but no separate command interface in this setup.

Names must agree across URDF, ros2_control, controllers.yaml, MoveIt controller
mapping, SRDF and task code. Changing `sim_joint_1` in only one place breaks the
connection. `use_sim_time=true` tells ROS nodes to use Gazebo's bridged `/clock`.
When Gazebo is paused, simulated time and motion stop.

Our physics engine is Bullet Featherstone because it passed the gripper mimic
motion test. Changing physics engines can change supported features and contacts;
rerun the motion/contact checks after such a change.

## 11. Configure MoveIt 2 for your own arm

The current configuration was written directly as XML/YAML and tested. For a new
arm, the GUI Setup Assistant is a useful alternative:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

In the GUI, create a new configuration and load your expanded URDF. Generate and
review the self-collision matrix. Add an arm planning group using the correct
base-to-tool chain, add the gripper group and end effector if present, define safe
named joint poses, and configure the controllers. Save a separate configuration
package in your new workspace. Match its controller names to your simulator.

Review collision exemptions rather than disabling checks to make a failed plan
pass. Our first straight-down sorting pose failed a genuine overlap check in the
proxy model; we changed the approach angle instead.

A five-axis arm cannot generally achieve every arbitrary six-dimensional tool
pose. Our general KDL configuration uses position-only IK; the sorting example
uses an analytic solution with a fixed tilted orientation. For a different arm,
choose IK and task orientations that its actual degrees of freedom support.

The SRDF adds planning meaning; it does not replace the URDF geometry. Regenerate
or review its groups, end-effector references, named poses and collision matrix
whenever the kinematic structure changes.

## 12. World SDF versus robot URDF

The world SDF contains the world, physics systems, lighting, floor and objects.
The robot is spawned separately from `robot_description` by `ros_gz_sim create`.
A dynamic box needs a visual, collision shape, mass and valid inertia. A bin is
represented here by a floor plus four fixed walls; a visual-only bin would not
catch a falling box.

The mixed scene generator writes all three matching files together:

1. `sorting_workcell.sdf`: physical boxes, supports and bins.
2. `layout.json`: object IDs, their assigned colour, spawn positions and bin slots.
3. `sorting_attachments.xacro`: one independently controlled grasp plugin per box.

Do not regenerate a layout while its simulation is running. The running world
would still contain the previous objects. Start a fresh scene after generation.

## 13. Build and launch the existing project

Dependencies were installed through the setup scripts. `scripts/setup_simulation.sh`
checks ROS package availability and installs missing packages using sudo when
needed. You do not need to reinstall ROS for every new robot.

```bash
cd /path/to/robot-arm-project
bash scripts/build.sh
```

This runs colcon with `--symlink-install`. To run the arm without the sorting task:

```bash
bash scripts/start_simulation.sh gazebo
```

To run the mixed-box sorting demo from a fresh start:

```bash
bash scripts/start_sorting_demo.sh
```

It generates a new seed and six cubes, builds the package, starts Gazebo/RViz and
MoveIt, then sorts the batch. To reproduce a particular layout:

```bash
bash scripts/start_sorting_demo.sh --seed 42
```

To use two boxes of each colour instead of three:

```bash
bash scripts/start_sorting_demo.sh --per-color 2 --seed 17
```

The current layout supports 1–3 boxes per colour. More boxes require changing
pickup spacing and bin capacity, not merely increasing a loop count. The default
randomization samples ground positions in a 275–335 mm annular sector with minimum
65 mm centre spacing, randomizes
yaw and shuffles the colour order. It is not an arbitrary dense pile or a clutter
perception benchmark. The seed is saved in `layout.json` and the result report.

The single command keeps the simulator open after completion. Press Ctrl+C before
starting a new batch. The boxes reset because the next run loads a fresh world.

## 14. What happens during a mixed-box sorting batch

The script initializes the grasp plugins, opens the gripper and builds MoveIt's
scene from Gazebo pose feedback. It then processes each object in the mixed order:

1. Select that object's recorded colour and its corresponding fixed bin.
2. Select an unused drop slot in the bin.
3. Plan to an approach pose, then descend to the measured pickup location.
4. Check that the tool is within 8 mm of the object centre.
5. Close the gripper partially and attach the box in Gazebo and MoveIt.
6. Lift, verify height/following, then remove support-contact permission.
7. Plan to the bin slot and lower the held object.
8. Open, detach, and wait for gravity to settle the box.
9. Check every cube corner is inside its assigned bin for at least 0.6 seconds.
10. Retreat and continue until every object is placed; return to ready.

All other boxes remain obstacles during planning. The supported pickup contact is
allowed only for the object/platform pair, not for arbitrary robot collisions.
The final report records per-object results. A failed check stops the batch.

## 15. Useful diagnostics

In a separate terminal, load this project's environment first:

```bash
source /path/to/robot-arm-project/simulation_env.sh
ros2 node list
ros2 topic list
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 action list
```

Expected active controllers are `joint_state_broadcaster`, `sim_arm_controller`
and `sim_gripper_controller`. Simulation uses domain 42 and a dedicated Gazebo
partition; use the same environment in every diagnostic terminal.

| Symptom | First things to check |
|---|---|
| Package not found | Source `/opt/ros/jazzy/setup.bash`, build, source workspace overlay |
| Robot is 1000 times too large | Mesh units and scale |
| A link spins around the wrong point | Joint origin and mesh offset |
| RViz robot moves but Gazebo does not | Mock backend vs Gazebo, active controllers, paused clock |
| Boxes visible in Gazebo but absent from planning | Explicit MoveIt scene synchronization |
| Planner rejects a goal | Joint bounds, collisions, IK reachability and current state |
| Nothing moves after pressing play | Controller/action failure logs and joint feedback |
| Simulation already running | Stop its owning terminal; do not start a duplicate |
| No fresh joint feedback | Gazebo server closed/paused or controller not publishing |
| Object is not at pickup point | Start a fresh sorting scene; don't reuse a completed batch |

## 16. Make a separate arm workspace without damaging this project

The simplest learning exercise is to copy the working simulation package into a
new workspace. Run this once with a new destination directory:

```bash
mkdir -p ~/my_new_arm_ws/src
cp -a /path/to/robot-arm-project/ros2_ws/src/arm_simulation ~/my_new_arm_ws/src/
```

Keep the package name temporarily. Open a NEW terminal for the new workspace;
do not source the old robot-arm activation script in that terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/my_new_arm_ws
nano src/arm_simulation/urdf/provisional_arm.urdf.xacro
colcon build --symlink-install --packages-select arm_simulation
source install/setup.bash
export ROS_DOMAIN_ID=43
export GZ_PARTITION=my_new_arm_learning
ros2 launch arm_simulation simulation.launch.py backend:=gazebo
```

The default launch has sorting disabled. Start by changing only a measured link
length and inspecting the result. Then change geometry, joint offsets and axes
one at a time, rebuilding/restarting and checking the zero pose after each change.
The original project stays on domain 42 and remains separate.

If the plain `colcon` command is unavailable, install `python3-colcon-common-extensions`
or use the Python/colcon environment already installed for the original project.
Do not mix workspace overlays accidentally.

For a new package from scratch instead, begin with:

```bash
mkdir -p ~/my_arm_ws/src
cd ~/my_arm_ws/src
source /opt/ros/jazzy/setup.bash
ros2 pkg create --build-type ament_cmake my_arm_description
mkdir -p my_arm_description/urdf my_arm_description/meshes my_arm_description/launch my_arm_description/config my_arm_description/worlds
```

Add the complete robot tree to `urdf/`, configure `package.xml` dependencies, and
install the resource folders in CMake before `ament_package()`. Use the existing
launch file as a reference, replacing package names and paths. Recreate the MoveIt
configuration for the new robot. Do not expect package creation alone to produce
a robot model or a working simulator.

## 17. Exactly what must change for a DIFFERENT arm

| Change | Files/code to review |
|---|---|
| Link lengths or mesh shape | Xacro geometry, joint origins, inertial properties, collision meshes |
| Joint count/order/names | URDF tree, ros2_control interfaces, controllers.yaml, MoveIt mapping |
| Joint axes or zero pose | Joint origins/axes, named poses, IK, hardware calibration |
| Motion limits | URDF limits, planning joint_limits.yaml, controller tolerances |
| Gripper type | Finger joints/mimic relation, actuator mapping, tool frame, grasp logic |
| Camera mount | Camera joint transform and camera/hand-eye calibration |
| Payload or link mass | Mass/inertia and dynamics testing |
| Planning chain | SRDF groups, end effector, IK solver and collision matrix |
| Pickup/bin locations | Reachability, world SDF, planning scene and task configuration |

Critical: `sorting/sort_boxes.py::joints()` reads the shared geometry through
`model/geometry.py`. Current derived dimensions are: 0.120 m, 0.110 m, shoulder height 0.054 m, combined wrist/tool offset 0.230 m,
with a height-dependent grasp pitch. Change config/geometry.yaml rather than
duplicating numeric lengths in the Xacro; a different axis arrangement still needs a new solver.
For a different arm, replace this solver with a verified model-based IK solution
or deliberately rederive it, then validate against forward kinematics and limits.
Also update the separate reaching environment's joint list, action limits, reset
pose and model checks. Do not load the old RL policy onto a different morphology
and assume it still works.

Renaming the package requires updating `package.xml`, CMake's `project(...)`,
package lookup calls, mesh `package://` URLs and relevant launch commands.
Renaming the robot requires agreement between URDF/SRDF and any model guards.

## 18. A practical learning sequence

First build one fixed link and inspect its scale. Add a joint and verify its axis.
Build the whole chain and check all joint-centre distances. Add inertias and test
physics. Activate controllers and test joint-space motion. Configure MoveIt and
check a planned joint goal. Add obstacles and verify they are avoided. Implement
one pickup with a clearly stated grasp abstraction. Then add multiple objects,
measured success checks and failures. Add calibrated camera perception. Introduce
learning only after the task reward measures the behaviour you actually want.

For this arm, the next perception step is to replace scene-provided identities
with OpenCV/YOLO observations and calibrate camera-to-gripper-to-world transforms.
For contact-based grasping, validate finger friction, mass, collision geometry,
actuation and lift success before replacing the attachment abstraction.

## References

- MoveIt Setup Assistant: https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html
- URDF/SRDF concepts: https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html
- Jazzy Gazebo control integration: https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
- Gazebo model spawning: https://gazebosim.org/docs/harmonic/spawn_urdf/

The MoveIt pages above document the current main branch; GUI labels can differ
from your installed Jazzy version. This project's tested files are the concrete
reference for the commands used here.


## Physical robot commissioning

See [REAL_ROBOT_SETUP.md](REAL_ROBOT_SETUP.md) for wiring, power, diagnostic uploads, measured motor calibration, camera setup and the remaining physical integration gates. Boxes now sit on the ground without pedestals.
