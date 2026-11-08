# Black and white boxes in Gazebo

The workcell now starts with one black cube and one white cube.

| Property | Value |
|---|---|
| Dimensions | 20 × 20 × 20 mm each |
| Assumed mass | 10 g each (not measured from real boxes) |
| Black starting centre | x = 220 mm, y = −60 mm, z = 11 mm |
| White starting centre | x = 220 mm, y = +60 mm, z = 11 mm |
| Frame | Gazebo world; ground is z = 0 |

The cubes spawn 1 mm above the ground and settle under gravity. Both have solid
box collision geometry and equal mass/inertia. The cube height is an assumption
for this demonstration; only a 20 mm side was previously confirmed.

They were also inserted into the running Gazebo world without restarting the
active reaching trainer. In Gazebo's Entity Tree select `black_box` or `white_box`
and use the context menu's move-to/focus option to inspect the small objects.

Future simulation launches include both cubes automatically:

```bash
bash /path/to/robot-arm-project/scripts/start_simulation.sh gazebo
```

World: `ros2_ws/src/arm_simulation/worlds/workcell.sdf`.
Standalone models: `black_box.sdf` and `white_box.sdf` in the same directory.

## Scope

These are physical Gazebo objects. They have not yet been synchronized into
MoveIt's planning scene or connected to camera detection, grasp verification,
object resets, sorting or RL rewards. Do not treat a MoveIt trajectory as
box-collision-aware yet. The existing trainer continues its reaching task and
its checkpoint is unchanged by adding these models. No picking skill is claimed.

Insertion uses Gazebo's supported EntityFactory create service:
https://gazebosim.org/docs/harmonic/spawn_urdf/
