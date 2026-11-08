> 2026-11-02: wrist compensation corrected to M4 roll / M5 pitch; task revision gazebo_reach_m4roll_m5pitch_v2. Start a new training run. This is simulation-only reaching, not vision-based grasping or sorting. Phone video is not yet an RL observation.

> Current model revision: **2026-10-29-final-dimensions**. See [final geometry notes](../model/FINAL_GEOMETRY.md). Previous validation reports and RL checkpoints apply only to their original geometry.

# Learning from attempts in Gazebo

This is the first learning stage: move `tool0` toward a target, using the current
120 mm / 110 mm arm model. It does **not** yet learn grasping, colour sorting,
vision, object contact, or real servo control. The target is a position, not a
complete grasp pose. There is no guaranteed improvement after each attempt.

## Run

Start the existing Gazebo simulation, and leave it playing. Do not issue RViz
Execute commands or other controller commands while training owns the arm.

```bash
bash /path/to/robot-arm-project/scripts/start_simulation.sh gazebo
```

The delivered checkpoint already contains two short training attempts. Use the
resume command below to continue it. For a new run, use a different `--output`
directory. The following command starts training only when no checkpoint exists:

```bash
bash /path/to/robot-arm-project/scripts/train_reaching.sh --episodes 20
```

Resume from saved policy **and experience buffer**:

```bash
bash /path/to/robot-arm-project/scripts/train_reaching.sh --episodes 20 --resume
```

One attempt ends when the tool is within 15 mm of the target or after 30 actions.
SAC collects the attempt, performs neural-network updates, then saves progress.
Each action changes three joints by at most 0.06 radians; wrist pitch compensates
for shoulder/elbow angles and wrist roll stays zero. MoveIt plans and executes
these bounded joint goals. Reward uses measured joint feedback and MoveIt forward
kinematics, not the commanded position. The observation contains five measured
joint angles, target XYZ and target-minus-tool XYZ.

Reward = 30 × distance reduction − remaining distance − 0.01 per action,
minus a small action penalty. Planning failure costs 1; reaching the target adds 5.
This encourages reaching efficiently. It does not measure grip strength or object
pickup. Targets come from a small joint-space region around the ready pose and start
at least 30 mm from the tool, outside the 15 mm success radius.

For a visible green target, add an RViz **Marker** display with topic `/rl_target`.
It is republished after every step. The target is a visual marker, not a box.

## Saved progress and stopping

`runs/reach/latest.json` points to the latest complete model ZIP and replay-buffer
PKL. The previous checkpoint is also retained. `episodes.monitor.csv` records
reward, episode length, success and final distance. Ctrl+C stops the trainer;
resume uses the last completed attempt (an interrupted attempt is not saved).
Load only locally trusted checkpoints, as model/replay loading deserializes files.

The Gazebo plugin in robot_description, advancing simulation clock, domain 42,
and sim_* joints are checked before training. There is no hardware option.
Planning/execution service failures stop training; ordinary planning failures
receive a penalty. Do not connect hardware on this simulation domain.

## How this fits the sorting robot

OpenCV/YOLO identifies box colour and position. MoveIt supplies collision-checked
motion. SAC is the learning component: it learns which small joint changes earn
more reward from accumulated experience. MoveIt remains responsible for executing
those proposed changes. RL does not replace camera calibration or the robot model.

Before a sorting-learning stage, add real box and bin collision geometry, gripper
contact/friction, automatic object reset and a measured success condition: grasp,
lift without dropping, then release in the correct bin. Reward that observed
outcome. Test a frozen policy on fixed held-out scenarios against an untrained
baseline before claiming improvement. Randomize object placements and relevant
physics only after contact behaviour is validated. Physical transfer additionally
needs calibrated hardware, limits, sensing and supervised tests.

Full Gazebo + MoveIt episodes are relatively slow. A short smoke run proves the
software loop, not a learned skill. Twenty episodes is a starting batch, not a
promise of convergence. No background training continues after the requested
number of episodes.

Dependencies are in requirements.txt; installed versions are SB3 2.9.2 and
Gymnasium 1.4.0. PyTorch uses the existing CPU installation.

References:
- https://gymnasium.farama.org/introduction/create_custom_env/
- https://stable-baselines3.readthedocs.io/en/master/modules/sac.html

## Compare a frozen policy with random actions

After training, source `simulation_env.sh`, then run (one at a time):

```bash
python reinforcement_learning/evaluate.py --episodes 10
python reinforcement_learning/evaluate.py --episodes 10 --random-baseline
```

Both use target seeds 1000–1009, separate from the training seed. Reports go to
`runs/reach/evaluation_policy.json` and `evaluation_random.json`. Neither command
updates the model. Compare success rate and final distance across repeated runs;
a positive reward alone does not establish improved performance.


Geometry revision 2026-10-29: new training defaults to runs/reach_2026-10-29-final-dimensions. Earlier runs/reach checkpoints are historical and are rejected by the geometry guards. Retrain on this model before evaluating a learned policy.
