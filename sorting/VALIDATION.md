# Sorting verification — 2026-10-26

- Sorting-world SDF validation passed; ROS packages rebuilt successfully.
- Six pickup/carry/release poses checked against MoveIt collision geometry.
- A straight-down grasp was rejected for palm/forearm self-collision; the tested
  tilted approach preserves all original robot dimensions and collision checking.
- Three unit tests passed: analytic IK, correct/wrong bin, edge/air rejection.
- Complete Gazebo cycle passed: both boxes lifted, carried, released, and observed
  settled in their correct bins; robot returned to ready.
- Black resting centre: [0.1993742, -0.2262498, 0.0150000] m.
- White resting centre: [0.1973148, 0.2253290, 0.0149997] m.
- Independent live Gazebo pose check confirmed all eight corners of each cube
  are inside the correct bin after the completed run.
- No camera classification, RL sorting policy, contact-friction grasp validation,
  physical servo movement, or real-arm transfer was performed.

The one-command launcher was added after this integration run; the successful
run used the same simulation launch and sorter via separate commands.

## Mixed-box extension — 2026-10-26

- Full integration run with seed 42: 6 cubes (3 per colour), mixed pickup order.
- All six boxes lifted, transported, released and verified inside their assigned
  bins using all-corner containment plus a sustained resting-height check.
- Arm returned to ready; one-command launcher completed its task successfully.
- Six unit tests include 100 generated layouts: balanced/mixed colours, minimum
  pickup spacing, reachable source IK, seed repeatability, fixed bin positions,
  drop-slot clearance, and rejection of wrong-bin/edge/air placements.
- Other random seeds have structural/IK tests, not full physics-run verification.
- No camera-based colour classification or RL sorting policy was used.
