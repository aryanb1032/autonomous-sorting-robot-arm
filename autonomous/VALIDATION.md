# Validation — 2026-11-02

- 19 unit/integration tests passed: autonomous contracts and policy, camera input,
  independent calibration validation, and corrected reaching targets.
- Two-cube episode completed: one black and one white, both placements verified,
  return home completed. Evidence: runs/20261102-105916/episode-001/result.json.
- Six-cube episode completed: three black and three white, zero blocked objects,
  all placements verified and return home completed. Evidence:
  runs/20261102-110359/episode-001/result.json and events.jsonl.
- STOP integration check passed during an active APPROACH: controller reported
  STOPPED / Operator STOP requested, launcher exited nonzero and cleaned up.
  See stop_validation.json for the run folder.
- Camera readiness correctly remains false because measured calibration, workspace,
  bins, latency and gripper aperture configuration are missing.
- No physical motor commands or firmware uploads were performed for these checks.

The first trial failed closing the gripper around a rotated cube. The simulation
now leaves sufficient finger clearance and explicitly uses a detachable-joint
grasp. Successful simulation does not validate real contact/force behaviour.

The policy checkpoint contains 9 updates: one failed attempt and eight successful
placements across the tests. These updates concern sorting-order selection only.
No claim of policy convergence, better performance, or learned grasp control.
