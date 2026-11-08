# Validation for 2026-10-29-final-dimensions

Completed checks:

- ROS workspace: both packages build successfully.
- 36 software tests: vision, trajectory validation, calibrated gripper conversion, generated model, geometry revision rejection and expanded-URDF forward-kinematics comparison passed.
- Six ground-layout/analytic-IK tests passed, including 100 random layouts.
- Three reaching action-boundary tests passed.
- Live mock ROS hinged-gripper action passed: angle command converted through measured-table test data and both estimated finger angles published correctly.
- Generated ESP32 motor firmware compiled using synthetic calibration, without uploading. Test fixtures under work/ are NOT actual servo calibration.

The first Gazebo attempt exposed old metre-based gripper tolerances, which were replaced with angular tolerances and a slower command. A later release caught a box on a bin wall; drop slots were moved inward without changing either bin's fixed centre. Final Gazebo batch (seed 42): all six ground boxes were verified inside their correct fixed bins, and the arm returned to ready. The revision-tagged report is sorting/last_result.json.

Physical limits: geometry is a dimensional proxy, not exact CAD; linkage pivots, masses, motor limits, channels and camera mount still need measurement. No physical motor motion, firmware upload or RL retraining occurred during this change. Previous RL runs and older test reports must not be treated as validation of this revision.

Logs: work/revised-final-build.log, work/revised-sorting-final.log, work/revised-simulation-final.log, work/revised-hardware-mock.log, work/revised-hardware-compile.log (relative to the parent task workspace).
