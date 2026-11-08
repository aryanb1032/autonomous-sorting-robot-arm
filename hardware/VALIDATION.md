> HISTORICAL: results below used the pre-29-October geometry. Current results are in ../model/VALIDATION.md.

# Preparation validation — 26–27 October 2026

- ROS workspace build: passed.
- Python dependency consistency (`pip check`) and required import checks: passed.
- Core and hardware unit tests: 25 passed.
- Ground-layout/sorting tests: 6 passed, including randomized spacing checks.
- Mock ROS actions: invalid-joint rejection, execution and cancellation passed.
- Mock gripper integration: a 0.0125 m slide command was converted to servo radians; both published finger estimates reached 0.0125 m.
- Firmware compilation: serial_check, pca_diagnostics, locked arm_controller and locked CameraWebServer passed. Binary hashes are in firmware/build/manifest.json.
- Generated calibrated firmware also compiled using a synthetic test fixture. That fixture is not real calibration and must never be uploaded to this arm.
- Ground Gazebo batch, seed 42: all six 20 mm boxes sorted into their assigned bins. No pickup pedestals. This uses scripted planning and simulated grasp attachments, not physical perception or learned sorting.
- Physical serial board detection: no board found during preparation. No uploads or real motor movement performed.
- Camera compilation uses an unconfirmed ESP-EYE pin selection with runtime camera initialization disabled. Correct PCB/FQBN selection is still required.
- Missing real measurements remain null in hardware/profile.template.json. No calibrated hardware/generated deployment has been produced.

Logs are in the workspace work/ directory: hardware-build-final.log, hardware-ros-build.log, calibrated-fixture-build.log, hardware-mock-launch.log and ground-sorting.log.

Read ../docs/REAL_ROBOT_SETUP.md before physical commissioning. A completed mock test does not validate real power, geometry, grip, servo tracking or collision avoidance.
