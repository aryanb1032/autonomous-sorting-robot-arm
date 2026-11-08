# Shared no-box trajectory

This stage uses model revision 2026-11-01-wrist-roll-pitch. It is a home→ready→home
motion, not sorting. The model and physical command order are M1 base, M2 shoulder,
M3 elbow, M4 wrist roll, M5 wrist pitch. Model +X is left in the user's marked photo.
Servo degrees = 90 + sign × degrees(model joint), signs [+1,-1,+1,+1,+1].
These are nominal user-observed mappings; there are no real encoders.

prepare_plan.py asks MoveIt to PLAN ONLY, checks returned points for collision,
resamples edges to at most2-degree increments, retimes them conservatively, and
saves plan.json with a content fingerprint. The gripper stays at90 degrees.
run_plan.py defaults to Gazebo ONLY. --physical --confirm-supported-home adds
physical commands to the same saved target sequence. Each segment must complete
in both before the next begins. The real MCU and Gazebo interpolate position-only
segments linearly. USB/ROS delays mean exact time synchronization and identical
actual motion cannot be guaranteed. Physical DONE means command completion only.

Firmware/live_trial is a dedicated bounded trajectory controller. Physical joint
commands are limited to ±45 model degrees for this trial, below the user's reported
full mechanical range. Maximum firmware rate is0.2rad/s; saved targets use ≤0.12rad/s.
Gripper remains at90 throughout this run. Startup commands all six servos to 90 degrees automatically unless STOP is latched.
STOP disables outputs and latches across reset; loss of heartbeat stops motion and holds last command.
I2C output failure disables OE. A normal successful run holds home.
An explicit HOME90 command clears the STOP latch and commands home. INIT only
verifies readiness at home; it cannot clear STOP. See hardware/HOME90_POLICY.md. Support the arm during upload/reset and stopping.

The O-key STOP panel uses control/stop.request to reach the active serial owner,
then requests Gazebo pause. Its window must have focus. It is software, not a
hardware emergency stop. Keep physical servo-power cutoff accessible.

Use the project's simulation_env.sh before running the Python files. Upload
firmware/live_trial only with servo power off. Do not run either plan preparer or
RViz Execute while replay is active. If stopped midpath, do not retry the stored
path without restoring and verifying the starting pose.
