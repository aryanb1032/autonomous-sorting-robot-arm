> Updated 2026-11-02: all six home angles are now 90. The earlier physical success used gripper home 80; the new behavior requires uploading and hardware verification.

# One virtual box, empty physical arm

The complete Gazebo preview passed on 2026-11-01: approach, descend, virtual grasp,
lift, carry, release into the black bin, retreat, home. See
`work/single-pick-preview.log` at the workspace root. The real arm has NOT yet run
this task. This is scripted MoveIt motion, not reinforcement learning or vision.

`single_box.py` plans arm motion with MoveIt, validates sampled joint positions,
and sends each linear segment to Gazebo. With `--physical`, it sends the same
arm targets to the ESP32 and waits for both controllers before proceeding.
Timing is approximate; ESP32 completion reports commanded positions, not measured
servo angles. Simulated grasp uses Gazebo attachment, not a physical grasp test.

Channels: 0,1,2,5,6,7. Arm model-to-servo signs: +,-,+,+,+. M4 rolls; M5 pitches.
Arm home is 90 degrees. Empty gripper gestures use 90 degrees open/home and
0 degrees closed; these are not calibrated jaw apertures or gripping force.
The virtual pickup uses `black_box_01` in the saved sorting layout.

Prepared firmware: `firmware/virtual_pick`, identity `VIRTUAL_PICK_HOME90_V2`.
It commands home on boot unless STOP is latched; INIT checks readiness. STOP disables torque and persists across reboot. Heartbeat timeout while
moving holds the last command. Normal success holds home. No automatic return
home on communication failure or power loss. See hardware/HOME90_POLICY.md. Keep the arm supported when disabled.

Launch Gazebo with `sorting:=true` using `simulation_env.sh`. Default script mode
is simulation only. Physical execution additionally requires `--physical
--confirm-supported-home`, a fresh virtual scene and the real supported home pose.
Scene setup occurs before connecting the physical controller. Do not rerun after
an interrupted physical path without restoring the real starting pose.

The STOP panel's O key works while that window has focus. It is a software stop;
keep physical servo-power cutoff accessible. A prior motion lost MCU completion;
the cause is unresolved. The new runner logs MCU replies and reports separately
whether Gazebo, the MCU ACK, or the MCU DONE was missing. It stops on reset/error
messages and never automatically retries motion.
