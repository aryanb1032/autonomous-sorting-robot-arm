> Current model revision: **2026-10-29-final-dimensions**. See [final geometry notes](../model/FINAL_GEOMETRY.md). Previous validation reports and RL checkpoints apply only to their original geometry.

# Connecting the simulated sorter to your assembled robot

This guide refers to the project at `/path/to/robot-arm-project`.

## 1. What is ready, and what still needs measurement

The PC has ROS 2 Jazzy, MoveIt 2, RViz, Gazebo Harmonic, the ROS/Gazebo controllers, OpenCV, Python serial support, Arduino CLI and ESP32 Arduino core 3.3.12. The ROS workspace builds. Serial diagnostics, PCA9685 diagnostics, locked motor firmware and a locked camera example compile. The camera example was copied from the installed Espressif core; its selected board is a compile-test profile, not identification of your camera PCB.

The ground simulation sorted six 20 mm cubes: three black and three white, randomly placed in a reachable area, into two fixed bins. This simulation uses known object poses and an attachment to represent grasping. It does not prove camera recognition, frictional gripping or a learned sorting policy on hardware. The separate reinforcement-learning experiment learns reaching, not the complete sorting task.

No physical board was detected during preparation. Nothing has been uploaded and no physical motor test has been performed. Channels, calibrated pulse/angle limits, camera pinout, physical collision scene and grasp verification are still required. Those cannot be determined from a screenshot or downloaded.

Your six servos include the gripper, so the current model has **five arm axes plus a gripper**, rather than six independent positioning axes. The arm dimensions use shoulder–elbow 120 mm and elbow–wrist 110 mm. The existing geometry remains a provisional model, not an exact CAD replica. Measure the other joint origins, axes, tool offset and camera mount before physical planning.

## 2. How the pieces connect

```text
ESP32-CAM --Wi-Fi JPEG--> PC /camera/image_raw
                              |
                        OpenCV detection
                     colour + size + position
                              |
                    sorting task coordinator
                              |
                     MoveIt 2 trajectory
                              |
                  ROS serial trajectory bridge
                              |
                        USB serial cable
                              |
                       ESP32 Dev Module
                              |
                      I2C SDA21 / SCL22
                              |
                          PCA9685
                              |
                    six calibrated PWM signals
                              |
                         physical servos
```

The PC does vision, coordinate conversion and path planning. The ESP32 receives joint commands and generates servo pulses through the driver. The ESP32-CAM is a separate computer with separate firmware. Gazebo replaces the physical mechanism during simulation; RViz displays the robot and planning scene. MoveIt 2 is the ROS 2 version of MoveIt, so you do not need to install a second ROS 1 MoveIt stack.

Ordinary hobby servos do not report their actual angles through these PWM wires. The bridge publishes **command estimates**, not encoder measurements. A successful trajectory response means commands completed; it does not prove that a stalled servo reached its target.

## 3. Fix and verify power before connecting motors

You reported a **7 V, 7.32 A** supply. TowerPro specifies **4.8–6.6 V** for its MG996R: do not feed those servos directly from 7 V. Use a suitably rated regulated servo rail within the ratings of every attached servo; 6 V is a candidate only after verifying the exact DEAO and MG996R variants. Measure the rail with a meter before connecting servos. The supply's current label does not establish that six loaded servos can operate together: account for simultaneous peak currents, regulator limits, wiring and connector ratings.

Provide a physical means to remove servo power. Support the arm when outputs or power are disabled: it can fall under gravity. Software STOP holds the last command; it is not a mechanical brake or a safety-rated emergency stop.

| Connection | Wiring |
|---|---|
| ESP32 3.3 V | PCA9685 VCC, logic supply |
| ESP32 GND | PCA9685 GND and servo-supply negative |
| ESP32 GPIO21 | PCA9685 SDA |
| ESP32 GPIO22 | PCA9685 SCL |
| ESP32 GPIO23 | PCA9685 OE, proposed additional disable line |
| Regulated servo positive | PCA9685 V+ / suitable servo power distribution |
| Servo signal wires | Their measured, recorded PCA9685 channels |
| PC USB | ESP32 Dev Module programming and serial control |

VCC and V+ are different rails. Do not apply 7 V to ESP32 3.3 V, camera USB, or the PCA9685 logic VCC. Do not carry the six-servo load through the ESP32, USB connection or a small breadboard. Use appropriately rated distribution, connectors, wire and protection.

OE HIGH disables PCA outputs. Some boards have a default pull-down, so a 10 kΩ pull-up alone may produce an invalid intermediate voltage. For a board with a 10 kΩ pull-down, a 1 kΩ pull-up to 3.3 V is one possible arrangement; verify your board's actual circuit and confirm OE stays reliably HIGH while the ESP32 is resetting, before connecting servo V+. The prepared firmware drives GPIO23 HIGH before initializing the driver. It enables outputs only after a valid initialization command and valid calibration.

Sources: [MG996R manufacturer specifications](https://towerpro.com.tw/product/mg996R/) and [PCA9685 board pinouts](https://learn.adafruit.com/16-channel-pwm-servo-driver/pinouts). Your clone board and servo variants must also be checked.

## 4. Open the project and inspect the PC

Open a terminal with Ctrl+Alt+T. Run:

```bash
cd /path/to/robot-arm-project
bash scripts/hardware_preflight.sh
```

This lists boards, serial ports, user groups and dependency imports; it sends no motor commands. The current user was not in `dialout`. To grant normal serial access, run in your own terminal:

```bash
sudo usermod -aG dialout "$USER"
```

Then log out of Ubuntu and log back in. This changes your session's group membership. Do not use `chmod 777` on serial devices.

Connect only the arm ESP32 first, with servo V+ disconnected. Run preflight again and record its `/dev/serial/by-id/...` path. Plugging devices one at a time distinguishes the arm from the camera USB adapter. A USB cable must support data. Close any serial monitor before uploading or starting the bridge, because one program should own the serial port.

## 5. Upload and test communication without servo power

In the following commands, replace `YOUR_ARM_DEVICE` with the actual device name found by preflight. It is a placeholder, not a real port.

```bash
bash scripts/upload.sh serial_check /dev/serial/by-id/YOUR_ARM_DEVICE
```

The script compiles for the ESP32 Dev Module before uploading. If the bootloader cannot connect, check the board's BOOT/EN procedure and data cable. Then upload the I2C-only diagnostic:

```bash
bash scripts/upload.sh pca_diagnostics /dev/serial/by-id/YOUR_ARM_DEVICE
source hardware_env.sh
python -m serial.tools.miniterm /dev/serial/by-id/YOUR_ARM_DEVICE 115200
```

Configure the terminal to send LF line endings (miniterm's Ctrl+T menu offers EOL selection). Send `PING`, then `SCAN`. Expect a PONG and the addresses that acknowledge. The common PCA address is 0x40; an acknowledgement alone does not identify the chip. This diagnostic never writes servo PWM registers. Exit miniterm with Ctrl+] before another program opens the port.

You can also upload `arm_controller` using the same script. That default firmware is intentionally locked because the channels and calibration are unknown. Do not change its calibration flag merely to make it move.

## 6. Measure the arm and each actuator

Prepare a worksheet with one row each for base, shoulder, elbow, wrist pitch, wrist rotation and gripper. Record the actual driver channel, axis direction, safe joint-angle range, corresponding pulse widths, initial angle and conservative speed limit.

The joint angle is the angle used by the URDF, not an arbitrary servo label such as “90 degrees.” A reversed mounting may require decreasing pulse width as the model angle increases. Angles in the configuration are radians: degrees × π / 180. Distances are metres: millimetres / 1000.

Perform initial servo characterization one supported/unloaded joint at a time, using a suitable bench servo tester and the servo's documented signal range. Do not sweep an unknown assembled arm to generic 0/180-degree or 500/2500-microsecond endpoints. Establish a small verified range, inspect clearance and load, and expand only within the mechanical and electrical limits. Record conservative endpoints before hard stops. The supplied PCA diagnostic is deliberately not a sweep tool.

For the gripper, measure several pairs of servo angle and **inside jaw opening** using calipers. The prepared table interpolates these measurements rather than assuming a linkage ratio. The current model assumes two jaws moving symmetrically. If only one jaw moves, the jaw model and tool-centre offset must be changed first. A 20 mm box needs approach clearance and a tested closing command; width alone cannot determine grip force.

Measure the true tool contact point, actual ground plane relative to the robot base, box height and bin dimensions. “One side is 20 mm” does not establish that every physical box is a 20 mm cube.

## 7. Create one shared hardware profile

```bash
cp hardware/profile.template.json hardware/profile.json
nano hardware/profile.json
```

Keep the template unchanged. Replace null values with measurements. `joint_1` through `joint_5` correspond to base, shoulder, elbow, wrist pitch and wrist rotation; `gripper_joint` is the actuator angle. Record the stable USB path under `port`. Supply address is decimal 64 for 0x40, unless diagnostics establish a different address.

Each joint needs `channel`, `rad_low`, `rad_high`, `pulse_at_low_us`, `pulse_at_high_us`, `initial_rad` and `max_rad_per_sec`. The jaw table contains pairs `[servo_angle_radians, inside_opening_metres]`, ordered by increasing angle, with strictly monotonic opening. It must cover the configured gripper angle range. Do not copy numbers from unit-test fixtures: they are synthetic, not your calibration.

Only mark `calibration_confirmed`, `servo_power_verified` and `oe_wired_verified` true after doing those checks. Leave `model_verified` and `workspace_scene_verified` false until physical geometry and the actual MoveIt collision scene are correct. These flags record work; setting them does not perform that work.

Generate the matched files:

```bash
source hardware_env.sh
python hardware/prepare.py hardware/profile.json
```

Incomplete or inconsistent inputs are rejected. A valid profile produces `hardware/generated/arm_controller/`, `bridge.json`, `real_arm.urdf`, `real_arm.srdf`, joint limits and controller configuration. Firmware and bridge share a calibration fingerprint so an old firmware configuration cannot silently run with a new profile.

The generated URDF uses your joint limits but retains the provisional geometry. Replace or correct geometry, origins and collision shapes in the source Xacro, then regenerate. Mesh appearance alone does not establish correct joint axes, mass or inertia.

With servo V+ disconnected, upload the calibrated motor program:

```bash
bash scripts/upload.sh calibrated_arm /dev/serial/by-id/YOUR_ARM_DEVICE
```

Any calibration change requires regenerating and re-uploading the matched firmware. Keep the calibration file with your robot project.

## 8. Test the hardware interface in mock mode first

```bash
source hardware_env.sh
ros2 launch hardware/bringup.launch.py mock:=true
```

This needs your generated configuration but does not open the physical serial port. It starts the robot description, bridge, MoveIt and RViz. Check model angles, axes and gripper displacement against your measurements. The bridge converts the proxy hinge angle through the measured jaw-opening table into servo radians. It publishes both finger angle estimates in radians.

The simulation uses ROS domain 42; hardware commissioning uses domain 44. Source `hardware_env.sh` in every hardware terminal to keep them separate. Mock mode proves software interfaces, not motor performance.

Before physical launch, build and verify a collision scene containing the real ground, base obstacles, camera mount and bins. The hardware launch does not automatically import the simulation's scene or measure your room. A physical scene adapter and grasp-verification source still need integration. Do not set `workspace_scene_verified` true until that scene is actually present and checked.

Once those prerequisites are complete, the deliberate physical launch is:

```bash
source hardware_env.sh
ros2 launch hardware/bringup.launch.py mock:=false confirm_initial_pose:=true
```

The initial pose must match the supported physical arm before enabling power/outputs. Hobby servos can jump toward a startup pulse, and no encoder here can establish their initial position. Start with supervised small movements and an accessible physical power cut. The bridge rejects unverified profiles, mismatched firmware fingerprints and missing ports. Never run both a serial monitor and bridge simultaneously.

## 9. Prepare the camera separately

OV3660 identifies the sensor. USB-C/TTL describes a connection. Neither identifies which ESP32 or ESP32-S3 pins connect to the camera. Read the PCB model, obtain the seller's schematic or provide clear front/back board photos before selecting a camera profile.

The official example is in `firmware/CameraWebServer/`. After identifying the board:

1. Select its correct single `CAMERA_MODEL_...` definition in `board_config.h`, or supply the verified custom pin map.
2. Enter Wi-Fi credentials locally in `CameraWebServer.ino`.
3. Set `CAMERA_BOARD_CONFIRMED=true` only after verifying the mapping.
4. Use the board's actual FQBN and required memory options. Do not assume the generic ESP32 compile-test target applies to an S3 board.
5. Upload with `bash scripts/upload_camera.sh YOUR_CAMERA_PORT YOUR_VERIFIED_FQBN`.
6. Open its serial monitor at 115200, read its network address and check the JPEG `/capture` endpoint.

Source: [Espressif's ESP32 core 3.3.12 camera example](https://github.com/espressif/arduino-esp32/tree/3.3.12/libraries/ESP32/examples/Camera/CameraWebServer). The prepared example remains locked until identification is complete.

Publish camera images on the PC, replacing CAMERA_IP:

```bash
source hardware_env.sh
ros2 run arm_sorter camera --ros-args -p capture_url:=http://CAMERA_IP/capture
```

This publishes `/camera/image_raw`. The timestamp is the PC receive time, not a measured exposure timestamp.

## 10. Convert camera pixels into ground pick locations

Because the camera moves with the wrist, its view changes whenever the arm moves. Start with one calibrated observation pose: return there, stop, allow vibration to settle, capture a fresh image, then detect boxes. Do not reuse one image-to-world transform from a different wrist pose.

First calibrate camera intrinsics and distortion at the actual operating resolution. Then measure an image-to-base-plane mapping using points with known base-frame coordinates. For detections on box tops, calibrate the **box-top plane**, at the measured box height above ground. A floor homography applied to elevated box tops introduces position error. Validate at several independent points across the usable area and compare the error with your available gripper clearance.

A general moving-camera solution needs camera-to-tool calibration and sufficiently accurate joint poses; commanded hobby-servo estimates can be inaccurate under load. The fixed observation pose is a simpler initial method, but still requires validation.

For black/white boxes on a contrasting, matte ground surface, the existing controlled-background OpenCV detector is a useful starting point. It will not reliably separate touching boxes, shadows or matching-colour surroundings without additional work. YOLO requires suitable trained box segmentation weights; downloading a generic model does not train it to recognize your boxes or determine millimetres.

The ROS perception template is under `ros2_ws/src/arm_sorter/config/`. Its `calibrated` flag stays false until intrinsics, mapping, resolution and thresholds are measured. The perception node accepts a `config` file parameter and only emits detections while `/sorting/observe` is enabled by the coordinator. Do not use synthetic calibration to bypass this gate.

## 11. The real autonomous cycle and its remaining integration

The desired cycle is: move to observation pose → settle → detect all visible boxes → choose an isolated reachable target → classify black/white → calculate contact pose and opening → plan collision-free approach → descend → close → lift → verify a real grasp → move to the appropriate measured bin slot → release → verify placement → return to observe and repeat.

Objects sit directly on the ground. Their physical resting centre is ground height plus half their measured height; the tool contact height also depends on finger shape and tool offset. It is not the old pedestal height. Random placement must stay inside the measured workspace, without collisions or occlusions that the current detector cannot resolve.

`sort_cycle` contains the coordinator foundation, but **full physical autonomous sorting is not ready to run yet**. Its execution gates remain closed. It requires calibrated poses, real collision-scene integration, a genuine `/sorting/verify_pick` service using sensors or validated vision, and placement/bin-capacity handling. A service that always returns success would not verify a grasp. Simulation's attachment plugin must never be mistaken for real grip feedback.

Commission one target at a time: verify detection without motion; check the planned approach in RViz; perform a supervised approach with ground clearance; validate one gentle grasp and lift; validate one release into each bin; then add multiple well-separated targets. Reobserve after each pick because other objects or the camera may have moved. Define a stop condition for no targets, unreachable targets, failed grasps and full bins.

Do not run reinforcement-learning exploration on the real arm at this stage. Learn in simulation, validate the policy and enforce calibrated limits and collision checks before any later supervised transfer. Repeating the scripted sorter does not train an RL policy.

## 12. Reusing this project for a different arm

Keep separate geometry, actuator calibration, environment geometry and task settings. For another arm, update Xacro joint origins/axes and link meshes, limits and inertias; regenerate the URDF; update SRDF groups and controller names; calibrate its motor mapping; verify its tool and camera transforms; and rebuild the workspace. The demonstration's analytic IK contains this arm's dimensions, so changing only its appearance will not adapt that sorter. Use the detailed `BUILD_YOUR_OWN_ARM.md` tutorial alongside this guide.

Useful files: `hardware/prepare.py` generates configurations; `arm_sorter/bridge.py` handles trajectories; `jaw_mapping.py` converts gripper units; `firmware/arm_controller/` controls pulses; `camera.py` obtains images; `perception.py` locates boxes; `sort_cycle.py` coordinates physical-task stages. Start every new robot with mock tests, then independent bench checks, before combining the whole system.
