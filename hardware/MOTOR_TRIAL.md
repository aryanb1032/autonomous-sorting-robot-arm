# Real motor trial, before simulation-to-hardware synchronization

This moves real actuators, even if a simulated arm is displayed alongside them. Camera and vision are not required.

Confirmed user mapping:

| Role | PCA9685 channel | Servo | Reported pulse range | Nominal 90° pulse |
|---|---:|---|---|---:|
| Base | 0 | DEAO 20 kg-class | 500–2500 µs | 1500 µs |
| Shoulder | 1 | DEAO 20 kg-class | 500–2500 µs | 1500 µs |
| Elbow | 2 | DEAO 20 kg-class | 500–2500 µs | 1500 µs |
| Wrist pitch | 5 | MG996R | 500–2500 µs | 1500 µs |
| Wrist rotation | 6 | MG996R | 500–2500 µs | 1500 µs |
| Gripper | 7 | MG996R | 1000–2300 µs | 1650 µs |

All are reported 180° positional servos. Values above assume a linear 0–180° mapping across the reported pulse endpoints. This is not measured calibration. A 90° library command with a different configured range may have produced a different pulse, especially for the gripper. Verify the supported starting posture against these pulse values.

## Initial blockers (subsequently updated)

The user reports the servo supply is still 7 V and GPIO23/OE is not wired. The trial is generated LOCKED. No motor command or upload has been performed. MG996R manufacturer rating is 4.8–6.6 V: https://towerpro.com.tw/product/mg996R/ . Use a correctly rated regulated rail and verify the exact DEAO model too. Do not power the MG996Rs directly from 7 V.

Wire GPIO23 to PCA9685 OE and ensure a verified pull-up holds OE HIGH during reset. Board pull-down resistors matter; see ../docs/REAL_ROBOT_SETUP.md. SDA21, SCL22, 3.3 V VCC and common GND remain as before. Servo V+ is a separate suitable power rail. Keep servo V+ off while uploading.

90° is a chosen startup command, not sensor-based homing. The controller cannot determine actual position. Support and position the arm before enabling it. Initialization commands ALL SIX servos to their nominal 90° pulses; afterwards only the selected joint moves. Support the arm before cutting power, because gravity-loaded joints can fall.

## Prepared files

- motor_trial.json: confirmed channel/angle/pulse information, with unverified power/OE flags false.
- prepare_motor_trial.py: produces narrow-range trial firmware under motor_trial/arm_controller. Only 88–92 servo degrees are accepted, at at most 2°/s.
- ../scripts/motor_trial.py: defaults to a preview with no serial access. Executable trial moves one selected motor 90→92→90, with watchdog keepalives and calibration-fingerprint verification.
- profile.template.json: channels and reported servo facts populated. Actual robot-angle limits, zero positions, directions and measured jaw-opening calibration remain null.

From the robot-arm project directory, preview without any motor connection:

```bash
python3 scripts/motor_trial.py --channel 0
python3 scripts/motor_trial.py --channel 7
```

Once the suitable supply is measured and OE actually wired, update only the truthful power/OE fields in hardware/motor_trial.json. Generate enabled trial code with `python3 hardware/prepare_motor_trial.py --enable`. That command rejects the current 7 V/unwired profile. Upload using `bash scripts/upload.sh motor_trial ACTUAL_ESP32_PORT` with servo V+ off.

For a supervised trial after all checks, `source hardware_env.sh`, then run `python scripts/motor_trial.py --channel 0 --port ACTUAL_ESP32_PORT --execute --confirm-supported-home`. The port placeholder must be replaced. The --execute flag opens the port, initializes all six motors and moves the selected one. Begin with one motor and observe direction/clearance; do not run a loop across all joints unattended. STOP holds the last command and is not an emergency power cut.

## Why the Gazebo sorter cannot simply be streamed yet

The model uses joint radians and mechanical reference frames. A servo's 90° does not tell us the model joint angle at that posture or the sign of rotation. Record each model zero offset and direction from the actual arm, along with safe mechanical limits. Full advertised 0–180° travel may hit the assembled arm. The new gripper additionally needs measured servo-angle/jaw-opening pairs.

After measuring those values, generate the normal hardware profile and validate the real collision scene. MoveIt can then send a trajectory through the real bridge while RViz displays estimated joint states. Those are command estimates, not encoder feedback. Gazebo's simulated box attachments and scripted successful sorting are not physical grasp verification. Do not connect the full autonomous sorter or RL exploration directly to this commissioning firmware.

## Latest readiness update

User subsequently confirmed a multimeter-measured 6.5 V servo rail and the GPIO23/OE connection. The current trial profile records these confirmations. Trial firmware may now be generated enabled, but outputs remain disabled at startup until an explicit INIT command. Keep servo V+ off during upload and support all six joints at their 90-degree starting pose before enabling output. The external OE pull-up during reset still needs physical verification.
