> Current model revision: **2026-10-29-final-dimensions**. See [final geometry notes](model/FINAL_GEOMETRY.md). Previous validation reports and RL checkpoints apply only to their original geometry.

# Arm hardware and calibration record

Fill unknown entries before generating motor firmware or an accurate robot model.

- ESP32 controller board: ESP32 Dev Module (exact module/board markings pending)
- Camera board: standard AI-Thinker-style ESP32-CAM with ESP-32S module, OV3660 ribbon marking confirmed in photos. Firmware prepared; upload has not succeeded yet.
- Camera mount: moves with gripper (eye-in-hand)
- Arm kit/model: custom printed arm; similar reference https://grabcad.com/library/6-dof-robot-arm-v1-1 (not verified; page returned HTTP 403)
- Number of arm joints, excluding gripper: apparently five, pending confirmation; six servos total reported, one used for gripper
- Servo inventory: three DEAO 20 kg servos and three MG996R; one MG996R drives hinged-linkage gripper
- Reported dimensions: ground-to-M1 35 mm (retained assumption); M1–M2 19 mm; M2–M3 120 mm; M3–M4 110 mm; M4–M5 60 mm; M5–gripper motor 100 mm; gripper motor–tip 70 mm. Confirm joint-axis-to-joint-axis distances and axis directions before URDF generation.
- Servo driver: user wrote PCA9865, presumed PCA9685 pending board marking confirmation; VCC 3.3 V, common GND, SDA GPIO21, SCL GPIO22. Channels pending.
- Servo supply voltage and current rating: reported 7 V, 7.32 A. Do not energize MG996R servos at 7 V without an exact applicable manufacturer rating; Tower Pro lists 4.8–6.6 V. Confirm regulated suitable servo supply before testing.
- Box height, width range and mass:
- Maximum jaw opening: pending; hinged-linkage mechanism

| Joint, base to tool | Servo model | GPIO or driver channel | Link dimensions | Rotation axis | Safe angle range | Zero offset | Direction |
|---|---|---|---|---|---|---|---|
| Base | DEAO 20 kg-class | pending | M1–M2 19 mm | provisional | pending | pending | pending |
| Shoulder | DEAO 20 kg-class | pending | M2–M3 120 mm | provisional | pending | pending | pending |
| Elbow | DEAO 20 kg-class | pending | M3–M4 110 mm | provisional | pending | pending | pending |
| Wrist 1 | MG996R | pending | M4–M5 60 mm | provisional | pending | pending | pending |
| Wrist 2 | MG996R | pending | M5–gripper motor 100 mm | provisional | pending | pending | pending |
| Gripper | MG996R | pending | Motor–tip 70 mm; pictured hinged linkage | pending | pending | pending | pending |

Do not infer mechanical limits from the servo's advertised electrical range.
Measure jaw opening at several safe servo positions. Keep commanded joint state
distinct from measured feedback if the servos expose no position telemetry.

## Moving camera strategy

Begin with a repeatable observation pose and capture only after the arm settles.
Calibrate pixel-to-workspace mapping at the known box-top plane in that pose.
Use this mapping only at that pose; reobserve from it after each pick.
This shortcut depends on mechanical repeatability and must be checked against
measured targets across the workspace. General vision from moving poses needs
camera-to-tool calibration and reliable base-to-tool transforms at image time.

- Recommended output-disable wiring for prepared firmware: GPIO23 to PCA9685 OE, with a verified pull-up from OE to 3.3 V that overcomes the board pull-down (see docs/REAL_ROBOT_SETUP.md). Not yet confirmed wired.
- Channel numbers will be provided later by the user; no assignment has been invented.
- Real boxes are on the ground, not pedestals. The simulation now uses ground pickups.

- Camera update 2026-10-29: user front/back photos match standard AI-Thinker-style ESP32-CAM with ESP-32S module. Selected esp32:esp32:esp32cam and enabled camera firmware. Sensor identity and UART/power wiring still require physical startup verification.

## Motor trial update — 29 October

User confirmed six 180-degree positional servos, all at a 90-degree startup pose. Channels base/shoulder/elbow/wrist pitch/wrist rotation/gripper: 0/1/2/5/6/7. Reported pulse endpoints: 500–2500 us for the five arm servos; 1000–2300 us for gripper. Nominal linear midpoint pulses are 1500 us for arm and 1650 us for gripper. These do not establish URDF zero offsets, signs or mechanically safe full travel. Supply is STILL 7 V; OE is NOT wired. Physical movement remains blocked.

- Latest power/OE update: user reports servo rail measured at 6.5 V with a multimeter and GPIO23 connected to PCA9685 OE. Trial profile updated; no motor output has yet been enabled. External OE reset pull-up has not been independently checked.
