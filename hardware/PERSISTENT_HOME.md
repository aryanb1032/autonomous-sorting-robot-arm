# Persistent home: arm 90 degrees, gripper 80 degrees

Dedicated firmware: firmware/persistent_home/persistent_home.ino.
User requested continuous holding and automatic home on power-up.
It has no sorting or trajectory command support. It does not time out or require
a PC heartbeat. Every ESP32 boot initializes the PCA9685 and enables home pulses.
If ESP32 stays powered while servo V+ is switched off/on, the same pulses remain
commanded and the servos can move immediately when V+ returns.

Channels 0,1,2,5,6 use 1500 us (nominal 90 degrees). The updated channel 7
range is 1200–2600 us across 0–180 degrees. Each HOME command or ESP32 boot
starts the gripper at 90 degrees (1900 us), ramps over five seconds to 80 degrees
(1822 us after integer truncation), and holds there continuously. The 2600 us
endpoint is only a mapping parameter; this firmware never commands that endpoint.
These are command estimates, not measured angles or URDF joint zero. The old
1650 us home and Windows calibrator 1500 us center are superseded for this mode.
The initial step to 1900 us is not a measured-position ramp: support the arm.

CONFIG ID: PERSISTENT_HOME_GRIP80_V2. HOME is preferred; HOME90 remains a legacy
alias that also performs arm90/gripper90-to80. STATE reports gripper command,
pulse, homing and enable state. No servo-rail sensing exists: switching only
servo V+ does NOT restart the sequence. If the ESP32 has already completed the
ramp with V+ off, applying V+ moves toward 80 degrees directly. To observe the
90-to80 sequence after applying V+, explicitly send HOME with the arm supported.

All other channels stay FULL_OFF. Registers are checked before OE goes LOW and
once per second while holding. Failed checks latch outputs disabled; no automatic
retry. STOP disables outputs until HOME90 or ESP32 reboot. INFO and STATE are
read-only commands. Power must remain off during upload and initial checks.
Automatic startup enabling is intentional: support the entire arm before servo
power is applied, and never adjust horns or screws while servos are powered.
OE may float during reset before setup; a hardware disable arrangement is still
needed for guaranteed reset-time behavior.

Power loss cannot initiate homing, and holding torque disappears without servo
power. A displaced arm can fall. Future trajectory firmware must explicitly
perform a collision-checked return home before an orderly shutdown; this firmware
does not add that feature to Gazebo, MoveIt or a hardware bridge.
