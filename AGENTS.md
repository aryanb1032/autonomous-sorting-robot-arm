# Robot startup contract — user requirement, 2026-11-02

Preserve nominal 90-degree startup and successful-task home for ALL SIX servos,
including channel 7 gripper. The earlier gripper home of 80 degrees is superseded.
Production motion firmware is firmware/live_trial and firmware/virtual_pick.
Both must command home automatically at MCU boot after successful configuration
and I2C setup. A persistent STOP latch is an exception; never bypass STOP through
boot, INIT or an automatic task. HOME90 is an explicit operator recovery command.

Before changing firmware or uploading, run tests/check_home90.py, compile the
selected sketch, and check matching host identity, initial angle and gripper range.
Do not upload legacy persistent_home, one_pass_direction or generated calibration
sketches as production firmware: they do not implement this current contract.
Historical commissioning logs must not be rewritten as newly measured evidence.

Keep the limitations visible: startup is a position command, not sensor-based
homing; current physical angles cannot be read; there is no servo-power rail sensor.
Cycling only servo V+ after interrupted motion does not cause MCU startup.
Do not upload/reset the controller with servo power on. No motor action is implied
by code preparation or compilation. Require the user's current hardware readiness.

# TPU gripper grasp policy — user requirement, 2026-11-02
Every physical grasp commands the existing full-close endpoint, servo 0 degrees
(channel 7, nominal 1200 us with the existing mapping), and retains that command
during lift/transfer until release. Release/startup/successful-task home stays
90 degrees. Do not replace physical full closure with a size-based partial close.
Gazebo's rigid finger clearance is separate from this physical servo target; it
does not model TPU deformation or guarantee grip force. Preserve STOP handling.
