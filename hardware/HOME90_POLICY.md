# All-six-servo home policy — 2026-11-02

User requested 90 degrees for every servo at startup, across future project changes.
This supersedes gripper home 80. Channels remain 0,1,2,5,6,7. Arm home PWM is
1500 microseconds; gripper home PWM is 1900 microseconds under the existing
1200–2600 microsecond / 0–180 degree mapping. Physical angles remain estimates.

Prepared firmware identities: LIVE_TRIAL_HOME90_V2 and VIRTUAL_PICK_HOME90_V2.
Both enable home automatically after successful MCU startup and configuration.
They continue holding home without a PC until a motion or STOP command.
INIT now verifies home and readiness; it does not enable a stopped arm.
The host pickup opens/returns the empty gripper to 90, closes to 0.

STOP disables OE and persists a latch in ESP32 storage. Reboot and INIT cannot
clear it. With the arm supported and path clear, explicit HOME90 clears STOP and
commands all six home. It may move immediately because physical position is unknown.
I2C/configuration/storage faults leave outputs disabled rather than forcing home.

There is no servo V+ sensing. If USB remains powered and servo V+ is cycled after
an interrupted move, the controller cannot know it should change its command.
For startup home, power the ESP32 from a fresh boot as well, or explicitly HOME90.
Nothing can physically return home after power is removed.

Future edits are governed by AGENTS.md and checked with tests/check_home90.py.
Legacy sketches outside the two production variants are historical and must not
be uploaded expecting this behavior. Upload only with servo V+ OFF and arm supported.
These changes are prepared and compiled; uploading is a separate step.
