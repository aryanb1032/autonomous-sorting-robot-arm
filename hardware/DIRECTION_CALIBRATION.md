# No-box joint direction calibration

User confirmed the upright home pose at arm servo90/gripper80. Ground to base
rotation axis is 70 mm, giving modeled shoulder height 89 mm and upright tool
height 549 mm using the existing dimensions. Home offsets remain provisional
until matched to model coordinates. Positive direction signs are unmeasured.

Prepared firmware/direction_calibration identifies as DIRECTION_CALIBRATION_V1.
It preserves auto-start arm90 and gripper90-to80 then continuous home holding.
JOG accepts only arm PCA channels 0,1,2,5,6. Each command slowly moves 90→95→90
in ten seconds; other arm channels hold 90 and gripper holds80. Send KEEPALIVE
at intervals below 1.5 seconds during a jog. On heartbeat loss it performs a slow
return from the last commanded position to home; STOP disables all outputs.
Successful completion replies DONE HOME_HELD ESTIMATED and keeps holding.
I2C/register failures disable outputs; support the arm. No feedback is measured.

This is not unrestricted MoveIt execution. Do not upload with servo V+ on.
Only user-observed directions may be entered as mapping signs. Define world +X
as the intended front of the arm. For shoulder/elbow/wrist pitch, positive model
rotation tilts an upright link toward +X; base positive is counterclockwise viewed
from above. Wrist roll is around its local +Z axis, with orientation determined
from the gripper. Gripper aperture and full travel limits are still uncalibrated.

Current trial contains no boxes. The new base geometry revision invalidates old
sorting/RL validation as evidence for this model. Use the ordinary workcell launch
(sorting false) for this stage. No camera or pickup verification is required for
this limited motion trial, but those are required before physical sorting.
