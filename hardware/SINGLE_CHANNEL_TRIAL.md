# Isolated channel diagnosis

Use this instead of motor_trial while identifying unexpected joint movements.
The old trial initializes all six motors; this diagnostic only writes a pulse to
one selected channel. Other channels retain PCA9685 FULL_OFF. GPIO23 drives OE
HIGH until an explicit RUN command. Startup never commands a servo position.

Keep servo V+ off while uploading. Wire SDA21, SCL22, logic VCC3.3V, common ground,
and GPIO23 to OE. The verified rail was 6.5V. Support the ENTIRE arm at its
nominal 90-degree pose: unselected motors have no commanded holding torque.
The selected motor may move when its nominal 90-degree pulse is first applied.
A reset may float OE before setup runs; keep V+ off during reset/upload.

Upload with scripts/upload.sh single_channel_trial PORT.
Preview with python3 scripts/single_channel_trial.py --channel 0.
After the operator explicitly enables servo power, execute with:
python3 scripts/single_channel_trial.py --channel 0 --port PORT --execute --confirm-supported-home

RUN enables only the selected channel at nominal 90 degrees, holds one second,
ramps to 92 over two seconds and returns over two seconds. After a short hold,
ALL outputs disable. STOP, I2C error, or a 1.5-second missing heartbeat also
disables outputs. Keep mechanical support in place throughout.

Channels allowed: 0,1,2,5,6,7. Mapping is still being diagnosed; do not infer it
from initialization movements. Channel 7 uses 1000–2300 us; others 500–2500 us.
PWM uses a nominal 50Hz PCA9685 oscillator setting; actual pulse widths and
physical angles have not been measured. Responses report command completion,
not encoder feedback. This is not simulation mirroring or autonomous control.

## 2026-10-30 correction and hardware register validation

V1 contained an incorrect write of zero to ALL_LED_OFF_H (0xFD) after selecting
one channel. This register broadcasts into all individual LEDn_OFF_H registers;
it is not a separate global-enable latch. The write cleared every FULL_OFF flag
and corrupted the upper PWM count bits. This can explain unexpected movement
of unselected motors. Do not use V1 or old compiled firmware binaries.
V2 removes that write and verifies all 16 channel registers before lowering OE.
The normal controller source and generated motor-trial source were also corrected.
No channel remapping was made: user reports base=0 and gripper=7.

With servo power OFF, V2 was uploaded at 115200 baud after an interrupted
higher-speed upload. CHECK reproduced the old broadcast effect with OE HIGH:
channel7_offH=0. Then both channel 0 and channel 7 configurations passed complete
register readback: selected channel had its expected pulse count, every other
channel had FULL_OFF set. Final STATE ready=1 selected=-1 oe=1.
Raw results: work/single-channel-register-result.txt relative to task workspace.
The check did not enable outputs or validate physical motor movement.
NXP datasheet section 7.3.4: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
