# Windows sketch comparison

The user-supplied sketch is preserved as windows_servo_calibrator.ino.
It uses PCA address 0x40, SDA21/SCL22, channels 0,1,2,5,6,7 and 50 Hz,
matching our configuration. It uses Adafruit_PWMServoDriver::writeMicroseconds.
It does not drive GPIO23/OE. All six channels start at 1500 us.
Its browser SAVE buttons update DOM text only; no persistent calibration exists.
It permits 500–2500 us on channel 7, beyond the user's specified 1000–2300 range.
Our gripper midpoint was 1650 us from that specified range; the Windows sketch
uses 1500 us. Neither value establishes the physical jaw calibration.

Prepared firmware/single_channel_adafruit uses Adafruit PWM Servo Driver 3.0.3
and BusIO 1.17.4 installed locally. The version originally used on Windows is
unknown. No startup motor commands: OE stays HIGH until explicit RUN, and all
unselected channels stay FULL_OFF. Same 90→92→90 small trial, watchdog and final
output disable as the direct-register diagnostic. The selected pulse registers
are read back after every update. No browser/network interface is introduced.
Use scripts/single_channel_trial.py with --driver adafruit after flashing this
specific firmware. Its CONFIG ID is SINGLE_CHANNEL_ADAFRUIT_V1.

This comparison has not identified the remaining no-motion root cause. Register
readback cannot verify the actual electrical waveform at the servo connector.
Compile passes are not physical validation. Upload only with servo V+ OFF.
