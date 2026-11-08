# ESP32-CAM through an external USB/UART adapter

Prepared 29 October 2026 for the AI-Thinker-style wiring diagram supplied by the user. The subsequent front/back photos show the standard AI-Thinker-style ESP32-CAM layout. The AI-Thinker profile is selected and CAMERA_BOARD_CONFIRMED=true. Actual camera initialization and sensor detection still require a powered board test. ESP32-S is not synonymous with ESP32-S3.

## Connection, with power off

| Adapter / supply | Camera board |
|---|---|
| UART TX, 3.3 V logic | U0R / RX0 |
| UART RX | U0T / TX0 |
| GND | GND |
| Suitable regulated 5 V supply | Board pin marked 5V |
| Temporary jumper | GPIO0 to GND, for uploading only |

Read printed pin names, not just wire colours or picture orientation. The board's 5V input is power; its UART is 3.3 V logic. Verify the adapter's TX level independently of its VCC jumper. Do not send 5 V logic to ESP32 UART, and do not use the arm's 7 V servo supply. If the adapter cannot supply reliable camera power, use a suitable separate regulated 5 V supply and common ground; do not parallel two positive supply outputs.

## Upload sequence

1. The board profile is already selected and enabled from the supplied photos. No Wi-Fi credential edits are needed for initial hotspot mode.
2. Keep the motor ESP32 disconnected so its serial port cannot be confused with this camera.
3. With camera power off, connect GPIO0 to GND, then connect USB/power and press/release the camera RESET button if required. The diagram has no automatic reset wiring.
4. Identify the adapter using scripts/hardware_preflight.sh. If serial permission is denied, run `sudo usermod -aG dialout "$USER"` yourself and log out/in. Do not change device permissions to 777.
5. From the robot-arm project directory, run the following with the actual adapter path:

```bash
bash scripts/upload_camera.sh /dev/serial/by-id/YOUR_CAMERA_ADAPTER esp32:esp32:esp32cam
```

6. After a successful upload, remove the GPIO0–GND jumper and reset the camera. Leaving it connected makes the chip start the downloader instead of the camera program.
7. Connect laptop Wi-Fi to **RobotArm-Camera**, password configured in your local copy of the sketch. Replace the placeholder before uploading; do not put a real password in Git. Your laptop may report no Internet, which is expected.
8. Open `http://192.168.4.1/` for controls or `http://192.168.4.1/capture` for a JPEG. The serial monitor at 115200 prints the actual address. Initial image resolution is 640×480; preserve that resolution during calibration.
9. To publish snapshots into ROS:

```bash
source hardware_env.sh
ros2 run arm_sorter camera --ros-args -p capture_url:=http://192.168.4.1/capture
```

USB/UART is used for flashing and logs; normal camera images travel over Wi-Fi. Keep the flash LED off initially to avoid glare on the boxes. Fix exposure/lighting and calibrate the vision system before interpreting image pixels as millimetres.

Nothing uploads automatically on USB insertion. Tell the assistant when the camera is connected and ready in download mode; upload authorization has already been given. A live serial port and download-mode wiring are still required. No board was connected during preparation.

Official references:
- https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html
- https://github.com/espressif/arduino-esp32/tree/3.3.12/libraries/ESP32/examples/Camera/CameraWebServer
