# Keeping the robot PC ready

1. Keep the PC plugged into power and connected to the internet during setup.
   In Ubuntu Settings > Power, disable automatic suspend temporarily while
   downloading or operating the robot. Restore your preferred setting later.
2. Run `bash setup_pc.sh` in this folder. Enter the sudo password in the terminal.
   The script installs system dependencies and adds your account to dialout.
   Log out and back in afterward for serial permissions to take effect.
3. In each new bash terminal, source this project's activate.sh using its full
   path. This selects ROS Jazzy and the isolated vision Python environment.
4. Run `bash check_pc.sh`. Missing-package output means setup is incomplete.
5. Connect the ESP32 Dev Module using a data-capable USB cable. Keep the
   servo power supply off during the first USB communication check.
6. Close other serial monitors before a ROS process opens the same port.
7. Keep the camera and PC on a network that lets devices communicate with
   each other. Record the camera stream URL once camera firmware is configured.
8. Keep the arm base secured and the working area clear. The servo supply,
   driver model, channels, voltage and current rating must be confirmed before
   motion testing. Never power all six servos from the ESP32 USB/5V pin.
9. Keep the outputs/robot-arm and work folders in place: setup uses local paths.
   Save source/configuration changes; avoid package upgrades during final trials.

No robot firmware has been flashed and no servo movement has been commanded.
The serial_check sketch is only for a later communication test.
YOLO's pretrained weights are a starting point, not a trained box-colour model.
