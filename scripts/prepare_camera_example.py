"""Copy Espressif's installed camera example without guessing camera-board pins."""
from pathlib import Path
import shutil
root=Path(__file__).resolve().parents[1]
cores=root.parents[1]/'work/arduino-data/packages/esp32/hardware/esp32'
examples=list(cores.glob('*/libraries/ESP32/examples/Camera/CameraWebServer'))
if len(examples)!=1:raise SystemExit('Need exactly one installed ESP32 core; complete board installation first.')
destination=root/'firmware/CameraWebServer'
if destination.exists():raise SystemExit('CameraWebServer already exists; preserving edits.')
shutil.copytree(examples[0],destination)
print('Copied official example to',destination)
print('Select your actual camera board in the example configuration; set Wi-Fi locally. OV3660 identifies the sensor, not the board pinout.')
