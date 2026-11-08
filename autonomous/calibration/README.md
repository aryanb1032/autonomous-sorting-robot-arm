# Fixed-phone calibration status

The stream at PHONE_IP:4747/video works at 1280 x 720. The observed view
contains the pickup rectangle but crops the bins and base. It contains no
checkerboard. Repeated frames of this scene do not determine a full lens model.

The provisional floor mapping is a diagnostic only: no lens correction, no
20 mm cube-top compensation, no robot-frame registration, and no independent
accuracy check. It is not installed as an active robot calibration.

## Automatic checkerboard collection

Print checkerboard_25mm.svg at actual size. Measure the squares: 25 mm. Mount the
print flat on rigid backing. Keep the phone lens/zoom/resolution fixed. Move the
BOARD through different positions, distances and tilts, covering the image centre
and edges. Keep all 7 x 5 internal corners visible. Do not show the pattern on a
screen as a substitute for a measured flat physical print.

Run using the project .venv Python:

    python autonomous/calibration/capture_checkerboard.py

It automatically saves 15 sufficiently different checkerboard views and fits lens
intrinsics using the existing calibration utility. It rejects near-duplicate
images and stops after 180 seconds (adjust --seconds if needed). Q exits. The
result is UNVALIDATED; pose diversity, error and independent accuracy still need
checking. It sends no motor commands and does not activate a calibration.

After lens calibration: register the robot base origin/heading and workspace,
map the 20 mm top plane, locate bins, and verify held-out metric points. The
current cropped view cannot visually verify placement inside both bins. Choose
a final view that includes those regions before completing registration.
