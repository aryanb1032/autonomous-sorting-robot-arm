> Fixed Redmi phone setup: see PHONE_CAMERA.md. Current objects are confirmed 20 mm cubes. Use --stream for MJPEG/RTSP video; --url is for individual JPEG snapshots.

# OpenCV: black/white boxes with a 2 cm side

One reported box side is 20 mm. This software measures both visible planar sides
and checks whether either is close to 20 mm. It does not assume square boxes,
20 mm height, two boxes per colour, or a particular box count.

## Files and operation

- box_vision.py: reads images, ESP32 JPEG snapshots, or a USB camera; identifies
  black/white quadrilaterals; labels centres and widths/lengths; writes JSON.
- settings.json: thresholds, region of interest, expected side and tolerance.
- calibrate.py: camera intrinsics and planar coordinate calibration.
- collect_plane_points.py: click reference pixels and enter measured coordinates.
- tests/: automated synthetic checks; real-camera accuracy remains untested.

The pipeline is image -> optional lens correction -> colour threshold masks ->
contours -> shape filtering -> optional plane mapping -> rotated dimensions.
It uses OpenCV, not YOLO, and does not issue motor commands.

## Prepare the scene

Use separated, matte boxes on a uniform mid-grey surface. Illuminate evenly;
exclude the gripper, bins, cables and background clutter with the ROI. Keep
all measured top faces at the same known height. Return the wrist-mounted camera
to its calibrated observation pose and let it settle before measuring.

Only the planar top face should define the contour. A tilted camera showing
large side faces can make the silhouette larger than the top face; this simple
threshold detector cannot distinguish those surfaces. Arrange a near top-down
view, verify the selected contour, and reject unsuitable views.

## Run on your ESP32 camera

```bash
source /path/to/robot-arm-project/activate.sh
cd "$ROBOT_PROJECT/opencv_boxes"
python box_vision.py --url http://CAMERA_IP/capture --settings settings.json --output results
```

Replace CAMERA_IP with the actual camera address. Its configured firmware must
provide a JPEG /capture endpoint. Press q to quit, or use --headless and Ctrl+C.
Without calibration this deliberately reports pixels, not millimetres.

For a saved photograph:

```bash
python box_vision.py --image /ABSOLUTE/PATH/photo.jpg --settings settings.json --headless --output results
```

For a USB camera, replace --image/--url with --camera 0 (use the correct index).
The camera source options are mutually exclusive.

## Calibrate the lens

Photograph a flat checkerboard from at least 10–20 different positions and tilts
covering the image at the exact intended resolution. The board's square size
must be measured. cols/rows are INNER corner counts, not square counts.

```bash
python calibrate.py intrinsics --images 'calibration_photos/*.jpg' \
  --cols 9 --rows 6 --square-mm 15 --output intrinsics.json
```

9, 6 and 15 are examples; replace them with your actual board dimensions.
Inspect reprojection error and coverage; merely producing a file does not prove
accurate calibration. Do not change lens focus or image resolution afterward.

## Calibrate the box-top measurement plane

Place a ruler/grid/reference board at the same height as box TOPS, with the arm
at its fixed observation pose. Use at least six noncollinear points spread across
the pickup area. Measure each point's XY coordinates in millimetres relative to
one consistent reference origin. Save a raw snapshot, then:

```bash
python collect_plane_points.py reference.jpg points.json
python calibrate.py plane --intrinsics intrinsics.json --points points.json --output calibration.json
```

Click the reference points in the image, press Enter, and type each measured X Y
in the terminal. The calibration script corrects those raw pixels for distortion.

Check the resulting measurement using additional reference lengths/positions
NOT used for calibration. After acceptable independent checks, set
"validated": true in calibration.json and record the plane height and observation
pose in its description. Then:

```bash
python box_vision.py --url http://CAMERA_IP/capture --settings settings.json \
  --calibration calibration.json --output results
```

The calibration's plane coordinates are NOT automatically robot-base coordinates.
If you want robot-base coordinates, define the reference XY in that frame and
convert millimetres to metres for ROS. This standalone program never publishes
robot targets. Do not use its mm homography directly in the earlier ROS config,
which expects metres; convert the first two matrix rows by dividing by 1000.

## Reading the output

results/latest.jpg: labelled image. results/latest.json: counts and object list.
Each object contains colour, per-frame label, centre, pixel dimensions, and
optional calibrated width_mm, length_mm and planar jaw-closing angle.
Width means the SHORTER side of the fitted rectangle; length means the longer.
Labels such as black_1 are re-assigned each frame and are not persistent tracks.

matches_20mm_side is true if either measured side falls within the configured
tolerance of expected_side_mm=20. It is null without calibration. The default
2 mm tolerance is a starting setting, not an accuracy guarantee. Change it to
match your measured vision error and box manufacturing tolerances.

Do not use the known 20 mm box both to define the scale and to claim independent
verification that the same box measures 20 mm. Use separate references.

## Tune detection

- black_max: maximum HSV brightness for black. Raise only if dark boxes disappear;
  increasing it too far includes shadows/background.
- white_min: minimum brightness for white. Lower only if white boxes disappear.
- white_max_saturation: excludes bright coloured objects from the white mask.
- roi: null for full frame, or [x,y,width,height] in UNDISTORTED image pixels.
- min_area_px: reject specks. max_area_fraction: reject very large regions.

Inspect black_mask.png and white_mask.png in the output directory. Correct masks
should isolate each complete top face. Touching boxes may merge; shadows, glare
and textured faces can change the outline. This controlled-scene algorithm does
not solve arbitrary clutter or recover 3D height from one image.

## Included demonstration

examples/synthetic_input.png contains two black and two white rectangles solely
to exercise multiple detections. Their simulated sizes are 20 x 30 mm.
examples/SYNTHETIC_ONLY_calibration.json is mathematically invented for that
image and MUST NOT be used with the real camera.

```bash
python box_vision.py --image examples/synthetic_input.png \
  --calibration examples/SYNTHETIC_ONLY_calibration.json \
  --headless --output examples/demo_results
```

Reference for planar transforms:
https://github.com/opencv/opencv/blob/4.x/doc/tutorials/features2d/homography/homography.markdown
