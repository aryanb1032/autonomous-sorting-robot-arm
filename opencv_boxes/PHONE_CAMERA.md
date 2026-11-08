# Redmi Note 13 — fixed camera, vision only

PC dependencies are installed. Use DroidCam on the Redmi Note 13; the supplied
address is http://PHONE_IP:4747/video. IP Webcam failed to open this phone's
camera, so it is not the selected source. Start DroidCam's server and keep phone
and PC on the same local network. The address may change after reconnecting.

Check that the app can select the intended rear 0.6x camera. Availability is
phone/app dependent; choosing 0.6x in Xiaomi's Camera app does not configure this
streaming app. Use fixed landscape orientation and a fixed resolution (start at
1280x720, increase to 1920x1080 if supported and stable). Avoid switching lenses,
zoom, cropping or resolution after calibration. No depth channel is assumed.

Run:

    bash /path/to/robot-arm-project/opencv_boxes/run_phone_camera.sh http://PHONE_IP:4747/video

Update the address if DroidCam reports a different IP. Press q in the camera window to
quit. The program has no serial interface and sends no robot motion commands.
A stream failure clears detections and exits rather than continuing with stale
results. Open/read timeouts are bounded through OpenCV's FFmpeg backend.

Outputs are in the workspace's work/phone-camera-live directory:
- latest_raw.jpg: last received unannotated frame, useful for calibration.
- latest.jpg: detector overlay.
- latest.json: colour counts, candidate contours, pixels and optional calibration.
- black_mask.png / white_mask.png: threshold diagnostics.

A PC processing timestamp is not a camera exposure timestamp. Buffering/network
latency have not been measured. This stream must not yet drive real grasp targets.
The current detector is threshold/contour based, not YOLO; it can confuse bins,
shadows and touching cubes. Tune the pickup ROI and lighting on the live view.

Known scene: fixed phone, reported 530 mm horizontal offset and 470 mm height,
20 mm cubes, fixed black/white bins. These measurements are context, not validated
camera extrinsics. Final calibration must use the actual lens/stream resolution,
lens-distortion correction and a plane at cube-top height, 20 mm above the floor.
Use independently measured reference points in robot-base coordinates. Do not
reuse a provisional calibration after moving the phone/work surface.

RL preparation: the existing Gazebo reaching environment has been corrected to
M4 roll / M5 pitch. Its policy observes simulated joint states and target geometry;
it does NOT consume this phone stream and does NOT learn pickup/grasp/sorting yet.
Old checkpoints are rejected by geometry/task revision checks. Real robot learning
is not enabled. First validate vision, then build/validate a simulated sorting
reward and observation pipeline before considering physical policy execution.
