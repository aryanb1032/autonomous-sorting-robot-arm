# Autonomous sorting preparation

This folder runs autonomous **simulation** without chat input. It does not open a
serial port, upload firmware, or move the physical arm. Camera observation tools
are prepared separately. A complete real-camera-to-physical-arm sorting controller
is NOT yet implemented or validated; changing a JSON flag does not enable one.

## One command from Linux login

1. Log into the Ubuntu graphical desktop.
2. Open Terminal with Ctrl+Alt+T. Close an existing project simulation first.
3. Run:

```bash
bash /path/to/robot-arm-project/autonomous/start.sh --learn
```

The command sources the installed ROS 2 Jazzy/workspace environment, creates three
black and three white 20 mm cubes in seeded random positions on the ground, starts
Gazebo, MoveIt 2, RViz, and a status/STOP window. It then selects, approaches,
grasps, lifts, transfers, releases and verifies each cube in its colour bin.
Successful completion returns the simulated arm to upright home with gripper open.
Gazebo and RViz remain open afterward. Ctrl+C in the terminal closes this run.
No additional terminal sourcing or assistant messages are needed.

For a shorter two-cube run:

```bash
bash /path/to/robot-arm-project/autonomous/start.sh --per-color 1 --learn
```

For five independent episodes, with different random placements and a saved policy:

```bash
bash /path/to/robot-arm-project/autonomous/start.sh --episodes 5 --learn --exit-after
```

Press **O while the status window has keyboard focus**, or click STOP, to cancel.
You can also run `bash autonomous/stop.sh` from the project folder.
It is not a global desktop hotkey and this window does not stop a separately running
physical controller. STOP aborts the current task; it does not automatically move
home through an uncertain scene. A new manual launch clears this simulation STOP.

## What autonomy and learning mean here

- `launch.py` owns startup, each episode, process cleanup and run logs.
- `run_sorter.py` owns the complete sorting state machine. It checks object pose,
  reachability, pickup alignment, lift, and stable placement inside the bin.
- MoveIt plans motion and sampled joint paths are collision checked.
- One planner rejection before grasp can be retried. Communication failures,
  uncertain grasp/drop, or STOP terminate the episode, without blind motion retries.
- Gazebo supplies object identities, colours and positions. This is ground truth,
  NOT machine vision, and must not be presented as a camera detection result.
- Grasp uses a Gazebo detachable joint. The finger clearance accommodates rotated
  cubes; this is NOT a validated friction, contact-force or real aperture model.
- `policy.py` learns the choice between nearest/black-first/white-first selection
  with tabular Q-learning. Verified placement earns reward, elapsed time reduces
  it, and failed attempts receive a penalty. It saves after every attempted cube.
- This policy learns sorting order, NOT servo angles, vision or grasping. There is
  no claim that a few trials produce a better policy. The separate reaching RL
  environment also does not constitute learned end-to-end sorting.

Run evidence is in `autonomous/runs/<timestamp>/episode-NNN/`: controller/Gazebo
logs, status, events, and (on success) result.json. Learned values are in
`autonomous/models/order.json`. Failed attempts remain recorded.

## Phone vision preparation

The saved stream is `http://PHONE_IP:4747/video`. Start the DroidCam server on
the phone, then run:

```bash
bash /path/to/robot-arm-project/autonomous/start_camera.sh
```

This opens continuous black/white contour detection, saves images and latest.json,
and exits on stream failure. Press q in the image window to close. It does not
control motors. Edit camera_config.json if the phone's IP changes. Use the same
lens, orientation, resolution and mounting position for calibration and operation.
Selecting 0.6x in the phone's normal Camera app does not establish which lens the
streaming app uses. No depth stream is assumed.

Existing `opencv_boxes/calibrate.py` supports checkerboard lens calibration and
pixel-to-plane mapping; `collect_plane_points.py` collects correspondence points.
Run either with --help for its arguments. Calibration requires at least ten varied
checkerboard images and six measured plane correspondences, plus independent
validation points. Four corner dots alone do not satisfy this fitter. Keep every
reference dot visible and outside the bins.

Use robot-base coordinates, millimetres, and the cube TOP plane 20 mm above the
floor. The four workspace corners define the pickup polygon, not camera intrinsics.
Record each bin's centre, inside diameter and rim height. The two reported camera
distances (530 and 470 mm) cannot replace calibration or distortion correction.
The wide-angle lens particularly needs distortion calibration.

`vision_contract.py` rejects unvalidated calibration, stale/repeated frames,
nonfinite values, wrong cube dimensions, and cubes crossing the pickup boundary.
A valid observation specifies the TOP face; the physical grasp offset must be
calibrated separately. Processing timestamps do not measure camera/network latency.
To list missing measured configuration without sending any motor commands:

```bash
/path/to/robot-arm-project/.venv/bin/python /path/to/robot-arm-project/autonomous/vision_contract.py
```

## Remaining physical integration

Before real autonomous sorting, measured camera calibration, bin geometry, gripper
aperture/grasp offset and stream latency are required. Then implement and verify a
physical executor using those observations, including visual pickup/drop success,
object tracking between frames, workspace/bin collision geometry, and recovery.
The current simulation controller deliberately has no physical execution option.

Existing physical firmware/empty-arm mirror tools are retained. All-six-servo
90-degree startup remains their contract, with the persistent STOP exception.
There are no joint encoders, so commanded pose is not measured pose. Unsupervised
reinforcement-learning exploration on the real arm is not enabled.

Installed ROS, Gazebo, MoveIt, OpenCV and Python dependencies are reused. The
simulation tests exercise them directly; no additional package installation is
needed for this entry point. Training uses the local lightweight Q-learning code.

### Independent calibration check

`validate_calibration.py --calibration FIT.json --points HELD_OUT_POINTS.json
--output VERIFIED.json` checks raw image pixel/robot-base millimetre pairs against
the fitted distortion/homography model. Supply four or more independently measured,
noncollinear reference points on the 20 mm plane; do not reuse fitting points.
It writes a validated mapping only when maximum error is at most 3 mm. This is a
configurable software threshold, not a guarantee of successful physical gripping.
See the script's --help for the JSON schema. Save the verified file's absolute path
in camera_config.json and copy its measured maximum error into the config.

### TPU gripper requirement

Every physical grasp must command the existing full-close endpoint: channel 7,
0 degrees (nominal 1200 microseconds), retained through lift/transfer until release.
The existing physical mirror host already used this endpoint; it is now explicitly
named in `live_trial/single_box.py`. Open/home remains 90 degrees (1900 us).
This requirement carries forward into camera-driven physical integration. Gazebo
uses separate finger clearance because its rigid geometry cannot deform like TPU.
A full-close command is not a measurement of jaw closure or gripping force.
