# Resume project entry

## Autonomous Colour-Sorting Robotic Arm | ROS 2 Jazzy, MoveIt 2, Gazebo, RViz, OpenCV, ESP32 | BOM: ₹5,500

- Built a six-DOF robotic arm with a servo-driven rack-and-pinion gripper and adaptive TPU fingers to handle 20 mm black and white cubes.
- Developed ROS 2 robot descriptions and motion-planning workflows, and simulated autonomous pick-and-place sorting with MoveIt 2, Gazebo and RViz.
- Implemented ESP32/PCA9685 servo-control firmware, serial communication, homing behavior, and emergency-stop handling.
- Developed OpenCV colour-detection and camera-calibration workflows for fixed-camera object sorting.
- Tested simulation-based Q-learning to select cube sorting order and log task outcomes.
- Verified a six-cube Gazebo episode with three cubes of each colour sorted and no blocked objects. The owner later reported successful physical autonomous sorting after integration changes on a second laptop.

### Short version

Built a ₹5,500 six-DOF robotic arm with an adaptive rack-and-pinion gripper for autonomous black/white cube sorting. Integrated ROS 2 Jazzy, MoveIt 2, Gazebo, RViz, OpenCV, ESP32 and PCA9685 servo control; tested simulation-based Q-learning for sorting order. Completed six-cube sorting in simulation; later physical sorting was reported after changes on a second laptop.

### Interview wording

The fingers move together through a rack-and-pinion mechanism, with TPU providing compliant contact. Simulation uses Gazebo ground truth and simulated grasp attachment. The saved RL evidence learns sorting order in simulation; it is not evidence of end-to-end physical reinforcement learning. Physical autonomy was achieved after later integration changes on another laptop, according to the project owner.
