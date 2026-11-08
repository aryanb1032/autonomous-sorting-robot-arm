#pragma once
// PCA9685 implementation ONLY. Confirm your driver before enabling this.
constexpr bool HARDWARE_CALIBRATED = false;
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
// OE needs a verified pull-up to 3.3 V overcoming any board pull-down; HIGH disables PWM.
constexpr int OE_PIN = 23;
constexpr bool OE_WIRED = false;
constexpr unsigned char PCA_ADDRESS = 0x40;
constexpr int JOINT_COUNT = 6; // five arm axes + gripper; verify actual topology
// Replace every placeholder using measured safe calibration. Radians throughout.
constexpr int CHANNEL[JOINT_COUNT] = {-1,-1,-1,-1,-1,-1};
constexpr float RAD_LOW[JOINT_COUNT] = {0,0,0,0,0,0};
constexpr float RAD_HIGH[JOINT_COUNT] = {0,0,0,0,0,0};
constexpr float PULSE_AT_LOW[JOINT_COUNT] = {0,0,0,0,0,0};
constexpr float PULSE_AT_HIGH[JOINT_COUNT] = {0,0,0,0,0,0};
constexpr float INITIAL_RAD[JOINT_COUNT] = {0,0,0,0,0,0};
constexpr float MAX_RAD_PER_SEC[JOINT_COUNT] = {0,0,0,0,0,0};

constexpr const char* CONFIG_ID = "UNCONFIGURED";
