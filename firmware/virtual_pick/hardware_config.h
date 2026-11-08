#pragma once
constexpr bool HARDWARE_CALIBRATED=true; // User-reported nominal mapping for bounded no-box trial only.
constexpr bool OE_WIRED=true;
constexpr int OE_PIN=23,SDA_PIN=21,SCL_PIN=22,JOINT_COUNT=6;
constexpr unsigned char PCA_ADDRESS=0x40;
constexpr const char* CONFIG_ID="VIRTUAL_PICK_HOME90_V2";
constexpr int CHANNEL[6]={0,1,2,5,6,7};
constexpr float RAD_LOW[6]={-1.5707963267948966,-1.5707963267948966,-1.5707963267948966,-1.5707963267948966,-1.5707963267948966,0};
constexpr float RAD_HIGH[6]={1.5707963267948966,1.5707963267948966,1.5707963267948966,1.5707963267948966,1.5707963267948966,1.5707963267948966};
constexpr float INITIAL_RAD[6]={0,0,0,0,0,1.5707963267948966};
constexpr float MAX_RAD_PER_SEC[6]={0.2,0.2,0.2,0.2,0.2,0.2};
constexpr float PULSE_AT_LOW[6]={500,2500,500,500,500,1200};
constexpr float PULSE_AT_HIGH[6]={2500,500,2500,2500,2500,1900};

// Project contract: nominal servo home is 90 degrees for all six channels.
static_assert(INITIAL_RAD[0]==0 && INITIAL_RAD[1]==0 && INITIAL_RAD[2]==0 && INITIAL_RAD[3]==0 && INITIAL_RAD[4]==0, "Arm home must remain 90 degrees");
static_assert(INITIAL_RAD[5]>1.57079f && INITIAL_RAD[5]<1.57080f, "Gripper home must remain 90 degrees");
