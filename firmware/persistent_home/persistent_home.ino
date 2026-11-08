// Dedicated automatic nominal-home firmware. No trajectory or sorting commands.
// Outputs HOLD continuously until power loss, explicit STOP, or detected I2C fault.
// Power-on can move every connected servo immediately. No position feedback.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pca(0x40);
constexpr int OE=23;
const int channels[]={0,1,2,5,6,7};
bool holding=false,ready=false;
char input[64];size_t used=0;bool overflow=false;
unsigned long lastCheck=0,homeStarted=0,lastRamp=0;
bool homing=false;
float gripperDegrees=90;
int gripperPulseUs=1900;
constexpr unsigned long RAMP_MS=5000;
int pulseForGripper(float degrees){return (int)(1200.0f+1400.0f*degrees/180.0f);}
int readReg(uint8_t reg){
 Wire.beginTransmission(0x40);Wire.write(reg);
 if(Wire.endTransmission(false)!=0 || Wire.requestFrom(0x40,1)!=1)return -1;
 return Wire.read();
}
void disable(){digitalWrite(OE,HIGH);holding=false;homing=false;}
bool validate(){
 int mode=readReg(0),prescale=readReg(0xFE);
 if(mode<0 || (mode&0x10) || !(mode&0x20) || prescale!=121)return false;
 for(int c=0;c<16;c++){
  bool active=false;for(int ch:channels)if(ch==c)active=true;
  int onL=readReg(0x06+4*c),onH=readReg(0x07+4*c);
  int offL=readReg(0x08+4*c),offH=readReg(0x09+4*c);
  if(onL<0 || onH<0 || offL<0 || offH<0)return false;
  if(active){
   int us=c==7?gripperPulseUs:1500;
   int ticks=(int)(us/(1000000.0*(prescale+1)/pca.getOscillatorFrequency()));
   if(onL!=0 || onH!=0 || offL!=(ticks&255) || offH!=((ticks>>8)&15))return false;
  }else if(!(offH&0x10))return false;
 }
 return true;
}
bool home(){
 disable();
 if(!ready)return false;
 for(int c=0;c<16;c++)if(pca.setPWM(c,0,4096)!=0)return false;
 gripperDegrees=90;gripperPulseUs=pulseForGripper(gripperDegrees);
 for(int c:channels)pca.writeMicroseconds(c,c==7?gripperPulseUs:1500);
 if(!validate())return false;
 holding=true;homing=true;homeStarted=lastRamp=millis();digitalWrite(OE,LOW);return true;
}
void command(){
 if(!strcmp(input,"INFO"))Serial.println("CONFIG PERSISTENT_HOME_GRIP80_V2");
 else if(!strcmp(input,"STATE"))Serial.printf("STATE ready=%d holding=%d oe=%d homing=%d gripper_command_deg=%.2f gripper_us=%d\n",ready,holding,digitalRead(OE),homing,gripperDegrees,gripperPulseUs);
 else if(!strcmp(input,"STOP")){disable();Serial.println("DISABLED UNTIL HOME90 OR REBOOT");}
 else if((!strcmp(input,"HOME") || !strcmp(input,"HOME90")))Serial.println(home()?"HOMING ARM90 GRIPPER90_TO80":"ERR HOME DISABLED");
 else Serial.println("ERR COMMAND");
}
void setup(){
 digitalWrite(OE,HIGH);pinMode(OE,OUTPUT);Serial.begin(115200);
 Wire.begin(21,22);Wire.setTimeOut(20);
 ready=pca.begin();
 if(ready){pca.setPWMFreq(50);delay(500);}
 Serial.println(home()?"HOMING ARM90 GRIPPER90_TO80":"ERR STARTUP DISABLED");
}
void loop(){
 while(Serial.available()){
  char c=Serial.read();if(c=='\r')continue;
  if(c=='\n'){input[used]=0;if(!overflow)command();else Serial.println("ERR OVERFLOW");used=0;overflow=false;}
  else if(used<sizeof(input)-1 && !overflow)input[used++]=c;else overflow=true;
 }
 if(holding && homing && millis()-lastRamp>=20){
  unsigned long now=millis();lastRamp=now;
  float fraction=min(1.0f,(now-homeStarted)/(float)RAMP_MS);
  gripperDegrees=90.0f-10.0f*fraction;
  gripperPulseUs=pulseForGripper(gripperDegrees);
  pca.writeMicroseconds(7,gripperPulseUs);
  if(!validate()){disable();ready=false;Serial.println("ERR RAMP_CHECK DISABLED");}
  else if(fraction>=1){homing=false;Serial.println("HOLDING ARM90 GRIPPER80 CONTINUOUS");}
 }
 if(holding && millis()-lastCheck>=1000){
  lastCheck=millis();
  if(!validate()){disable();ready=false;Serial.println("ERR REGISTER_CHECK DISABLED");}
 }
}
