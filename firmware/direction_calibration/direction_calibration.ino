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
int armPulse[16]={0};int jogChannel=-1;
unsigned long jogStarted=0,jogContact=0,jogUpdate=0;
bool jogReturning=false;float jogDegrees=90,jogReturnStart=90;
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
void disable(){digitalWrite(OE,HIGH);holding=false;homing=false;jogChannel=-1;}
bool validate(){
 int mode=readReg(0),prescale=readReg(0xFE);
 if(mode<0 || (mode&0x10) || !(mode&0x20) || prescale!=121)return false;
 for(int c=0;c<16;c++){
  bool active=false;for(int ch:channels)if(ch==c)active=true;
  int onL=readReg(0x06+4*c),onH=readReg(0x07+4*c);
  int offL=readReg(0x08+4*c),offH=readReg(0x09+4*c);
  if(onL<0 || onH<0 || offL<0 || offH<0)return false;
  if(active){
   int us=c==7?gripperPulseUs:armPulse[c];
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
 for(int c:channels){armPulse[c]=1500;pca.writeMicroseconds(c,c==7?gripperPulseUs:1500);}
 if(!validate())return false;
 holding=true;homing=true;homeStarted=lastRamp=millis();digitalWrite(OE,LOW);return true;
}
void command(){
 if(!strcmp(input,"KEEPALIVE")){jogContact=millis();return;}
 if(!strncmp(input,"JOG ",4)){
  char *end;long c=strtol(input+4,&end,10);
  if(end==input+4 || *end || !(c==0 || c==1 || c==2 || c==5 || c==6)){Serial.println("ERR CHANNEL");return;}
  if(!holding || !ready || homing || jogChannel>=0){Serial.println("ERR NOT_READY");return;}
  jogChannel=c;jogStarted=jogContact=millis();jogUpdate=0;jogReturning=false;jogDegrees=90;
  Serial.printf("ACK JOG %d\n",jogChannel);return;
 }
 if(jogChannel>=0 && (!strcmp(input,"HOME") || !strcmp(input,"HOME90"))){Serial.println("ERR BUSY");return;}

 if(!strcmp(input,"INFO"))Serial.println("CONFIG DIRECTION_CALIBRATION_V1");
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
 if(holding && jogChannel>=0 && millis()-jogUpdate>=20){
  unsigned long now=millis();jogUpdate=now;
  if(!jogReturning && now-jogContact>1500){
   jogReturning=true;jogReturnStart=jogDegrees;jogStarted=now;
   Serial.println("WATCHDOG RETURNING_HOME");
  }
  float t=(now-jogStarted)/5000.0f;
  bool done=false;
  if(jogReturning){jogDegrees=jogReturnStart-(jogReturnStart-90)*min(1.0f,t);done=t>=1;}
  else if(t<1)jogDegrees=90+5*t;
  else if(t<2)jogDegrees=95-5*(t-1);
  else {jogDegrees=90;done=true;}
  armPulse[jogChannel]=(int)(500+2000*jogDegrees/180);
  pca.writeMicroseconds(jogChannel,armPulse[jogChannel]);
  if(!validate()){disable();ready=false;Serial.println("ERR JOG_CHECK DISABLED");}
  else if(done){jogChannel=-1;Serial.println("DONE HOME_HELD ESTIMATED");}
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
