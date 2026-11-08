#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pca(0x40);
constexpr int OE=23;
const int channels[]={0,1,2,5,6,7};
int pulses[6]={1500,1500,1500,1500,1500,1822};
bool ready=false,enabled=false,running=false;
int indexNow=0,phase=-1;
unsigned long started=0,lastUpdate=0,lastCheck=0,lastContact=0;
char input[64];size_t used=0;bool overflow=false;
int readReg(int reg){Wire.beginTransmission(0x40);Wire.write(reg);if(Wire.endTransmission(false)!=0 || Wire.requestFrom(0x40,1)!=1)return -1;return Wire.read();}
void stop(){digitalWrite(OE,HIGH);enabled=false;running=false;}
bool validate(){
 int pre=readReg(0xFE),mode=readReg(0);
 if(pre!=121 || mode<0 || (mode&0x10) || !(mode&0x20))return false;
 for(int c=0;c<16;c++){
  int idx=-1;for(int i=0;i<6;i++)if(channels[i]==c)idx=i;
  int a=readReg(6+4*c),b=readReg(7+4*c),d=readReg(8+4*c),e=readReg(9+4*c);
  if(a<0||b<0||d<0||e<0)return false;
  if(idx<0){if(!(e&16))return false;}
  else {int tick=(int)(pulses[idx]/(1000000.0*(pre+1)/pca.getOscillatorFrequency()));if(a!=0||b!=0||d!=(tick&255)||e!=((tick>>8)&15))return false;}
 }
 return true;
}
bool home(){
 stop();if(!ready)return false;
 for(int c=0;c<16;c++)if(pca.setPWM(c,0,4096)!=0)return false;
 for(int i=0;i<6;i++){pulses[i]=i==5?1822:1500;pca.writeMicroseconds(channels[i],pulses[i]);}
 if(!validate())return false;
 digitalWrite(OE,LOW);enabled=true;return true;
}
void command(){
 if(!strcmp(input,"INFO")){Serial.println("CONFIG ONE_PASS_DIRECTION_V1");return;}
 if(!strcmp(input,"STATE")){Serial.printf("STATE ready=%d enabled=%d running=%d channel=%d oe=%d\n",ready,enabled,running,channels[indexNow],digitalRead(OE));return;}
 if(!strcmp(input,"STOP")){stop();Serial.println("DISABLED");return;}
 if(!strcmp(input,"KEEPALIVE")){lastContact=millis();return;}
 if(!strcmp(input,"RUN_ALL")){
  if(running){Serial.println("ERR BUSY");return;}
  if(!home()){Serial.println("ERR HOME DISABLED");return;}
  indexNow=0;phase=-1;started=millis();lastContact=started;running=true;
  Serial.println("ACK RUN_ALL HOME_HELD");return;
 }
 Serial.println("ERR COMMAND");
}
void setup(){
 digitalWrite(OE,HIGH);pinMode(OE,OUTPUT);Serial.begin(115200);
 Wire.begin(21,22);Wire.setTimeOut(20);ready=pca.begin();
 if(ready){pca.setPWMFreq(50);delay(500);for(int c=0;c<16;c++)if(pca.setPWM(c,0,4096)!=0)ready=false;}
 Serial.println(ready?"ONE_PASS_DIRECTION_V1 DISABLED WAIT_RUN":"ERR I2C");
}
void loop(){
 while(Serial.available()){
  char c=Serial.read();if(c=='\r')continue;
  if(c=='\n'){input[used]=0;if(!overflow)command();else Serial.println("ERR OVERFLOW");used=0;overflow=false;}
  else if(used<sizeof(input)-1&&!overflow)input[used++]=c;else overflow=true;
 }
 unsigned long now=millis();
 if(running && now-lastContact>1500){running=false;Serial.println("ERR WATCHDOG MOTION_HALTED HOLDING_LAST");}
 if(running && now-lastUpdate>=20){
  lastUpdate=now;
  unsigned long elapsed=now-started;
  unsigned long ramp=indexNow==5?8000:4000;
  float homeAngle=indexNow==5?80:90,target=indexNow==5?0:110,angle=homeAngle;
  int nextPhase;
  if(elapsed<1000){nextPhase=0;}
  else if(elapsed<1000+ramp){nextPhase=1;angle=homeAngle+(target-homeAngle)*(elapsed-1000)/(float)ramp;}
  else if(elapsed<3000+ramp){nextPhase=2;angle=target;}
  else if(elapsed<3000+2*ramp){nextPhase=3;angle=target+(homeAngle-target)*(elapsed-3000-ramp)/(float)ramp;}
  else {nextPhase=4;}
  if(nextPhase!=phase){phase=nextPhase;Serial.printf("CHANNEL %d PHASE %s\n",channels[indexNow],phase==0?"HOME":phase==1?"OUTWARD":phase==2?"TARGET_HOLD":phase==3?"RETURN_HOME":"HOME_WAIT_5_SECONDS");}
  pulses[indexNow]=(int)((indexNow==5?1200:500)+(indexNow==5?1400:2000)*angle/180.0f);
  pca.writeMicroseconds(channels[indexNow],pulses[indexNow]);
  if(pca.getPWM(channels[indexNow],true)!=(int)(pulses[indexNow]/(1000000.0*(pca.readPrescale()+1)/pca.getOscillatorFrequency()))){stop();ready=false;Serial.println("ERR PWM DISABLED");}
  else if(elapsed>=3000+2*ramp+(indexNow==5?0:5000)){
   if(indexNow==5){running=false;Serial.println("DONE ALL HOME_HELD ESTIMATED");}
   else {indexNow++;phase=-1;started=now;}
  }
 }
 if(enabled && now-lastCheck>=1000){lastCheck=now;if(!validate()){stop();ready=false;Serial.println("ERR REGISTER_CHECK DISABLED");}}
}
