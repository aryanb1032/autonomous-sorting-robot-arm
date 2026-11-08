// Positions and DONE messages are commanded estimates, never encoder feedback.
// On timeout/STOP, hold last commanded position. Power removal can drop the arm.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include "hardware_config.h"
Preferences settings;
bool stopped=false,storageReady=false;
char line[240]; unsigned int used=0; bool overflow=false;
bool ready=false, enabled=false, moving=false;
float currentQ[JOINT_COUNT], startQ[JOINT_COUNT], targetQ[JOINT_COUNT];
unsigned long started=0, durationMs=0, lastContact=0, lastUpdate=0;
bool regWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA_ADDRESS); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission()==0;
}
bool pwm(int channel, float pulseUs) {
  int tick=lround(pulseUs*4096.0f/20000.0f);
  Wire.beginTransmission(PCA_ADDRESS); Wire.write(0x06+4*channel);
  Wire.write(0); Wire.write(0); Wire.write(tick & 255); Wire.write((tick>>8)&15);
  return Wire.endTransmission()==0;
}
bool outputQ() {
  for(int i=0;i<JOINT_COUNT;i++) {
    float pulse=PULSE_AT_LOW[i]+(currentQ[i]-RAD_LOW[i])/(RAD_HIGH[i]-RAD_LOW[i])*(PULSE_AT_HIGH[i]-PULSE_AT_LOW[i]);
    if(!pwm(CHANNEL[i],pulse)) return false;
  }
  return true;
}
void state() {
  Serial.print("STATE "); Serial.print(enabled?1:0);
  for(int i=0;i<JOINT_COUNT;i++){Serial.print(' ');Serial.print(currentQ[i],6);}
  Serial.println();
}
bool validConfig() {
  if(!HARDWARE_CALIBRATED || !OE_WIRED || OE_PIN<0 || OE_PIN==SDA_PIN || OE_PIN==SCL_PIN || SDA_PIN<0 || SCL_PIN<0 || SDA_PIN==SCL_PIN) return false;
  for(int i=0;i<JOINT_COUNT;i++) {
    if(!isfinite(RAD_LOW[i]) || !isfinite(RAD_HIGH[i]) || !isfinite(MAX_RAD_PER_SEC[i]) || !isfinite(PULSE_AT_LOW[i]) || !isfinite(PULSE_AT_HIGH[i]) || PULSE_AT_LOW[i]==PULSE_AT_HIGH[i]) return false;
    if(CHANNEL[i]<0 || CHANNEL[i]>15 || !(RAD_HIGH[i]>RAD_LOW[i]) || !(MAX_RAD_PER_SEC[i]>0)) return false;
    if(!isfinite(INITIAL_RAD[i]) || INITIAL_RAD[i]<RAD_LOW[i] || INITIAL_RAD[i]>RAD_HIGH[i]) return false;
    if(PULSE_AT_LOW[i]<400 || PULSE_AT_LOW[i]>2600 || PULSE_AT_HIGH[i]<400 || PULSE_AT_HIGH[i]>2600) return false;
    for(int j=0;j<i;j++) if(CHANNEL[i]==CHANNEL[j]) return false;
  }
  return true;
}
bool initialCommand() {
  for(int i=0;i<JOINT_COUNT;i++)if(fabs(currentQ[i]-INITIAL_RAD[i])>0.00001f)return false;
  return true;
}
void command(char* text) {
  if(!strcmp(text,"INFO")){Serial.print("CONFIG ");Serial.println(CONFIG_ID);return;}
  if(!strcmp(text,"PING")){Serial.println("PONG ARM_V1");return;}
  if(!strcmp(text,"STATE")){state();return;}
  if(!strcmp(text,"KEEPALIVE")){lastContact=millis();return;}
  if(!strcmp(text,"STOP")){
    moving=false;enabled=false;digitalWrite(OE_PIN,HIGH);
    if(!stopped && storageReady)settings.putBool("stopped",true);
    stopped=true;Serial.println("DISABLED");return;
  }
  // Explicit operator command clears a latched STOP and commands nominal home.
  // Initial physical angle is unknown: this is not sensor-based homing.
  if(!strcmp(text,"HOME90")){
    if(!ready || !storageReady || moving){Serial.println("ERR HOME");return;}
    if(settings.putBool("stopped",false)!=1){Serial.println("ERR STORAGE");return;}
    stopped=false;
    for(int i=0;i<JOINT_COUNT;i++)currentQ[i]=INITIAL_RAD[i];
    enabled=outputQ();digitalWrite(OE_PIN,enabled?LOW:HIGH);
    Serial.println(enabled?"HOME90 HELD ESTIMATED":"ERR I2C");return;
  }
  if(!strcmp(text,"INIT CONFIRM_INITIAL_POSE")) {
    if(!ready || stopped || !enabled || moving || !initialCommand()){Serial.println("ERR INIT NOT_HOME_OR_STOPPED");return;}
    lastContact=millis();Serial.println("READY");return;
  }
  if(strncmp(text,"MOVE ",5)){Serial.println("ERR COMMAND");return;}
  if(!enabled || moving){Serial.println("ERR NOT_READY");return;}
  char* token=strtok(text+5," ");
  if(!token){Serial.println("ERR FORMAT");return;}
  char* end; long ms=strtol(token,&end,10);
  if(*end || ms<20 || ms>10000){Serial.println("ERR DURATION");return;}
  float q[JOINT_COUNT];
  for(int i=0;i<JOINT_COUNT;i++) {
    token=strtok(nullptr," ");if(!token){Serial.println("ERR FORMAT");return;}
    q[i]=strtof(token,&end);
    if(*end || !isfinite(q[i]) || q[i]<RAD_LOW[i] || q[i]>RAD_HIGH[i] || fabs(q[i]-currentQ[i])/(ms/1000.0f)>MAX_RAD_PER_SEC[i]+0.0001f){Serial.println("ERR LIMIT");return;}
  }
  if(strtok(nullptr," ")){Serial.println("ERR FORMAT");return;}
  for(int i=0;i<JOINT_COUNT;i++){startQ[i]=currentQ[i];targetQ[i]=q[i];}
  durationMs=ms;started=millis();lastContact=started;moving=true;Serial.println("ACK");
}
void setup() {
  digitalWrite(OE_PIN,HIGH);pinMode(OE_PIN,OUTPUT);
  Serial.begin(115200);
  storageReady=settings.begin("robot-home",false);
  if(!storageReady){Serial.println("ERR STORAGE DISABLED");return;}
  stopped=settings.getBool("stopped",false);
  for(int i=0;i<JOINT_COUNT;i++)currentQ[i]=INITIAL_RAD[i];
  if(!validConfig()){Serial.println("LOCKED CONFIG_REQUIRED");return;}
  Wire.begin(SDA_PIN,SCL_PIN);Wire.setTimeOut(20);
  // 25 MHz nominal PCA9685 oscillator, 50 Hz PWM. Validate pulse widths on hardware.
  bool ok=regWrite(0x00,0x10) && regWrite(0xFE,121) && regWrite(0x00,0x20);
  delay(1);ok=ok && regWrite(0x00,0xA0);
  // Keep OE disabled until every home pulse is configured.
  ok=ok && regWrite(0xFD,0x10);
  ready=ok;
  // ALL_LED_OFF_H broadcasts to individual registers; never clear it globally.
  // INIT writes only the configured channels, leaving all others FULL_OFF.
  if(ready){for(int c=0;c<16;c++)ready=regWrite(0x09+4*c,0x10)&&ready;}
  if(!ready){Serial.println("ERR I2C");return;}
  if(stopped){Serial.println("STOP LATCHED: HOME90 required");return;}
  enabled=outputQ();digitalWrite(OE_PIN,enabled?LOW:HIGH);
  Serial.println(enabled?"BOOT HOME90 HELD ESTIMATED":"ERR I2C");
}
void loop() {
  while(Serial.available()) {
    char c=Serial.read();if(c=='\r')continue;
    if(c=='\n') {line[used]=0;if(!overflow)command(line);else Serial.println("ERR OVERFLOW");used=0;overflow=false;}
    else if(used<sizeof(line)-1 && !overflow)line[used++]=c;else overflow=true;
  }
  unsigned long now=millis();
  if(moving && now-lastContact>1500){moving=false;Serial.println("ERR WATCHDOG");}
  if(moving && now-lastUpdate>=20) {
    lastUpdate=now;float t=min(1.0f,(now-started)/(float)durationMs);
    for(int i=0;i<JOINT_COUNT;i++)currentQ[i]=startQ[i]+t*(targetQ[i]-startQ[i]);
    if(!outputQ()){moving=false;enabled=false;digitalWrite(OE_PIN,HIGH);Serial.println("ERR I2C");}
    else if(t>=1){moving=false;Serial.println("DONE ESTIMATED");}
  }
}
