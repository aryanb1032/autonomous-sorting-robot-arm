// Supported-arm commissioning only. All unselected channels remain FULL_OFF.
// No position sensing. Completion and communication timeout disable ALL outputs.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pca(0x40);
constexpr int OE=23, ADDRESS=0x40;
char input[80]; size_t used=0; bool overflow=false, ready=false;
int selected=-1;
unsigned long started=0, contact=0, updated=0;
bool writeReg(uint8_t reg,uint8_t value) {
  Wire.beginTransmission(ADDRESS); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission()==0;
}
bool allOff() {
  digitalWrite(OE,HIGH); selected=-1;
  bool ok=writeReg(0xFD,0x10);
  for(int c=0;c<16;c++) ok=writeReg(0x09+4*c,0x10)&&ok;
  return ok;
}
int expectedTicks(float us) {
  uint16_t microseconds=(uint16_t)us;
  return (int)(microseconds / (1000000.0 * (pca.readPrescale()+1) / pca.getOscillatorFrequency()));
}
bool pulse(int channel,float degrees) {
  float us=channel==7 ? 1000+1300*degrees/180 : 500+2000*degrees/180;
  pca.writeMicroseconds(channel,(uint16_t)us);
  return pca.getPWM(channel,false)==0 && pca.getPWM(channel,true)==expectedTicks(us);
}
int readReg(uint8_t reg) {
  Wire.beginTransmission(ADDRESS);Wire.write(reg);
  if(Wire.endTransmission(false)!=0)return -1;
  if(Wire.requestFrom(ADDRESS,1)!=1)return -1;
  return Wire.read();
}
bool isolated(int channel) {
  int expected=expectedTicks(channel==7?1650.0f:1500.0f);
  for(int c=0;c<16;c++) {
    int offH=readReg(0x09+4*c);
    if(offH<0)return false;
    if(c==channel) {
      if(readReg(0x06+4*c)!=0 || readReg(0x07+4*c)!=0 ||
         readReg(0x08+4*c)!=(expected&255) || offH!=((expected>>8)&15))return false;
    } else if(!(offH&0x10))return false;
  }
  return true;
}
void checkRegisters() {
  // OE stays HIGH throughout: register test never drives a servo output.
  if(!allOff()){Serial.println("ERR I2C");return;}
  bool passed=true;
  for(int testChannel : {0,7}) {
    bool ok=allOff() && pulse(testChannel,90);
    int expected=expectedTicks(testChannel==7?1650.0f:1500.0f);
    for(int c=0;c<16;c++) {
      int onL=readReg(0x06+4*c),onH=readReg(0x07+4*c);
      int offL=readReg(0x08+4*c),offH=readReg(0x09+4*c);
      bool valid=onL>=0 && onH>=0 && offL>=0 && offH>=0;
      if(c==testChannel)valid=valid && onL==0 && onH==0 && offL==(expected&255) && offH==((expected>>8)&15);
      else valid=valid && (offH&0x10);
      ok=valid&&ok;
      Serial.printf("REG test=%d channel=%d onL=%d onH=%d offL=%d offH=%d valid=%d\n",testChannel,c,onL,onH,offL,offH,valid);
    }
    passed=ok&&passed;
  }
  passed=allOff()&&passed;
  Serial.printf("CHECK %s MODE1=%d MODE2=%d PRESCALE=%d OE=%d\n",passed?"PASS":"FAIL",readReg(0),readReg(1),readReg(0xFE),digitalRead(OE));
}
void command() {
  if(!strcmp(input,"CHECK")){checkRegisters();return;}
  if(!strcmp(input,"PING")){Serial.println("PONG SINGLE_CHANNEL_ADAFRUIT_V1");return;}
  if(!strcmp(input,"INFO")){Serial.println("CONFIG SINGLE_CHANNEL_ADAFRUIT_V1");return;}
  if(!strcmp(input,"STATE")){
    Serial.printf("STATE ready=%d selected=%d oe=%d\n",ready,selected,digitalRead(OE));return;
  }
  if(!strcmp(input,"STOP")){bool ok=allOff();Serial.println(ok?"DISABLED":"ERR I2C");return;}
  if(!strcmp(input,"KEEPALIVE")){contact=millis();return;}
  if(strncmp(input,"RUN ",4)){Serial.println("ERR COMMAND");return;}
  char *end; long c=strtol(input+4,&end,10);
  if(end==input+4 || *end || !(c==0 || c==1 || c==2 || c==5 || c==6 || c==7)){
    Serial.println("ERR CHANNEL");return;
  }
  if(!ready || selected>=0){Serial.println("ERR NOT_READY");return;}
  // Disable every channel first; configure only the selected channel while OE is HIGH.
  // ALL_LED_OFF_H is a broadcast write, NOT a separate global enable bit.
  // Never clear it after configuring individual channels.
  if(!allOff() || !pulse(c,90) || !isolated(c)){
    allOff();ready=false;Serial.println("ERR I2C");return;
  }
  selected=c;started=contact=millis();updated=0;
  digitalWrite(OE,LOW); Serial.printf("ACK CHANNEL %d\n",selected);
}
void setup() {
  digitalWrite(OE,HIGH);pinMode(OE,OUTPUT);Serial.begin(115200);
  Wire.begin(21,22);Wire.setTimeOut(20);
  bool ok=pca.begin();
  if(ok){pca.setPWMFreq(50);delay(500);}
  ready=allOff()&&ok;
  Serial.println(ready?"SINGLE_CHANNEL_ADAFRUIT_V1 DISABLED":"ERR I2C");
}
void loop() {
  while(Serial.available()){
    char c=Serial.read();if(c=='\r')continue;
    if(c=='\n'){
      input[used]=0;if(!overflow)command();else Serial.println("ERR OVERFLOW");
      used=0;overflow=false;
    }else if(used<sizeof(input)-1 && !overflow)input[used++]=c;else overflow=true;
  }
  if(selected<0)return;
  unsigned long now=millis(),elapsed=now-started;
  if(now-contact>1500){allOff();Serial.println("ERR WATCHDOG DISABLED");return;}
  if(elapsed>=5500){bool ok=allOff();Serial.println(ok?"DONE ESTIMATED DISABLED":"ERR I2C");return;}
  if(now-updated>=20){
    updated=now;
    float degrees=90;
    if(elapsed>=1000 && elapsed<3000)degrees=90+2*(elapsed-1000)/2000.0f;
    else if(elapsed>=3000 && elapsed<5000)degrees=92-2*(elapsed-3000)/2000.0f;
    if(!pulse(selected,degrees)){allOff();ready=false;Serial.println("ERR I2C DISABLED");}
  }
}
