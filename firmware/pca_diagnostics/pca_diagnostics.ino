#include <Arduino.h>
#include <Wire.h>
// Diagnostic only: does not program any servo PWM register.
// Upload with servo V+ disconnected. I2C acknowledges do not identify a chip model.
String line;
void scan() {
  int found=0;
  for (uint8_t address=8;address<120;address++) {
    Wire.beginTransmission(address);
    if(Wire.endTransmission()==0) {Serial.printf("I2C_ACK 0x%02X\n",address);found++;}
  }
  Serial.printf("SCAN_DONE %d\n",found);
}
void setup(){Serial.begin(115200);Wire.begin(21,22);Wire.setTimeOut(20);Serial.println("DIAGNOSTIC_ONLY SDA=21 SCL=22; send PING or SCAN");}
void loop(){
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'){
      line.trim();
      if(line=="PING")Serial.println("PONG PCA_DIAGNOSTIC");
      else if(line=="SCAN")scan();
      else Serial.println("ERR send PING or SCAN");
      line="";
    }else if(c!='\r'){
      if(line.length()<64)line+=c;else line="";
    }
  }
}
