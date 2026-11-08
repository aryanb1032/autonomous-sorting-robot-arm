"""Compile and exercise actual firmware startup/STOP logic with simulated IO, no serial access."""
from pathlib import Path
import subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as folder:
 d=Path(folder)
 (d/'Arduino.h').write_text('''#pragma once
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <cmath>
using std::min;
#define HIGH 1
#define LOW 0
#define OUTPUT 1
inline int oe=1;
inline unsigned long clockMs=100;
inline void digitalWrite(int,int v){oe=v;}
inline void pinMode(int,int){}
inline unsigned long millis(){return clockMs;}
inline void delay(int n){clockMs+=n;}
struct SerialStub {void begin(int){} template<class T> void print(T){} template<class T> void print(T,int){} template<class T> void println(T){} void println(){} int available(){return 0;} char read(){return 0;}};
inline SerialStub Serial;
''')
 (d/'Wire.h').write_text('''#pragma once
struct WireStub {int registers[256]={};int index=0;bool first=true;void begin(int,int){} void setTimeOut(int){} void beginTransmission(int){first=true;} void write(int x){if(first){index=x;first=false;}else registers[index++]=x;} int endTransmission(){return 0;}};
inline WireStub Wire;
''')
 (d/'Preferences.h').write_text('''#pragma once
struct Preferences {inline static bool latch=false;bool begin(const char*,bool){return true;} bool getBool(const char*,bool){return latch;} int putBool(const char*,bool v){latch=v;return 1;}};
''')
 for name in ['live_trial','virtual_pick']:
  src=ROOT/'firmware'/name/(name+'.ino')
  harness=d/'test.cpp'
  harness.write_text('#include <cassert>\n#include "'+str(src)+'"\n'+'''
void sendCommand(const char* value){char buffer[240];strcpy(buffer,value);command(buffer);}
int main(){
 setup();assert(enabled && ready && oe==LOW && !moving);
 for(int i=0;i<6;i++){
  float pulse=PULSE_AT_LOW[i]+(INITIAL_RAD[i]-RAD_LOW[i])/(RAD_HIGH[i]-RAD_LOW[i])*(PULSE_AT_HIGH[i]-PULSE_AT_LOW[i]);
  assert(fabs(pulse-(i==5?1900:1500))<.01);
  int channel=CHANNEL[i];int actual=Wire.registers[8+4*channel]+256*Wire.registers[9+4*channel];
  assert(actual==lround(pulse*4096/20000));
 }
 sendCommand("INIT CONFIRM_INITIAL_POSE");assert(enabled);
 sendCommand("STOP");assert(!enabled && oe==HIGH && Preferences::latch);
 setup();assert(!enabled && oe==HIGH && stopped);
 sendCommand("INIT CONFIRM_INITIAL_POSE");assert(!enabled);
 sendCommand("HOME90");assert(enabled && oe==LOW && !stopped && !Preferences::latch);
 sendCommand("MOVE 1000 0 0 0 0 0 1.47079633");assert(moving);
 clockMs+=1000;loop();assert(!moving && fabs(currentQ[5]-1.47079633)<.00001);
 sendCommand("INIT CONFIRM_INITIAL_POSE");assert(fabs(currentQ[5]-1.47079633)<.00001);
 sendCommand("STOP");assert(!enabled && oe==HIGH);
}
''')
  subprocess.run(['g++','-std=c++17','-I',str(d),str(harness),'-o',str(d/'test')],check=True)
  subprocess.run([str(d/'test')],check=True)
  print(name+': startup PWM, latched STOP, explicit HOME90, INIT and trajectory behavior PASS')
