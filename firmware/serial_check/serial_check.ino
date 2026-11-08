// Communication check only: no servo pins are configured or driven.
#include <Arduino.h>
String command;
void setup() {
  Serial.begin(115200);
  command.reserve(32);
}
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      command.trim();
      if (command == "PING") Serial.println("PONG ROBOT_ARM_SERIAL_CHECK_V1");
      else Serial.println("ERR UNKNOWN_COMMAND");
      command = "";
    } else if (command.length() < 32) {
      command += c;
    } else {
      command = "";
    }
  }
}
