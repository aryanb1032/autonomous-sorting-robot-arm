#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca(0x40);
WebServer server(80);

// =====================================================
// WIFI
// =====================================================

const char* ssid = "6CH_Servo_Calibrator";
const char* password = "12345678";

// Channels we are using
int allowedChannels[] = {0, 1, 2, 5, 6, 7};
const int NUM_CHANNELS = 6;

// =====================================================
// WEB PAGE
// =====================================================

const char webpage[] PROGMEM = R"rawliteral(

<!DOCTYPE html>
<html>

<head>

<meta name="viewport" content="width=device-width, initial-scale=1">

<title>6CH Servo Calibrator</title>

<style>

body {
  background:#101010;
  color:white;
  font-family:Arial;
  text-align:center;
  margin:15px;
}

h1 {
  margin-bottom:5px;
}

.subtitle {
  color:#aaa;
  margin-bottom:25px;
}

.servo {
  background:#202020;
  border-radius:15px;
  padding:18px;
  margin:18px auto;
  max-width:750px;
}

.channel {
  font-size:26px;
  font-weight:bold;
}

.value {
  font-size:42px;
  font-weight:bold;
  margin:10px;
}

.unit {
  font-size:18px;
  color:#aaa;
}

input[type=range] {
  width:90%;
  height:35px;
}

.scale {
  display:flex;
  justify-content:space-between;
  width:90%;
  margin:auto;
  color:#888;
}

button {
  padding:11px 15px;
  margin:5px;
  border:0;
  border-radius:8px;
  font-size:15px;
  cursor:pointer;
}

.step {
  background:#2196F3;
  color:white;
}

.center {
  background:#00c853;
  color:white;
}

.save {
  background:#ff9800;
  color:white;
}

.saved {
  margin-top:10px;
  color:#ddd;
}

.allCenter {
  background:#e91e63;
  color:white;
  font-size:20px;
  padding:15px 30px;
}

</style>

</head>

<body>

<h1>6-CHANNEL SERVO CALIBRATOR</h1>

<div class="subtitle">
PCA9685 Virtual Potentiometers
</div>

<button class="allCenter" onclick="centerAll()">
CENTER ALL SERVOS
</button>

<div id="container"></div>

<script>

const channels = [0,1,2,5,6,7];

let pwm = {
  0:1500,
  1:1500,
  2:1500,
  5:1500,
  6:1500,
  7:1500
};


// =============================================
// CREATE SIX VIRTUAL POTENTIOMETERS
// =============================================

function createControls() {

  let html = "";

  channels.forEach(ch => {

    html += `

    <div class="servo">

      <div class="channel">
        CHANNEL ${ch}
      </div>

      <div class="value">
        <span id="value${ch}">1500</span>
        <span class="unit">us</span>
      </div>

      <input
        type="range"
        id="slider${ch}"
        min="500"
        max="2500"
        step="5"
        value="1500"
        oninput="moveServo(${ch},this.value)"
      >

      <div class="scale">
        <span>500</span>
        <span>1000</span>
        <span>1500</span>
        <span>2000</span>
        <span>2500</span>
      </div>

      <br>

      <button class="step"
        onclick="changePWM(${ch},-50)">
        -50
      </button>

      <button class="step"
        onclick="changePWM(${ch},-10)">
        -10
      </button>

      <button class="center"
        onclick="setPWM(${ch},1500)">
        CENTER
      </button>

      <button class="step"
        onclick="changePWM(${ch},10)">
        +10
      </button>

      <button class="step"
        onclick="changePWM(${ch},50)">
        +50
      </button>

      <br>

      <button class="save"
        onclick="saveValue(${ch},'MIN')">
        SAVE MIN
      </button>

      <button class="save"
        onclick="saveValue(${ch},'HOME')">
        SAVE HOME
      </button>

      <button class="save"
        onclick="saveValue(${ch},'MAX')">
        SAVE MAX
      </button>

      <div class="saved">

        MIN:
        <span id="min${ch}">---</span>

        &nbsp;&nbsp;

        HOME:
        <span id="home${ch}">---</span>

        &nbsp;&nbsp;

        MAX:
        <span id="max${ch}">---</span>

      </div>

    </div>

    `;
  });

  document.getElementById("container").innerHTML = html;
}


// =============================================
// MOVE SERVO
// =============================================

function moveServo(ch,value) {

  value = parseInt(value);

  pwm[ch] = value;

  document.getElementById("value"+ch).innerHTML = value;

  fetch("/set?ch=" + ch + "&pulse=" + value);
}


// =============================================
// SET EXACT PWM
// =============================================

function setPWM(ch,value) {

  pwm[ch] = value;

  document.getElementById("slider"+ch).value = value;
  document.getElementById("value"+ch).innerHTML = value;

  fetch("/set?ch=" + ch + "&pulse=" + value);
}


// =============================================
// +/- BUTTONS
// =============================================

function changePWM(ch,amount) {

  let newValue = pwm[ch] + amount;

  if(newValue < 500)
    newValue = 500;

  if(newValue > 2500)
    newValue = 2500;

  setPWM(ch,newValue);
}


// =============================================
// SAVE CALIBRATION VALUE
// =============================================

function saveValue(ch,type) {

  if(type == "MIN")
    document.getElementById("min"+ch).innerHTML = pwm[ch];

  if(type == "HOME")
    document.getElementById("home"+ch).innerHTML = pwm[ch];

  if(type == "MAX")
    document.getElementById("max"+ch).innerHTML = pwm[ch];
}


// =============================================
// CENTER ALL
// =============================================

function centerAll() {

  channels.forEach(ch => {

    setPWM(ch,1500);

  });
}


// Build page

createControls();

</script>

</body>
</html>

)rawliteral";


// =====================================================
// CHECK IF CHANNEL IS ALLOWED
// =====================================================

bool channelAllowed(int ch) {

  for(int i = 0; i < NUM_CHANNELS; i++) {

    if(allowedChannels[i] == ch)
      return true;

  }

  return false;
}


// =====================================================
// HANDLE SERVO COMMAND
// =====================================================

void handleServo() {

  if(!server.hasArg("ch") || !server.hasArg("pulse")) {

    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  int ch = server.arg("ch").toInt();
  int pulse = server.arg("pulse").toInt();


  // Safety check

  if(!channelAllowed(ch)) {

    server.send(400, "text/plain", "Channel not allowed");
    return;
  }


  if(pulse < 500 || pulse > 2500) {

    server.send(400, "text/plain", "PWM out of range");
    return;
  }


  // MOVE SERVO

  pca.writeMicroseconds(ch,pulse);


  Serial.print("CH");
  Serial.print(ch);

  Serial.print(" = ");

  Serial.print(pulse);

  Serial.println(" us");


  server.send(200,"text/plain","OK");
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  Wire.begin(21,22);


  // PCA9685

  if(!pca.begin()) {

    Serial.println("PCA9685 NOT FOUND!");

    while(1);
  }


  pca.setPWMFreq(50);

  delay(500);


  Serial.println("PCA9685 FOUND");


  // -----------------------------------------
  // START ALL SIX SERVOS AT 1500 us
  // -----------------------------------------

  for(int i = 0; i < NUM_CHANNELS; i++) {

    pca.writeMicroseconds(
      allowedChannels[i],
      1500
    );

  }


  // -----------------------------------------
  // WIFI ACCESS POINT
  // -----------------------------------------

  WiFi.softAP(ssid,password);


  Serial.println();
  Serial.println("===============================");
  Serial.println(" 6CH VIRTUAL SERVO CALIBRATOR");
  Serial.println("===============================");

  Serial.print("WiFi: ");
  Serial.println(ssid);

  Serial.print("Password: ");
  Serial.println(password);

  Serial.print("Open browser: http://");
  Serial.println(WiFi.softAPIP());


  // Website

  server.on("/", []() {

    server.send(
      200,
      "text/html",
      webpage
    );

  });


  server.on("/set",handleServo);


  server.begin();


  Serial.println();
  Serial.println("SERVER READY");
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  server.handleClient();

}