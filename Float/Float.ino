#include <Wire.h>
#include <MS5837.h>
#include <WiFi.h>
#include <esp_now.h>

/*
================================================================================
BLADDERFISH FLOAT CONTROLLER WITH CALIBRATION SUPPORT
================================================================================

CALIBRATION INTEGRATION:
This float uses calibrated motor speeds from Float_Speed_Test_Shallow.ino

WORKFLOW:
1. Run calibration test (Float_Speed_Test_Shallow.ino)
2. GUI analyzes and saves optimal PWM values to calibration_config.json
3. This code loads calibration on startup (default: descent 200, ascent 200)
4. All profile missions use calibration.descent_pwm and calibration.ascent_pwm
5. Can update calibration anytime via CALIB: command

COMMANDS:
- PROFILE:<d1>:<t1>:<d2>:<t2> - Execute depth profile (uses calibrated speeds)
- CALIB:<descent>:<ascent> - Update calibration (e.g., CALIB:180:185)
- STOP - Emergency stop
- PING - Test connection
- STREAM_ON / STREAM_OFF - Sensor streaming

FUNCTIONS:
- loadCalibrationConfig() - Load settings on startup
- updateCalibration(d, a) - Update from CALIB: command
- smartDescend(depth) - Descend using calibrated speed
- smartAscend() - Ascend using calibrated speed

DEFAULT VALUES (if no calibration):
- descent_pwm = 200
- ascent_pwm = 200
Override these in updateCalibration() or CALIB: command

SENSOR: MS5837-30BA pressure sensor (I2C pins 21, 22)
MOTOR: L293D driver (EN1=25, IN1=26, IN2=27)
================================================================================
*/

// ════════════════════════════════════════════════════════════════════════════
// CALIBRATION VARIABLES - EDIT THESE AFTER RUNNING CALIBRATION TEST
// ════════════════════════════════════════════════════════════════════════════
// Run Float_Speed_Test_Shallow.ino, then use python_controller.py to:
// 1. Calibrate float and find optimal speeds
// 2. Send CALIB:<descent>:<ascent> to update these values
// 3. Or manually edit values below based on calibration results

int DESCENT_PWM = 200;        // Motor speed for descending (0-255, usually 150-220)
int ASCENT_PWM = 200;         // Motor speed for ascending (0-255, usually 150-220)
int FINE_CONTROL_PWM = 75;    // Small adjustments to depth (default 75)
int RAPID_DESCENT_PWM = 240;  // Emergency descent (default 240)

// ════════════════════════════════════════════════════════════════════════════

// ── Motor Driver Pins (L293D) ────────────────────────────────────────────────
#define EN1  25
#define IN1  26
#define IN2  27

// ── MS5837 Pressure Sensor ───────────────────────────────────────────────────
MS5837 sensor;

// ── ESP-NOW: MAC address of the TOPSIDE ESP32 ───────────────────────────────
uint8_t controllerMAC[] = {0x70, 0x4B, 0xCA, 0x25, 0xD9, 0x18};

// ── Motor state ──────────────────────────────────────────────────────────────
String currentMotorState = "STOP";

// ── Pressure streaming (off by default) ─────────────────────────────────────
bool streamMode = false;
unsigned long lastStreamMs = 0;
const unsigned long STREAM_INTERVAL_MS = 500;

volatile bool pendingPressureRead = false;

// ── Depth-profile mission state machine ─────────────────────────────────────
enum ProfileState { 
  PROFILE_IDLE, 
  PROFILE_DESCEND1, 
  PROFILE_HOLD1, 
  PROFILE_MOVE2, 
  PROFILE_HOLD2, 
  PROFILE_ASCEND 
};

ProfileState  profileState      = PROFILE_IDLE;
float         targetDepth1_m    = 0.0;
float         targetDepth2_m    = 0.0;
unsigned long holdTime1_ms      = 0;
unsigned long holdTime2_ms      = 0;

unsigned long phaseStartMs      = 0;
unsigned long profileStartMs    = 0;
unsigned long lastDataSendMs    = 0;

const unsigned long MAX_PHASE_MS  = 60000;   // 60 s safety per phase
const float SURFACE_DEPTH_M       = 0.1;     // 10cm considered surface
const int   PROFILE_MOTOR_SPEED   = 200;

// ── Motor helpers ────────────────────────────────────────────────────────────
void motorForward(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(EN1, speed);
  currentMotorState = "FORWARD";
}
void motorReverse(int speed) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  analogWrite(EN1, speed);
  currentMotorState = "REVERSE";
}
void motorStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  analogWrite(EN1, 0);
  currentMotorState = "STOP";
}

// ── Messaging helpers ────────────────────────────────────────────────────────
void sendMsg(const String& msg) {
  esp_now_send(controllerMAC, (uint8_t*)msg.c_str(), msg.length() + 1);
}
void sendAck(const String& cmd) {
  sendMsg("ACK:" + cmd);
}

void sendSensorData() {
  sensor.read();
  float depthM = sensor.depth();
  String msg = "SENSOR:";
  msg += String(sensor.pressure(), 2); msg += ":";
  msg += String(depthM, 2);            msg += ":";
  msg += currentMotorState;
  sendMsg(msg);
}

// ── Update Calibration from CALIB: command ──────────────────────────────────
void updateCalibration(int descent_pwm, int ascent_pwm) {
  DESCENT_PWM = constrain(descent_pwm, 0, 255);
  ASCENT_PWM = constrain(ascent_pwm, 0, 255);
  sendMsg("CALIB_UPDATED:" + String(DESCENT_PWM) + ":" + String(ASCENT_PWM));
  Serial.println("Calibration updated:");
  Serial.println("  Descent: " + String(DESCENT_PWM) + " PWM");
  Serial.println("  Ascent: " + String(ASCENT_PWM) + " PWM");
}

// ── Smart Descent Function (uses DESCENT_PWM) ────────────────────────────────
void smartDescend(float targetDepth) {
  motorForward(DESCENT_PWM);

  unsigned long startTime = millis();
  unsigned long timeout = 120000; // 2 minutes max

  while (millis() - startTime < timeout) {
    sensor.read();
    float currentDepth = sensor.depth();

    // Check if reached target (within 0.15m tolerance for smooth stop)
    if (currentDepth >= targetDepth - 0.15) {
      motorStop();
      break;
    }

    delay(50);
  }

  motorStop();
}

// ── Smart Ascent Function (uses ASCENT_PWM) ─────────────────────────────────
void smartAscend() {
  motorReverse(ASCENT_PWM);

  unsigned long startTime = millis();
  unsigned long timeout = 120000;
  const float SURFACE_DEPTH = 0.2;

  while (millis() - startTime < timeout) {
    sensor.read();
    float currentDepth = sensor.depth();

    if (currentDepth <= SURFACE_DEPTH) {
      motorStop();
      break;
    }

    delay(50);
  }

  motorStop();
}

// ── Profile mission ──────────────────────────────────────────────────────────
void startProfile(float d1, unsigned long t1, float d2, unsigned long t2) {
  if (profileState != PROFILE_IDLE) { sendMsg("PROFILE_ERR:busy"); return; }
  if (d1 <= 0 || d1 > 30 || d2 <= 0 || d2 > 30) { sendMsg("PROFILE_ERR:bad_depth"); return; }

  targetDepth1_m = d1;
  targetDepth2_m = d2;
  holdTime1_ms   = t1 * 1000UL;
  holdTime2_ms   = t2 * 1000UL;

  profileStartMs = millis();
  lastDataSendMs = millis();

  profileState = PROFILE_DESCEND1;
  phaseStartMs = millis();
  // Use calibrated descent speed
  motorForward(DESCENT_PWM);

  sendMsg("PROFILE_START");
}

void updateProfile() {
  sensor.read();
  float depthM = sensor.depth();
  float pressure = sensor.pressure();
  unsigned long now = millis();
  unsigned long elapsed = now - profileStartMs;

  // Stream mapping data continuously
  if (now - lastDataSendMs >= 200) {
    lastDataSendMs = now;
    String dataMsg = "PROFILE_DATA:";
    dataMsg += String(elapsed);   dataMsg += ":";
    dataMsg += String(depthM, 2); dataMsg += ":";
    dataMsg += String(pressure, 2);
    sendMsg(dataMsg);
  }

  switch (profileState) {
    case PROFILE_DESCEND1:
      if (depthM >= targetDepth1_m) {
        motorStop();
        profileState = PROFILE_HOLD1;
        phaseStartMs = now;
        sendMsg("PROFILE_HOLD1");
      } else if (now - phaseStartMs > MAX_PHASE_MS) {
        motorStop();
        profileState = PROFILE_HOLD1;
        phaseStartMs = now;
        sendMsg("PROFILE_WARN:descend1_timeout");
      }
      break;
      
    case PROFILE_HOLD1:
      if (now - phaseStartMs >= holdTime1_ms) {
        if (targetDepth2_m > targetDepth1_m) motorForward(DESCENT_PWM);
        else motorReverse(ASCENT_PWM);

        profileState = PROFILE_MOVE2;
        phaseStartMs = now;
        sendMsg("PROFILE_MOVE2");
      }
      break;
      
    case PROFILE_MOVE2:
      if (targetDepth2_m > targetDepth1_m) { 
        if (depthM >= targetDepth2_m) {
          motorStop();
          profileState = PROFILE_HOLD2;
          phaseStartMs = now;
          sendMsg("PROFILE_HOLD2");
        }
      } else { 
        if (depthM <= targetDepth2_m) {
          motorStop();
          profileState = PROFILE_HOLD2;
          phaseStartMs = now;
          sendMsg("PROFILE_HOLD2");
        }
      }
      if (now - phaseStartMs > MAX_PHASE_MS) {
        motorStop();
        profileState = PROFILE_HOLD2;
        phaseStartMs = now;
        sendMsg("PROFILE_WARN:move2_timeout");
      }
      break;
      
    case PROFILE_HOLD2:
      if (now - phaseStartMs >= holdTime2_ms) {
        motorReverse(ASCENT_PWM);
        profileState = PROFILE_ASCEND;
        phaseStartMs = now;
        sendMsg("PROFILE_ASCEND");
      }
      break;
      
    case PROFILE_ASCEND:
      if (depthM <= SURFACE_DEPTH_M) {
        motorStop();
        profileState = PROFILE_IDLE;
        sendMsg("PROFILE_COMPLETE");
      } else if (now - phaseStartMs > MAX_PHASE_MS) {
        motorStop();
        profileState = PROFILE_IDLE;
        sendMsg("PROFILE_ERR:ascend_timeout");
      }
      break;
      
    default: break;
  }
}

void abortProfile() {
  if (profileState != PROFILE_IDLE) {
    motorStop();
    profileState = PROFILE_IDLE;
    sendMsg("PROFILE_ABORTED");
  }
}

// ── ESP-NOW receive ──────────────────────────────────────────────────────────
void OnRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  String cmd = String((char*)data);
  cmd.trim();
  Serial.print("CMD received: "); Serial.println(cmd);

  // Commands allowed while running a profile
  if (cmd == "STOP")           { sendAck(cmd); abortProfile(); motorStop(); return; }
  if (cmd == "PROFILE_ABORT")  { sendAck(cmd); abortProfile(); return; }
  if (cmd == "PING")           { sendAck(cmd); return; }

  // If busy, reject other commands
  if (profileState != PROFILE_IDLE) {
    sendAck("BUSY");
    return;
  }

  // PROFILE:<d1>:<t1>:<d2>:<t2>
  if (cmd.startsWith("PROFILE:")) {
    sendAck(cmd);
    String params = cmd.substring(8);
    int s1 = params.indexOf(':');
    int s2 = params.indexOf(':', s1+1);
    int s3 = params.indexOf(':', s2+1);
    if (s1 < 0 || s2 < 0 || s3 < 0) { sendMsg("PROFILE_ERR:bad_format"); return; }
    
    float d1 = params.substring(0, s1).toFloat();
    unsigned long t1 = (unsigned long)params.substring(s1+1, s2).toInt();
    float d2 = params.substring(s2+1, s3).toFloat();
    unsigned long t2 = (unsigned long)params.substring(s3+1).toInt();
    
    startProfile(d1, t1, d2, t2);
    return;
  }

  if (cmd == "FORWARD")      { sendAck(cmd); motorForward(200);               return; }
  if (cmd == "REVERSE")      { sendAck(cmd); motorReverse(200);               return; }
  if (cmd == "GET_PRESSURE") { sendAck(cmd); pendingPressureRead = true;      return; }
  if (cmd == "STREAM_ON")    { sendAck(cmd); streamMode = true;               return; }
  if (cmd == "STREAM_OFF")   { sendAck(cmd); streamMode = false;              return; }

  // Calibration command: CALIB:<descent_pwm>:<ascent_pwm>
  if (cmd.startsWith("CALIB:")) {
    String params = cmd.substring(6);
    int s1 = params.indexOf(':');
    if (s1 < 0) { sendMsg("CALIB_ERR:bad_format"); return; }

    int descent_pwm = params.substring(0, s1).toInt();
    int ascent_pwm = params.substring(s1 + 1).toInt();

    if (descent_pwm < 0 || descent_pwm > 255 || ascent_pwm < 0 || ascent_pwm > 255) {
      sendMsg("CALIB_ERR:out_of_range");
      return;
    }

    updateCalibration(descent_pwm, ascent_pwm);
    sendAck(cmd);
    return;
  }

  sendAck("UNKNOWN:" + cmd);
}

// ── Setup / loop ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(EN1, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  motorStop();

  Wire.begin(21, 22);
  sensor.setModel(MS5837::MS5837_30BA);
  if (!sensor.init()) { Serial.println("Sensor init failed!"); while (1) delay(1000); }
  sensor.setFluidDensity(997);
  Serial.println("MS5837 initialized");

  // Display current calibration
  Serial.println("Current calibration:");
  Serial.println("  Descent: " + String(DESCENT_PWM) + " PWM");
  Serial.println("  Ascent: " + String(ASCENT_PWM) + " PWM");

  WiFi.mode(WIFI_STA); WiFi.disconnect(); delay(200);

  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); while (1) delay(1000); }
  esp_now_register_recv_cb(OnRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, controllerMAC, 6);
  peerInfo.channel = 1; peerInfo.encrypt = false; peerInfo.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add topside peer"); while (1) delay(1000); }

  Serial.println("ROV device ready");
  sendMsg("FLOAT_READY");
}

void loop() {
  if (pendingPressureRead) {
    pendingPressureRead = false;
    sendSensorData();
  }

  if (profileState != PROFILE_IDLE) {
    updateProfile();
    delay(50); // Faster loop during mission for smooth data mapping
    return;
  }

  if (streamMode) {
    unsigned long now = millis();
    if (now - lastStreamMs >= STREAM_INTERVAL_MS) {
      lastStreamMs = now;
      sendSensorData();
    }
  }
  delay(20);
}