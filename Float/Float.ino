#include <Wire.h>
#include <MS5837.h>
#include <WiFi.h>
#include <esp_now.h>

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
  PROFILE_CALIBRATE_NB,
  PROFILE_DESCEND1,
  PROFILE_HOLD1,
  PROFILE_MOVE2,
  PROFILE_HOLD2,
  PROFILE_ASCEND,
  PROFILE_RETURN_SURFACE
};

ProfileState  profileState      = PROFILE_IDLE;
float         targetDepth1_m    = 0.0;
float         targetDepth2_m    = 0.0;
unsigned long holdTime1_ms      = 0;
unsigned long holdTime2_ms      = 0;

unsigned long phaseStartMs      = 0;
unsigned long profileStartMs    = 0;
unsigned long lastDataSendMs    = 0;

// ── Neutral buoyancy configuration ──────────────────────────────────────────
unsigned long neutral_buoyancy_pump_ms = 2000;  // Pump duration (ms) to achieve neutral buoyancy
float descent_offset_m                 = 0.3;   // Extra depth margin before reversing pump
float depth_tolerance_m                = 0.15;  // Acceptable depth range around target

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

// ── Profile mission ──────────────────────────────────────────────────────────
void startProfile(float d1, unsigned long t1, float d2, unsigned long t2,
                  unsigned long nb_pump_ms = 0, float offset = -1.0, float tol = -1.0) {
  if (profileState != PROFILE_IDLE) { sendMsg("PROFILE_ERR:busy"); return; }
  if (d1 <= 0 || d1 > 30 || d2 <= 0 || d2 > 30) { sendMsg("PROFILE_ERR:bad_depth"); return; }

  // Update NB parameters if provided
  if (nb_pump_ms > 0) neutral_buoyancy_pump_ms = nb_pump_ms;
  if (offset >= 0) descent_offset_m = offset;
  if (tol >= 0) depth_tolerance_m = tol;

  targetDepth1_m = d1;
  targetDepth2_m = d2;
  holdTime1_ms   = t1 * 1000UL;
  holdTime2_ms   = t2 * 1000UL;

  profileStartMs = millis();
  lastDataSendMs = millis();
  phaseStartMs = millis();

  // Start from neutral buoyancy calibration if at surface
  sensor.read();
  if (sensor.depth() <= SURFACE_DEPTH_M) {
    profileState = PROFILE_CALIBRATE_NB;
    motorForward(PROFILE_MOTOR_SPEED);
    sendMsg("PROFILE_CALIBRATE_NB");
  } else {
    // Already underwater, skip calibration
    profileState = PROFILE_DESCEND1;
    motorForward(PROFILE_MOTOR_SPEED);
    sendMsg("PROFILE_START");
  }
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
    case PROFILE_CALIBRATE_NB: {
      // Calibrate neutral buoyancy at surface by pumping for specified duration
      unsigned long pumpElapsed = now - phaseStartMs;
      if (pumpElapsed >= neutral_buoyancy_pump_ms) {
        motorStop();
        profileState = PROFILE_DESCEND1;
        phaseStartMs = now;
        motorForward(PROFILE_MOTOR_SPEED);
        sendMsg("PROFILE_START");
      }
      break;
    }

    case PROFILE_DESCEND1: {
      // Intelligent descent with buoyancy correction
      // Start pumping out water when approaching target to maintain controlled descent
      float startEmptyingDepth = targetDepth1_m - descent_offset_m;

      if (depthM >= targetDepth1_m - depth_tolerance_m && depthM <= targetDepth1_m + depth_tolerance_m) {
        // Within tolerance of target
        motorStop();
        profileState = PROFILE_HOLD1;
        phaseStartMs = now;
        sendMsg("PROFILE_HOLD1");
      } else if (depthM >= startEmptyingDepth && currentMotorState != "REVERSE") {
        // Approaching target: start emptying to slow descent and return toward NB
        motorReverse(PROFILE_MOTOR_SPEED);
        sendMsg("PROFILE_DBG:start_emptying");
      } else if (depthM < startEmptyingDepth && currentMotorState != "FORWARD") {
        // Still above "start emptying" depth: pump in to continue descending
        motorForward(PROFILE_MOTOR_SPEED);
      } else if (now - phaseStartMs > MAX_PHASE_MS) {
        motorStop();
        profileState = PROFILE_HOLD1;
        phaseStartMs = now;
        sendMsg("PROFILE_WARN:descend1_timeout");
      }
      break;
    }
      
    case PROFILE_HOLD1:
      if (now - phaseStartMs >= holdTime1_ms) {
        if (targetDepth2_m > targetDepth1_m) motorForward(PROFILE_MOTOR_SPEED);
        else motorReverse(PROFILE_MOTOR_SPEED);
        
        profileState = PROFILE_MOVE2;
        phaseStartMs = now;
        sendMsg("PROFILE_MOVE2");
      }
      break;
      
    case PROFILE_MOVE2: {
      // Move to second target with buoyancy-aware control
      bool isDescending = targetDepth2_m > targetDepth1_m;
      bool nearTarget = false;

      if (isDescending) {
        // Descending to deeper target
        float startEmptyingDepth = targetDepth2_m - descent_offset_m;
        nearTarget = (depthM >= targetDepth2_m - depth_tolerance_m && depthM <= targetDepth2_m + depth_tolerance_m);

        if (nearTarget) {
          motorStop();
        } else if (depthM >= startEmptyingDepth && currentMotorState != "REVERSE") {
          motorReverse(PROFILE_MOTOR_SPEED);
        } else if (depthM < startEmptyingDepth && currentMotorState != "FORWARD") {
          motorForward(PROFILE_MOTOR_SPEED);
        }
      } else {
        // Ascending to shallower target
        float stopAscendingDepth = targetDepth2_m + descent_offset_m;
        nearTarget = (depthM >= targetDepth2_m - depth_tolerance_m && depthM <= targetDepth2_m + depth_tolerance_m);

        if (nearTarget) {
          motorStop();
        } else if (depthM <= stopAscendingDepth && currentMotorState != "FORWARD") {
          motorForward(PROFILE_MOTOR_SPEED);
        } else if (depthM > stopAscendingDepth && currentMotorState != "REVERSE") {
          motorReverse(PROFILE_MOTOR_SPEED);
        }
      }

      if (nearTarget) {
        profileState = PROFILE_HOLD2;
        phaseStartMs = now;
        sendMsg("PROFILE_HOLD2");
      } else if (now - phaseStartMs > MAX_PHASE_MS) {
        motorStop();
        profileState = PROFILE_HOLD2;
        phaseStartMs = now;
        sendMsg("PROFILE_WARN:move2_timeout");
      }
      break;
    }
      
    case PROFILE_HOLD2:
      if (now - phaseStartMs >= holdTime2_ms) {
        motorReverse(PROFILE_MOTOR_SPEED);
        profileState = PROFILE_ASCEND;
        phaseStartMs = now;
        sendMsg("PROFILE_ASCEND");
      }
      break;

    case PROFILE_ASCEND:
      // Ascend to intermediate depth, then transition to return-to-surface
      if (depthM <= SURFACE_DEPTH_M) {
        profileState = PROFILE_RETURN_SURFACE;
        phaseStartMs = now;
        sendMsg("PROFILE_RETURN_SURFACE");
      } else if (now - phaseStartMs > MAX_PHASE_MS) {
        motorStop();
        profileState = PROFILE_RETURN_SURFACE;
        phaseStartMs = now;
        motorReverse(PROFILE_MOTOR_SPEED);
        sendMsg("PROFILE_WARN:ascend_slow");
      }
      break;

    case PROFILE_RETURN_SURFACE:
      // Return to surface for data transmission
      if (depthM <= SURFACE_DEPTH_M) {
        motorStop();
        profileState = PROFILE_IDLE;
        sendMsg("PROFILE_COMPLETE");
      } else if (now - phaseStartMs > MAX_PHASE_MS) {
        motorStop();
        profileState = PROFILE_IDLE;
        sendMsg("PROFILE_ERR:return_timeout");
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

  // SET_NB:<pump_ms> - Set neutral buoyancy pump duration (milliseconds)
  if (cmd.startsWith("SET_NB:")) {
    unsigned long pump_ms = (unsigned long)cmd.substring(7).toInt();
    if (pump_ms > 0 && pump_ms < 60000) {
      neutral_buoyancy_pump_ms = pump_ms;
      sendAck(cmd);
      sendMsg("NB_SET:" + String(neutral_buoyancy_pump_ms) + "ms");
    } else {
      sendMsg("NB_ERR:invalid_pump_time");
    }
    return;
  }

  // PROFILE:<d1>:<t1>:<d2>:<t2> or PROFILE:<d1>:<t1>:<d2>:<t2>:<nb_pump_ms>:<offset>:<tol>
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

    // Check for optional NB parameters (pump_ms, offset, tolerance)
    int s4 = params.indexOf(':', s3+1);
    if (s4 > 0) {
      int s5 = params.indexOf(':', s4+1);
      if (s5 > 0) {
        unsigned long nb_pump = (unsigned long)params.substring(s3+1, s4).toInt();
        float offset = params.substring(s4+1, s5).toFloat();
        float tol = params.substring(s5+1).toFloat();
        startProfile(d1, t1, d2, t2, nb_pump, offset, tol);
      } else {
        startProfile(d1, t1, d2, t2);
      }
    } else {
      startProfile(d1, t1, d2, t2);
    }
    return;
  }

  if (cmd == "FORWARD")      { sendAck(cmd); motorForward(200);               return; }
  if (cmd == "REVERSE")      { sendAck(cmd); motorReverse(200);               return; }
  if (cmd == "GET_PRESSURE") { sendAck(cmd); pendingPressureRead = true;      return; }
  if (cmd == "STREAM_ON")    { sendAck(cmd); streamMode = true;               return; }
  if (cmd == "STREAM_OFF")   { sendAck(cmd); streamMode = false;              return; }

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