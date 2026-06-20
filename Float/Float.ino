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

// Pending one-shot pressure read (deferred to loop to avoid I2C races)
volatile bool pendingPressureRead = false;

// ── Depth-profile mission state machine ─────────────────────────────────────
enum ProfileState { PROFILE_IDLE, PROFILE_DESCEND, PROFILE_HOLD,
                    PROFILE_ASCEND, PROFILE_COMPLETE };

ProfileState  profileState      = PROFILE_IDLE;
float         targetDepthFt     = 0.0;
unsigned long holdTimeMs        = 0;
unsigned long phaseStartMs      = 0;

float         profMaxPressure   = 0.0;
float         profAvgPressure   = 0.0;
float         profMaxDepthFt    = 0.0;
float         profPressureSum   = 0.0;
int           profSampleCount   = 0;

const unsigned long MAX_DESCEND_MS = 60000;   // 60 s safety
const unsigned long MAX_ASCEND_MS  = 60000;
const float SURFACE_DEPTH_FT       = 0.5;     // depth considered "back at surface"
const float DEPTH_TOLERANCE_FT     = 0.3;
const int   PROFILE_MOTOR_SPEED    = 200;

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
  float depthFeet = sensor.depth() * 3.28084;
  String msg = "SENSOR:";
  msg += String(sensor.pressure(), 2); msg += ":";
  msg += String(depthFeet, 2);         msg += ":";
  msg += currentMotorState;
  sendMsg(msg);
}

// ── Profile mission ──────────────────────────────────────────────────────────
void startProfile(float depthFt, unsigned long holdSec) {
  if (profileState != PROFILE_IDLE) { sendMsg("PROFILE_ERR:busy"); return; }
  if (depthFt <= 0 || depthFt > 100) { sendMsg("PROFILE_ERR:bad_depth"); return; }

  targetDepthFt    = depthFt;
  holdTimeMs       = holdSec * 1000UL;
  profMaxPressure  = 0.0;
  profMaxDepthFt   = 0.0;
  profPressureSum  = 0.0;
  profSampleCount  = 0;
  profAvgPressure  = 0.0;

  profileState = PROFILE_DESCEND;
  phaseStartMs = millis();
  motorForward(PROFILE_MOTOR_SPEED);

  Serial.printf("PROFILE START: target=%.2fft hold=%lus\n", depthFt, holdSec);
  sendMsg("PROFILE_START:" + String(depthFt, 2) + ":" + String(holdSec) + "s");
}

void updateProfile() {
  sensor.read();
  float depthFeet = sensor.depth() * 3.28084;
  float pressure  = sensor.pressure();
  unsigned long now = millis();

  switch (profileState) {
    case PROFILE_DESCEND: {
      if (depthFeet > profMaxDepthFt)  profMaxDepthFt  = depthFeet;
      if (pressure  > profMaxPressure) profMaxPressure = pressure;

      if (depthFeet >= targetDepthFt) {
        motorStop();
        profileState  = PROFILE_HOLD;
        phaseStartMs  = now;
        sendMsg("PROFILE_HOLD:reached " + String(depthFeet, 2) + "ft");
      } else if (now - phaseStartMs > MAX_DESCEND_MS) {
        motorStop();
        sendMsg("PROFILE_WARN:descend_timeout at " + String(depthFeet, 2) + "ft");
        profileState  = PROFILE_HOLD;
        phaseStartMs  = now;
      }
      break;
    }
    case PROFILE_HOLD: {
      profPressureSum += pressure;
      profSampleCount++;
      if (pressure > profMaxPressure) profMaxPressure = pressure;

      if (now - phaseStartMs >= holdTimeMs) {
        if (profSampleCount > 0)
          profAvgPressure = profPressureSum / profSampleCount;
        motorReverse(PROFILE_MOTOR_SPEED);
        profileState  = PROFILE_ASCEND;
        phaseStartMs  = now;
        sendMsg("PROFILE_ASCEND:hold_complete");
      }
      break;
    }
    case PROFILE_ASCEND: {
      if (depthFeet <= SURFACE_DEPTH_FT) {
        motorStop();
        String result = "PROFILE_RESULT:";
        result += String(profAvgPressure, 2); result += ":";
        result += String(profMaxPressure, 2); result += ":";
        result += String(profMaxDepthFt, 2);
        sendMsg(result);
        Serial.println(result);
        profileState = PROFILE_IDLE;
      } else if (now - phaseStartMs > MAX_ASCEND_MS) {
        motorStop();
        sendMsg("PROFILE_ERR:ascend_timeout");
        profileState = PROFILE_IDLE;
      }
      break;
    }
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

  // PROFILE:<depth_ft>:<hold_sec>
  if (cmd.startsWith("PROFILE:")) {
    sendAck(cmd);
    String params = cmd.substring(8);
    int sep = params.indexOf(':');
    if (sep < 0) { sendMsg("PROFILE_ERR:bad_format"); return; }
    float d = params.substring(0, sep).toFloat();
    unsigned long t = (unsigned long)params.substring(sep + 1).toInt();
    startProfile(d, t);
    return;
  }
  if (cmd == "PROFILE_ABORT") { sendAck(cmd); abortProfile(); return; }

  if (cmd == "STOP")     { sendAck(cmd); abortProfile(); motorStop();     return; }
  if (cmd == "FORWARD")  { sendAck(cmd); motorForward(200);               return; }
  if (cmd == "REVERSE")  { sendAck(cmd); motorReverse(200);               return; }

  if (cmd == "GET_PRESSURE") { sendAck(cmd); pendingPressureRead = true;  return; }
  if (cmd == "STREAM_ON")    { sendAck(cmd); streamMode = true;           return; }
  if (cmd == "STREAM_OFF")   { sendAck(cmd); streamMode = false;          return; }

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
  Serial.print("My MAC: "); Serial.println(WiFi.macAddress());
  sendMsg("FLOAT_READY");
}

void loop() {
  // Deferred one-shot pressure read (safe I2C context)
  if (pendingPressureRead) {
    pendingPressureRead = false;
    sendSensorData();
  }

  if (profileState != PROFILE_IDLE) {
    updateProfile();
    delay(100);
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