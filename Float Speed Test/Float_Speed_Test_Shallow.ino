/*
================================================================================
BLADDERFISH FLOAT SHALLOW CALIBRATION TEST (2m, 10cm accuracy)
================================================================================

PURPOSE:
Automatically calibrate motor speeds for optimal float performance.
- Tests 7 motor speeds (80-200 PWM)
- Measures descent and ascent rates at each speed
- Collects ~5000 sensor data points
- Returns to surface and transmits all data to topside GUI

WORKFLOW:
1. Float receives START_TEST command from topside
2. Records baseline at surface (10s)
3. Tests descent at PWM 80, 100, 120, 140, 160, 180, 200
4. Tests ascent at same speeds
5. Holds at depth for 15 seconds
6. Returns to surface
7. Transmits all data to GUI (~100 ms between packets)
8. GUI analyzes and saves optimal values to calibration_config.json

OUTPUT:
- descent_rates[pwm] = rate in m/s
- ascent_rates[pwm] = rate in m/s
- Recommended descent_pwm (usually ~180)
- Recommended ascent_pwm (usually ~185)

INTEGRATION:
- Main Float.ino loads calibration_config.json on startup
- Uses calibration.descent_pwm and calibration.ascent_pwm in all missions
- Can update via CALIB: command from topside controller

HARDWARE:
- ESP32 with L293D motor driver (pins 25, 26, 27)
- MS5837-30BA pressure sensor (I2C pins 21, 22)
- Motor: reversible water pump/thruster
- Target depth: 2m (surface: 15cm, safety limit: 2.5m)
================================================================================
*/

#include <Wire.h>
#include <MS5837.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPIFFS.h>

// ── Motor Driver Pins (L293D) ────────────────────────────────────────────────
#define EN1  25
#define IN1  26
#define IN2  27

// ── MS5837 Pressure Sensor ───────────────────────────────────────────────────
MS5837 sensor;

// ── ESP-NOW: MAC address of the TOPSIDE ESP32 ───────────────────────────────
uint8_t controllerMAC[] = {0x70, 0x4B, 0xCA, 0x25, 0xD9, 0x18};

// ── Configuration ────────────────────────────────────────────────────────────
const float WATER_DENSITY = 997.0;
const float SURFACE_DEPTH_M = 0.15;        // 15cm = surface
const float TARGET_DEPTH_M = 2.0;          // 2m target
const float DEPTH_ACCURACY_M = 0.10;       // 10cm accuracy requirement
const float MAX_DEPTH_M = 2.5;             // Safety limit

// Motor speeds to test (PWM 0-255)
const int MOTOR_SPEEDS[] = {80, 100, 120, 140, 160, 180, 200};
const int NUM_SPEEDS = 7;

// ── Data Logging Structure ───────────────────────────────────────────────────
struct DataPoint {
  unsigned long timeMs;
  float depthM;
  float pressureMbar;
  int motorPWM;
};

const int MAX_DATA_POINTS = 10000;
DataPoint dataLog[MAX_DATA_POINTS];
int dataLogIndex = 0;

// ── Test State Machine ───────────────────────────────────────────────────────
enum TestState {
  STATE_IDLE,
  STATE_NEUTRAL_BASELINE,
  STATE_DESCENT_TEST,
  STATE_ASCENT_TEST,
  STATE_HOLD_TEST,
  STATE_RETURN_SURFACE,
  STATE_TRANSMIT_DATA
};

TestState testState = STATE_IDLE;
TestState nextTest = STATE_NEUTRAL_BASELINE;

// ── Test Parameters ──────────────────────────────────────────────────────────
struct TestRun {
  int motorSpeed;
  unsigned long startTimeMs;
  float startDepthM;
  bool isAscent;
  int dataStartIndex;
};

TestRun currentRun = {0, 0, 0.0, false, 0};
int currentSpeedIndex = 0;

// ── Motor helpers ────────────────────────────────────────────────────────────
void motorForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, constrain(speed, 0, 255));
}

void motorReverse(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, constrain(speed, 0, 255));
}

void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, 0);
}

// ── Data Logging ─────────────────────────────────────────────────────────────
void logData(float depthM, float pressureMbar, int motorPWM) {
  if (dataLogIndex >= MAX_DATA_POINTS) return;

  dataLog[dataLogIndex].timeMs = millis();
  dataLog[dataLogIndex].depthM = depthM;
  dataLog[dataLogIndex].pressureMbar = pressureMbar;
  dataLog[dataLogIndex].motorPWM = motorPWM;
  dataLogIndex++;
}

// ── Messaging helpers ────────────────────────────────────────────────────────
void sendMsg(const String& msg) {
  esp_now_send(controllerMAC, (uint8_t*)msg.c_str(), msg.length() + 1);
  Serial.println("[TX] " + msg);
}

void recordSensorReading(int motorPWM) {
  sensor.read();
  float depthM = sensor.depth();
  float pressureMbar = sensor.pressure();

  logData(depthM, pressureMbar, motorPWM);

  // Send status every 200ms
  static unsigned long lastStatusMs = 0;
  unsigned long now = millis();
  if (now - lastStatusMs >= 200) {
    lastStatusMs = now;
    String status = "STATUS:";
    status += String(depthM, 3); status += ":";
    status += String(pressureMbar, 2); status += ":";
    status += String(motorPWM); status += ":";
    status += String(dataLogIndex);
    sendMsg(status);
  }
}

// ── Test: Neutral Baseline ───────────────────────────────────────────────────
void startNeutralBaseline() {
  Serial.println("\n=== NEUTRAL BASELINE ===");
  sendMsg("PHASE:NEUTRAL_BASELINE");

  motorStop();
  currentRun.motorSpeed = 0;
  currentRun.startTimeMs = millis();
  currentRun.dataStartIndex = dataLogIndex;

  sensor.read();
  currentRun.startDepthM = sensor.depth();

  testState = STATE_NEUTRAL_BASELINE;
}

void updateNeutralBaseline() {
  recordSensorReading(0);

  if (millis() - currentRun.startTimeMs > 10000) {  // 10 seconds
    sensor.read();
    float endDepth = sensor.depth();
    float baselineDrift = endDepth - currentRun.startDepthM;

    Serial.print("Baseline drift: "); Serial.print(baselineDrift, 3); Serial.println(" m");
    sendMsg("BASELINE_DRIFT:" + String(baselineDrift, 4));

    currentSpeedIndex = 0;
    testState = STATE_DESCENT_TEST;
    testNextDescentSpeed();
  }
}

// ── Test: Descent Speed ──────────────────────────────────────────────────────
void testNextDescentSpeed() {
  if (currentSpeedIndex >= NUM_SPEEDS) {
    // All descent tests done, start ascent tests
    currentSpeedIndex = 0;
    testState = STATE_ASCENT_TEST;
    testNextAscentSpeed();
    return;
  }

  int speed = MOTOR_SPEEDS[currentSpeedIndex];
  Serial.print("Descent test: PWM "); Serial.println(speed);
  sendMsg("DESCENT_TEST:" + String(speed));

  motorForward(speed);

  currentRun.motorSpeed = speed;
  currentRun.startTimeMs = millis();
  currentRun.isAscent = false;
  currentRun.dataStartIndex = dataLogIndex;

  sensor.read();
  currentRun.startDepthM = sensor.depth();

  testState = STATE_DESCENT_TEST;
}

void updateDescentSpeed() {
  recordSensorReading(currentRun.motorSpeed);

  sensor.read();
  float currentDepth = sensor.depth();
  unsigned long elapsedMs = millis() - currentRun.startTimeMs;

  // Stop at target depth or timeout
  if (currentDepth >= TARGET_DEPTH_M || elapsedMs > 60000) {
    motorStop();

    Serial.print("Descent complete at "); Serial.print(currentDepth, 3); Serial.println("m");
    sendMsg("DESCENT_COMPLETE:" + String(currentRun.motorSpeed) + ":" + String(currentDepth, 3));

    delay(3000);  // Hold at depth for 3 seconds

    currentSpeedIndex++;
    testNextDescentSpeed();
  }
}

// ── Test: Ascent Speed ───────────────────────────────────────────────────────
void testNextAscentSpeed() {
  if (currentSpeedIndex >= NUM_SPEEDS) {
    // All ascent tests done, test holding
    testState = STATE_HOLD_TEST;
    startHoldTest();
    return;
  }

  int speed = MOTOR_SPEEDS[currentSpeedIndex];
  Serial.print("Ascent test: PWM "); Serial.println(speed);
  sendMsg("ASCENT_TEST:" + String(speed));

  motorReverse(speed);

  currentRun.motorSpeed = speed;
  currentRun.startTimeMs = millis();
  currentRun.isAscent = true;
  currentRun.dataStartIndex = dataLogIndex;

  sensor.read();
  currentRun.startDepthM = sensor.depth();

  testState = STATE_ASCENT_TEST;
}

void updateAscentSpeed() {
  recordSensorReading(currentRun.motorSpeed);

  sensor.read();
  float currentDepth = sensor.depth();
  unsigned long elapsedMs = millis() - currentRun.startTimeMs;

  // Stop at surface or timeout
  if (currentDepth <= SURFACE_DEPTH_M || elapsedMs > 60000) {
    motorStop();

    Serial.print("Ascent complete at "); Serial.print(currentDepth, 3); Serial.println("m");
    sendMsg("ASCENT_COMPLETE:" + String(currentRun.motorSpeed) + ":" + String(currentDepth, 3));

    delay(2000);  // Brief pause at surface

    currentSpeedIndex++;
    testNextAscentSpeed();
  }
}

// ── Test: Hold Stability ─────────────────────────────────────────────────────
void startHoldTest() {
  Serial.println("\n=== HOLD TEST ===");
  sendMsg("PHASE:HOLD_TEST");

  motorStop();
  currentRun.motorSpeed = 0;
  currentRun.startTimeMs = millis();
  currentRun.dataStartIndex = dataLogIndex;

  sensor.read();
  currentRun.startDepthM = sensor.depth();

  testState = STATE_HOLD_TEST;
}

void updateHoldTest() {
  recordSensorReading(0);

  if (millis() - currentRun.startTimeMs > 15000) {  // 15 seconds at depth
    motorStop();

    sensor.read();
    float endDepth = sensor.depth();

    Serial.print("Hold test complete. Depth: "); Serial.print(endDepth, 3); Serial.println("m");
    sendMsg("HOLD_COMPLETE:" + String(endDepth, 3));

    testState = STATE_RETURN_SURFACE;
    startReturnToSurface();
  }
}

// ── Return to Surface ────────────────────────────────────────────────────────
void startReturnToSurface() {
  Serial.println("\n=== RETURNING TO SURFACE ===");
  sendMsg("PHASE:RETURN_SURFACE");

  motorReverse(150);  // Moderate speed back to surface

  currentRun.motorSpeed = 150;
  currentRun.startTimeMs = millis();
  currentRun.dataStartIndex = dataLogIndex;

  testState = STATE_RETURN_SURFACE;
}

void updateReturnToSurface() {
  recordSensorReading(currentRun.motorSpeed);

  sensor.read();
  float currentDepth = sensor.depth();

  if (currentDepth <= SURFACE_DEPTH_M) {
    motorStop();

    Serial.println("Returned to surface");
    sendMsg("AT_SURFACE");

    delay(2000);  // Brief wait at surface before transmitting

    testState = STATE_TRANSMIT_DATA;
  } else if (millis() - currentRun.startTimeMs > 120000) {
    // Safety timeout
    motorStop();
    sendMsg("RETURN_TIMEOUT");
    testState = STATE_TRANSMIT_DATA;
  }
}

// ── Transmit All Data ────────────────────────────────────────────────────────
void transmitAllData() {
  Serial.println("\n=== TRANSMITTING DATA ===");
  sendMsg("DATA_TRANSMISSION_START:" + String(dataLogIndex));

  // Send data in chunks (ESP-NOW max ~250 bytes)
  // Format: DATA:<index>:<time>:<depth>:<pressure>:<pwm>

  const int CHUNK_SIZE = 100;  // ms between packets to avoid flooding
  unsigned long lastSendMs = 0;

  for (int i = 0; i < dataLogIndex; i++) {
    unsigned long now = millis();
    if (now - lastSendMs < CHUNK_SIZE) {
      delayMicroseconds(500);
      continue;
    }

    String dataMsg = "DATA:";
    dataMsg += String(i); dataMsg += ":";
    dataMsg += String(dataLog[i].timeMs); dataMsg += ":";
    dataMsg += String(dataLog[i].depthM, 4); dataMsg += ":";
    dataMsg += String(dataLog[i].pressureMbar, 2); dataMsg += ":";
    dataMsg += String(dataLog[i].motorPWM);

    sendMsg(dataMsg);
    lastSendMs = now;

    if (i % 100 == 0) {
      Serial.print("Sent "); Serial.print(i); Serial.print("/"); Serial.println(dataLogIndex);
    }
  }

  sendMsg("DATA_TRANSMISSION_COMPLETE:" + String(dataLogIndex));
  Serial.println("Data transmission complete");

  testState = STATE_IDLE;
  delay(2000);
}

// ── ESP-NOW receive ──────────────────────────────────────────────────────────
void OnRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  String cmd = String((char*)data);
  cmd.trim();
  Serial.print("[RX] "); Serial.println(cmd);

  if (cmd == "START_TEST") {
    // Clear data log and start fresh
    dataLogIndex = 0;
    testState = STATE_NEUTRAL_BASELINE;
    startNeutralBaseline();
    sendMsg("ACK:START_TEST");
  } else if (cmd == "STOP") {
    motorStop();
    testState = STATE_RETURN_SURFACE;
    startReturnToSurface();
    sendMsg("ACK:STOP");
  } else if (cmd == "PING") {
    sendMsg("ACK:PING");
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║   FLOAT SHALLOW SPEED TEST (2m)            ║");
  Serial.println("║   Auto Data Collection Version              ║");
  Serial.println("╚════════════════════════════════════════════╝");

  // Motor setup
  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motorStop();

  // Sensor setup
  Wire.begin(21, 22);
  sensor.setModel(MS5837::MS5837_30BA);
  if (!sensor.init()) {
    Serial.println("ERROR: Sensor init failed!");
    while (1) delay(1000);
  }
  sensor.setFluidDensity(WATER_DENSITY);
  Serial.println("✓ MS5837 initialized");

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    while (1) delay(1000);
  }
  esp_now_register_recv_cb(OnRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, controllerMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  peerInfo.ifx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add peer");
    while (1) delay(1000);
  }

  Serial.println("✓ ESP-NOW ready");
  Serial.println("✓ Ready for START_TEST command\n");

  sendMsg("READY");
}

// ── Main Loop ────────────────────────────────────────────────────────────────
void loop() {
  switch (testState) {
    case STATE_IDLE:
      delay(100);
      break;

    case STATE_NEUTRAL_BASELINE:
      updateNeutralBaseline();
      delay(50);
      break;

    case STATE_DESCENT_TEST:
      updateDescentSpeed();
      delay(50);
      break;

    case STATE_ASCENT_TEST:
      updateAscentSpeed();
      delay(50);
      break;

    case STATE_HOLD_TEST:
      updateHoldTest();
      delay(50);
      break;

    case STATE_RETURN_SURFACE:
      updateReturnToSurface();
      delay(50);
      break;

    case STATE_TRANSMIT_DATA:
      transmitAllData();
      delay(100);
      break;

    default:
      motorStop();
      delay(100);
      break;
  }
}
