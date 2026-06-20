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

void motorForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, speed);
  currentMotorState = "FORWARD";
}

void motorReverse(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN1, speed);
  currentMotorState = "REVERSE";
}

void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(EN1, 0);
  currentMotorState = "STOP";
}

// ── Send sensor data to topside ───────────────────────────────────────────────
void sendSensorData() {
  sensor.read();
  float depthFeet = sensor.depth() * 3.28084;

  String msg = "SENSOR:";
  msg += String(sensor.pressure(), 2);
  msg += ":";
  msg += String(depthFeet, 2);
  msg += ":";
  msg += currentMotorState;

  esp_now_send(controllerMAC, (uint8_t*)msg.c_str(), msg.length() + 1);
}

// ── Receive motor commands from topside ──────────────────────────────────────
void OnRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  String cmd = String((char*)data);
  cmd.trim();

  Serial.print("CMD received: ");
  Serial.println(cmd);

  if (cmd == "FORWARD") {
    motorForward(200);
  } else if (cmd == "REVERSE") {
    motorReverse(200);
  } else if (cmd == "STOP") {
    motorStop();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(EN1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motorStop();

  Wire.begin(21, 22);
  sensor.setModel(MS5837::MS5837_30BA);
  if (!sensor.init()) {
    Serial.println("Sensor init failed!");
    while (1) delay(1000);
  }
  sensor.setFluidDensity(997);
  Serial.println("MS5837 initialized");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(1000);
  }

  esp_now_register_recv_cb(OnRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, controllerMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add topside peer");
    while (1) delay(1000);
  }

  Serial.println("ROV device ready");
  Serial.print("My MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  sendSensorData();
  delay(500);
}