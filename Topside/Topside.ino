#include <WiFi.h>
#include <esp_now.h>

// ── ESP-NOW: MAC address of the FLOAT (ROV) ESP32 ───────────────────────────
uint8_t deviceMAC[] = {0x70, 0x4B, 0xCA, 0x26, 0x81, 0x40};

unsigned long packetsReceived = 0;

// ── Send a motor command to the float ────────────────────────────────────────
void sendCommand(String cmd) {
  cmd.trim();

  if (cmd != "FORWARD" && cmd != "REVERSE" && cmd != "STOP") {
    Serial.println("ERROR: Invalid command. Use FORWARD, REVERSE, or STOP.");
    return;
  }

  esp_err_t result = esp_now_send(deviceMAC, (uint8_t*)cmd.c_str(), cmd.length() + 1);

  if (result == ESP_OK) {
    Serial.print("SEND_OK: ");
    Serial.println(cmd);
  } else {
    Serial.print("SEND_FAIL: ");
    Serial.println(cmd);
  }
}

// ── Receive sensor data from float ───────────────────────────────────────────
void OnRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  packetsReceived++;
  String msg = String((char*)data);
  msg.trim();

  if (msg.startsWith("SENSOR:")) {
    msg = msg.substring(7);

    int s1 = msg.indexOf(':');
    int s2 = msg.indexOf(':', s1 + 1);

    String pressure   = msg.substring(0, s1);
    String depth      = msg.substring(s1 + 1, s2);
    String motorState = msg.substring(s2 + 1);

    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.print(" mbar | Depth: ");
    Serial.print(depth);
    Serial.print(" ft | Motor: ");
    Serial.print(motorState);
    Serial.print(" | Packets: ");
    Serial.println(packetsReceived);
  } else {
    Serial.println(msg);
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(1000);
  }

  esp_now_register_recv_cb(OnRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, deviceMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add float peer");
    while (1) delay(1000);
  }

  Serial.println("Topside controller ready");
  Serial.println("Commands: FORWARD | REVERSE | STOP");
  Serial.print("My MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    sendCommand(cmd);
  }
}