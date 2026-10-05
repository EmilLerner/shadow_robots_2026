#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

typedef struct __attribute__((packed)) {
  float torque;
  float delta_torque;
  float torque_rate;
  uint32_t seq;
} TelemetryPacket;

TelemetryPacket rxPacket;

void onDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(TelemetryPacket)) {
    memcpy(&rxPacket, incomingData, sizeof(TelemetryPacket));

    Serial.printf("[#%u] Torque: %5.2f Nm | dTorque: %5.3f Nm | Rate: %6.2f Nm/s\n",
                  rxPacket.seq,
                  rxPacket.torque,
                  rxPacket.delta_torque,
                  rxPacket.torque_rate);
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Match the transmitter channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW initialization failed");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("[OK] ESP-NOW Receiver ready.");
}

void loop() {
  // Empty loop: processing happens asynchronously in the ESP-NOW callback
}