#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

uint8_t esp2Mac[] = {
  0x8C, 0x94, 0xDF, 0xAC, 0x6E, 0x5C
};

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  Serial.write(data, len);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  Serial.print("ESP1 MAC: ");
  Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, esp2Mac, 6);
  peer.channel = 1;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add ESP2");
    while (true) delay(1000);
  }

  Serial.println("ESP1 ready");
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    if (msg.length() > 0 && msg.length() <= 200) {
      esp_err_t result = esp_now_send(esp2Mac, (const uint8_t*)msg.c_str(), msg.length());
      if (result != ESP_OK) {
        Serial.println("ERROR,SEND_FAILED");
      }
    }
  }
}