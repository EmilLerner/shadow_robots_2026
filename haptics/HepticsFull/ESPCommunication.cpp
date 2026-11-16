#include "ESPCommunication.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>

// ESP1's MAC address
uint8_t esp1Mac[] = {
  0x68, 0xEE, 0x8F, 0x4C, 0x23, 0xC0
};

ESPCommunication* ESPCommunication::instance = nullptr;

bool ESPCommunication::INIT() {
  instance = this;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return false;
  }

  esp_now_register_recv_cb(ESPCommunication::onReceive);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, esp1Mac, 6);
  peer.channel = 1;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add ESP1");
    return false;
  }
  return true;
}

void ESPCommunication::onReceive(const uint8_t* mac, const uint8_t* data, int len) {
  // checks if the inputs are valid
  if (instance == nullptr || data == nullptr || len <= 0 || len >= 100) return;
  
  // puts the data into a string of max 100 charecters
  char msg[100];
  memcpy(msg, data, len);
  msg[len] = '\0';
  // checks the header with number leters plus comma
  if (strncmp(msg, "GRIPSTATUS,", 11) == 0) {
    // restrives the number after the header
    instance->gripStatus = (atoi(msg + 11) != 0);
  } else if(strncmp(msg, "ELBOWTORQUE,", 12) == 0) {
    instance->elbowTorque = atof(msg + 12);
  }
}

bool ESPCommunication::gripRight(float value) {
  // puts the value into the string with "GRIP," as the header
  char msg[40];
  snprintf(msg, sizeof(msg), "GRIP,%.3f", value);
  // sends the messeges. Returns true if succssesful 
  esp_err_t result = esp_now_send(esp1Mac,(const uint8_t*)msg, strlen(msg));
  if (result == ESP_OK) return true;
  return false;
}

bool ESPCommunication::gripDetect() {
  return gripStatus;
}

float ESPCommunication::getTorque() {
  return elbowTorque;
}
