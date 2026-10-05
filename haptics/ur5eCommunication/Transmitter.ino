#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// --- Configuration ---
// REPLACE THIS with your receiver ESP32's actual MAC address
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Binary protocol constants
const uint8_t HEADER_BYTE1 = 0xAA;
const uint8_t HEADER_BYTE2 = 0x55;
const size_t PAYLOAD_SIZE = 4; // 32-bit float from Python

typedef struct __attribute__((packed)) {
  float torque;       // Instantaneous torque (Nm)
  float delta_torque; // Delta torque: tau[k] - tau[k-1] (Nm)
  float torque_rate;  // Rate of change: d(tau)/dt (Nm/s)
  uint32_t seq;       // Packet sequence counter
} TelemetryPacket;

TelemetryPacket txPacket;
uint32_t packetCounter = 0;

// Derivative tracking variables
float prevTorque = 0.0f;
uint32_t prevTimestampUs = 0;
bool firstSample = true;

enum ParseState {
  WAIT_HEADER_1,
  WAIT_HEADER_2,
  READ_PAYLOAD
};

ParseState currentState = WAIT_HEADER_1;
uint8_t payloadBuffer[PAYLOAD_SIZE];
size_t bytesRead = 0;

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Pin Wi-Fi channel to Channel 1
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();

    switch (currentState) {
      case WAIT_HEADER_1:
        if (inByte == HEADER_BYTE1) {
          currentState = WAIT_HEADER_2;
        }
        break;

      case WAIT_HEADER_2:
        if (inByte == HEADER_BYTE2) {
          currentState = READ_PAYLOAD;
          bytesRead = 0;
        } else if (inByte != HEADER_BYTE1) {
          currentState = WAIT_HEADER_1;
        }
        break;

      case READ_PAYLOAD:
        payloadBuffer[bytesRead++] = inByte;

        if (bytesRead == PAYLOAD_SIZE) {
          uint32_t nowUs = micros();
          float currentTorque = 0.0f;
          memcpy(&currentTorque, payloadBuffer, sizeof(float));

          // Compute delta and torque rate
          if (firstSample) {
            txPacket.torque = currentTorque;
            txPacket.delta_torque = 0.0f;
            txPacket.torque_rate = 0.0f;
            firstSample = false;
          } else {
            float dt = (nowUs - prevTimestampUs) * 1e-6f; // Convert microseconds to seconds
            float delta = currentTorque - prevTorque;

            txPacket.torque = currentTorque;
            txPacket.delta_torque = delta;
            txPacket.torque_rate = (dt > 1e-4f) ? (delta / dt) : 0.0f; // Guard against dt ~ 0
          }

          // Update history
          prevTorque = currentTorque;
          prevTimestampUs = nowUs;
          txPacket.seq = ++packetCounter;

          // Dispatch immediately over ESP-NOW
          esp_now_send(receiverAddress, (uint8_t *)&txPacket, sizeof(txPacket));

          currentState = WAIT_HEADER_1;
        }
        break;
    }
  }
}