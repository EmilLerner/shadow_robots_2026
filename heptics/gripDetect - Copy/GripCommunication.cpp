#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Replace with ESP1's MAC address!
uint8_t esp1Mac[] = {
  0x48, 0x9D, 0x31, 0xE3, 0x9F, 0xB0
};

class GripCommunication {
private:
  volatile bool gripStatus = false;
  volatile bool statusReceived = false;

  static GripCommunication* instance;

  static void onReceive(const uint8_t* mac,
                        const uint8_t* data,
                        int len) {
    if (instance == nullptr || len <= 0 || len >= 100) {
      return;
    }

    char msg[100];
    memcpy(msg, data, len);
    msg[len] = '\0';

    // Handle Python's reply: GRIPSTATUS,1 or GRIPSTATUS,0
    if (strncmp(msg, "GRIPSTATUS,", 11) == 0) {
      instance->gripStatus = (atoi(msg + 11) != 0);
      instance->statusReceived = true;

      Serial.print("Grip status received: ");
      Serial.println(instance->gripStatus ? "TRUE" : "FALSE");
    }
  }

public:
  bool begin() {
    instance = this;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
      Serial.println("ESP-NOW init failed");
      return false;
    }

    esp_now_register_recv_cb(onReceive);

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

  // Send a grip command to Python
  bool gripRight(float value) {
    char msg[40];
    snprintf(msg, sizeof(msg), "GRIP,%.3f", value);

    statusReceived = false;

    esp_err_t result = esp_now_send(
      esp1Mac,
      (const uint8_t*)msg,
      strlen(msg)
    );

    if (result == ESP_OK) {
      Serial.print("Grip command sent: ");
      Serial.println(msg);
      return true;
    }

    Serial.println("Failed to send grip command");
    return false;
  }

  // Return the latest status received from Python
  bool gripDetect() {
    return gripStatus;
  }

  bool hasGripStatus() {
    return statusReceived;
  }
};

GripCommunication* GripCommunication::instance = nullptr;

// Create the communication object
GripCommunication gripComm;

void setup() {
  Serial.begin(115200);

  if (!gripComm.begin()) {
    Serial.println("Grip communication setup failed");
    while (true) delay(1000);
  }

  Serial.println("Grip communication ready");
}

void loop() {
  // Example command, for testing only:
  // gripComm.gripRight(0.75);

  // Check the latest result whenever needed:
  if (gripComm.hasGripStatus()) {
    bool detected = gripComm.gripDetect();
    Serial.printf("Grip detected: %s\n",
                  detected ? "true" : "false");
  }

  delay(10);
}