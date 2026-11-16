#ifndef ESPCOMMUNICATION_H
#define ESPCOMMUNICATION_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

class ESPCommunication {
public:
    bool INIT();
    bool gripRight(float value);
    bool gripDetect();
    bool hasGripStatus();

    float getTorque();

private:
    volatile bool gripStatus = false;
    volatile float elbowTorque = 0;
    static ESPCommunication* instance;
    static void onReceive(const uint8_t* mac, const uint8_t* data, int len );
};

#endif