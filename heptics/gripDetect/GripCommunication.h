#ifndef GRIPCOMMUNICATION_H
#define GRIPCOMMUNICATION_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

class GripCommunication {
public:
    bool INIT();
    bool gripRight(float value);
    bool gripDetect();
    bool hasGripStatus();

private:
    volatile bool gripStatus = false;
    static GripCommunication* instance;
    static void onReceive(const uint8_t* mac, const uint8_t* data, int len );
};

#endif