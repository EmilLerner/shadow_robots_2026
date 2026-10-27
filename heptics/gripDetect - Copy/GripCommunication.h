#pragma once

#include <Arduino.h>
#include <esp_now.h>

class GripCommunication {
public:
    bool begin();
    bool gripRight(float value);
    bool gripDetect();
    bool hasGripStatus();

private:
    volatile bool gripStatus = false;
    volatile bool statusReceived = false;

    static GripCommunication* instance;

    static void onReceive(
        const uint8_t* mac,
        const uint8_t* data,
        int len
    );
};
