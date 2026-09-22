#pragma once
#include <Arduino.h>
#include "config.h"

class relay {
public:
    static void init();

    // channel 1 control
    static void openCh1();
    static void closeCh1();
    static bool isCh1Closed();

    // channel 2 control
    static void openCh2();
    static void closeCh2();
    static bool isCh2Closed();

    // dual channel control
    static void openAll();
    static void closeAll();

private:
    static bool ch1State;
    static bool ch2State;
};