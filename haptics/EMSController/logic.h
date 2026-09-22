#pragma once
#include <Arduino.h>
#include "config.h"
#include "relay.h"
#include "MCP4231.h"

enum class EmsMode : uint8_t {
    STANDBY = 0,         // torque below threshold -> Both channels isolated
    PAIRED_FLEXION,     // Primary Ch1 (Biceps), Antagonist Ch2 (Triceps)
    PAIRED_EXTENSION,   // Primary Ch2 (Triceps), Antagonist Ch1 (Biceps)

class logic {
public:
    static void init();
    static void execute(float torque, float deltaTorque);
    static void emergencyStop();
private:
    static EmsMode classifyMode(float intensityPct, float torque, float deltaTorque);
    static uint8_t percentToStep(float pct);
};