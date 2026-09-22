#include "logic.h"

void logic::init() {
    relay::init();
    MCP4231::init();
    emergencyStop();
}

void logic::emergencyStop() {
    relay::openAll();
    MCP4231::setPosition(1, 128);
    MCP4231::setPosition(2, 128);
}

uint8_t logic::percentToStep(float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    float span = (float)(INTENSITY_MIN - INTENSITY_MAX);
    return (uint8_t)(INTENSITY_MIN - ((pct / 100.0f) * span));
}

EmsMode logic::classifyMode(float netIntensityPct, float deltaTorque) {
    if (netIntensityPct <= RELAY_THRESHOLD) {
        return EmsMode::STANDBY;
    }

    if (deltaTorque >= 0.0f) {
        return EmsMode::PAIRED_FLEXION;
    } else {
        return EmsMode::PAIRED_EXTENSION;
    }
}

void logic::execute(float torque, float deltaTorque) {
    // 1. Calculate normalized net intensity (0.0 to 1.0)
    float magnitude = fabs(torque);
    float normTorque = constrain((magnitude - TORQUE_MIN_SCALE_NM) / (TORQUE_MAX_SCALE_NM - TORQUE_MIN_SCALE_NM), 0.0f, 1.0f);
    float netIntensityPct = normTorque * 100.0f;

    EmsMode mode = classifyMode(netIntensityPct, deltaTorque);

    float ch1DemandPct = 0.0f;
    float ch2DemandPct = 0.0f;

    switch (mode) {
        case EmsMode::PAIRED_FLEXION: {
            // Agonist: Biceps (CH1) | Antagonist: Triceps (CH2)
            // Agonist receives directional demand + baseline stiffness
            ch1DemandPct = (PAIRED_EMS_BIAS + (1.0f - PAIRED_EMS_BIAS) * normTorque) * 100.0f;
            
            // Antagonist provides joint stiffness, tapering off as torque saturates
            ch2DemandPct = (PAIRED_EMS_BIAS * (1.0f - normTorque)) * 100.0f;

            // Both relays engage to maintain closed pair
            relay::closeCh1();
            if (ch2DemandPct > RELAY_THRESHOLD) {
                relay::closeCh2();
            } else {
                relay::openCh2();
            }
            break;
        }

        case EmsMode::PAIRED_EXTENSION: {
            // Agonist: Triceps (CH2) | Antagonist: Biceps (CH1)
            ch2DemandPct = (PAIRED_EMS_BIAS + (1.0f - PAIRED_EMS_BIAS) * normTorque) * 100.0f;
            ch1DemandPct = (PAIRED_EMS_BIAS * (1.0f - normTorque)) * 100.0f;

            relay::closeCh2();
            if (ch1DemandPct > RELAY_THRESHOLD) {
                relay::closeCh1();
            } else {
                relay::openCh1();
            }
            break;
        }

        case EmsMode::STANDBY:
        default:
            emergencyStop();
            Serial.printf("[LOGIC] STANDBY | Torque: %5.1f Nm | Both Channels Cut Off\n", magnitude);
            return;
    }

    // Apply steps to both MCP4231 wipers simultaneously
    uint8_t stepCh1 = percentToStep(ch1DemandPct);
    uint8_t stepCh2 = percentToStep(ch2DemandPct);

    MCP4231::setPosition(1, stepCh1);
    MCP4231::setPosition(2, stepCh2);

    Serial.printf("[PAIRED-EMS] Mode: %s | T: %5.1f Nm | Agonist: %4.1f%% (Step %2u) | Antagonist: %4.1f%% (Step %2u)\n",
                  (mode == EmsMode::PAIRED_FLEXION) ? "FLEX" : "EXT ",
                  magnitude,
                  (mode == EmsMode::PAIRED_FLEXION) ? ch1DemandPct : ch2DemandPct,
                  (mode == EmsMode::PAIRED_FLEXION) ? stepCh1 : stepCh2,
                  (mode == EmsMode::PAIRED_FLEXION) ? ch2DemandPct : ch1DemandPct,
                  (mode == EmsMode::PAIRED_FLEXION) ? stepCh2 : stepCh1);
}