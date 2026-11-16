#include "control.h"
#include "ems.h"
#include <Arduino.h>

namespace {
constexpr float    FULLSCALE_NM  = 20.0f;  // |torque| that maps to 100%
constexpr float    BIAS          = 0.60f;  // secondary = primary * BIAS
constexpr float    DTORQUE_WARN  = 3.0f;   // Nm change per sample that triggers a buzz
constexpr float    BUZZ_PCT      = 10.0f;  // light intensity added to both channels
constexpr uint32_t BUZZ_MS       = 300;    // buzz duration (retriggers on further spikes)
constexpr uint32_t BUZZ_PERIOD   = 40;     // on/off toggle period, gives the "buzz" feel

float    prev = 0.0f, primary = 0.0f, secondary = 0.0f;
bool     cw = true;
uint32_t buzzEnd = 0;
}  // namespace

void Control::onTorque(float nm) {
  float dTorque = nm - prev;
  prev = nm;

  if (nm != 0.0f) cw = nm > 0.0f;   // hold last direction at zero torque

  float p = fabsf(nm) / FULLSCALE_NM * 100.0f;
  primary   = p;
  secondary = p * BIAS;

  if (fabsf(dTorque) > DTORQUE_WARN) buzzEnd = millis() + BUZZ_MS;
}

void Control::update() {
  uint32_t now  = millis();
  bool     buzz = (int32_t)(buzzEnd - now) > 0 && ((now / BUZZ_PERIOD) & 1);
  float    add  = buzz ? BUZZ_PCT : 0.0f;

  float ch1 = (cw ? primary : secondary) + add;   // CW:  CH1 from torque, CH2 from CH1
  float ch2 = (cw ? secondary : primary) + add;   // CCW: CH2 from torque, CH1 from CH2
  EMS::set(ch1, ch2);
}
