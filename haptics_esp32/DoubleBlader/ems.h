#pragma once
#include <Arduino.h>

namespace EMS {
  void begin();                       // init SPI/relays, both channels at 0%
  void set(float pct1, float pct2);   // channel intensities in % (clamped to 0..100)
}
