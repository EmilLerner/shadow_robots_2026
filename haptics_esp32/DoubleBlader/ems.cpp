#include "ems.h"
#include <SPI.h>

#define PIN_RELAY1_CTRL 27   // GPIO 3 (CH1 Relays)
#define PIN_RELAY2_CTRL 26   // GPIO 4 (CH2 Relays)
#define PIN_CS_          22   // GPIO 5 (MCP4231 Chip Select)
#define PIN_SCK         18   // GPIO 8
#define PIN_MISO        19   // GPIO 9
#define PIN_MOSI        23  // GPIO 10

namespace {
constexpr uint8_t PIN_RELAY[2]  = {PIN_RELAY1_CTRL, PIN_RELAY2_CTRL};
constexpr uint8_t PIN_CS        = PIN_CS_;
constexpr uint8_t WIPER_ADDR[2] = {0x00, 0x01};
constexpr uint8_t TCON_ADDR     = 0x04;
constexpr float   RELAY_ON_PCT  = 5.0f;

int  steps[2]  = {128, 128};   // 128 = 0%, 0 = 100%
bool relays[2] = {false, false};

void writeMCP(uint8_t reg, uint16_t data) {
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer((reg << 4) | ((data >> 8) & 0x03));
  SPI.transfer(data & 0xFF);
  delayMicroseconds(2);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void apply(int ch, float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  int  step = lroundf(128.0f * (1.0f - pct / 100.0f));
  bool on   = pct > RELAY_ON_PCT;

  if (step != steps[ch]) { steps[ch] = step; writeMCP(WIPER_ADDR[ch], step); }
  if (on != relays[ch])  { relays[ch] = on;  digitalWrite(PIN_RELAY[ch], on); }

  Serial.print("Channel: ");
  Serial.print(ch);
  Serial.print(",  Step: ");
  Serial.print(step);
  Serial.print(",  Status");
  Serial.println(on);
}
}  // namespace

void EMS::begin() {
  for (uint8_t pin : PIN_RELAY) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);

  writeMCP(TCON_ADDR, 0x01FF);
  delay(50);
  for (uint8_t addr : WIPER_ADDR) writeMCP(addr, 128);
}

void EMS::set(float pct1, float pct2) {
  apply(0, pct1);
  apply(1, pct2);
}
