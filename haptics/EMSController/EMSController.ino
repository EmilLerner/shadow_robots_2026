#include <SPI.h>
#include "MCP4231.h"

// ESP32-C3 Supermini testing GAAQY210, MCP4231
const int RELAY_PIN = 20;
static const int8_t PIN_SCK  = 4;
static const int8_t PIN_MISO = 5;
static const int8_t PIN_MOSI = 6;
static const int8_t PIN_CS   = 7;

MCP4231 pot(PIN_CS);  // GPIO -> resistor -> Pin 1 (Anode)

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // LED off at boot
  Serial.println("Send '1' to turn LED ON, '0' to turn OFF.");

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  pot.begin();

  Serial.println("MCP4231 rheostat demo ready.");
}

void loop() {
  // relay switch
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("LED circuit ON");
    } else if (c == '0') {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("LED circuit OFF");
    }
  }
}