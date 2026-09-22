#include "MCP4231.h"

#define ADDR_WIPER_0 0x00 // channel 1 (pin 6 P0W)
#define ADDR_WIPER_1 0x01 // channel 2 (pin 9 P1W)
#define ADDR_TCON    0x04 // terminal connection register
#define CMD_WRITE    0x00

uint8_t MCP4231::ch1Position = 128;
uint8_t MCP4231::ch2Position = 128;

void MCP4231::init() {
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);

    // Connect all potentiometer terminals in TCON register
    writeRegister(ADDR_TCON, 0x01FF);
    delay(10);

    // set both wipers safely at 0
    setPosition(1, 0);
    setPosition(2, 0);
}

void MCP4231::setPosition(uint8_t channel, uint8_t step) {
    uint8_t clampedStep = constrain(step, 0, 128);

    if (channel == 1) {
        ch1Position = clampedStep;
        writeRegister(ADDR_WIPER_0, ch1Position);
    } else if (channel == 2) {
        ch2Position = clampedStep;
        writeRegister(ADDR_WIPER_1, ch2Position);
    }
}

uint8_t MCP4231::getPosition(uint8_t channel) {
    return (channel == 1) ? ch1Position : ch2Position;
}

void MCP4231::increment(uint8_t channel, uint8_t delta) {
    uint8_t current = getPosition(channel);
    if (current + delta > 128) {
        setPosition(channel, 128);
    } else {
        setPosition(channel, current + delta);
    }
}

void MCP4231::decrement(uint8_t channel, uint8_t delta) {
    uint8_t current = getPosition(channel);
    if (current < delta) {
        setPosition(channel, 0);
    } else {
        setPosition(channel, current - delta);
    }
}

void MCP4231::writeRegister(uint8_t regAddress, uint16_t data) {
    uint8_t highByte = (regAddress << 4) | (CMD_WRITE << 2) | ((data >> 8) & 0x03);
    uint8_t lowByte  = data & 0xFF;

    SPI.beginTransaction(SPISettings(250000, MSBFIRST, MODE0));
    digitalWrite(PIN_CS, LOW);
    delayMicroseconds(2);
    SPI.transfer(highByte);
    SPI.transfer(lowByte);
    delayMicroseconds(2);
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
}