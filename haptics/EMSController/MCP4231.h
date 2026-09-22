#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "config.h"

class MCP4231 {
public:
    static void init();

    // position control
    static void setPosition(uint8_t channel, uint8_t step);
    static uint8_t getPosition(uint8_t channel);

    // step modifiers
    static void increment(uint8_t channel, uint8_t delta = 1);
    static void decrement(uint8_t channel, uint8_t delta = 1);

private:
    static uint8_t ch1Position;
    static uint8_t ch2Position;

    static void writeRegister(uint8_t regAddress, uint16_t data);
};