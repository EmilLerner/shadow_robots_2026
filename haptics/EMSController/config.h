#pragma once
#include <Arduino.h>

// MCP4231
#define PIN_CS  D3  // MCP4231 chip select
#define PIN_SCK  D8  // SPI Clock
#define PIN_MISO  D9  // SPI SDO
#define PIN_MOSI  D10  // SPI SDI

// relays
#define PIN_RELAY_CH1  D1  // channel 1 relays
#define PIN_RELAY_CH2  D2  // channel 2 relays
#define RELAY_THRESHOLD  5.0f // relays do not turn on until ~5% position on MCP4231

// intensity curve calibration
#define INTENSITY_MIN  96  // min intensity at 96 steps
#define INTENSITY_MAX  32  // max intensity at 32 steps

// UR5e specifications and constraints
#define ELBOW_JOINT_INDEX  2  // joint 2 in RTDE array
#define ELBOW_MAX_TORQUE  150.0f  // 150Nm max rated torque

// Paired EMS parameters
#define PAIRED_EMS_BIAS  0.20f  // antagonist contracts at 20% of agonist


