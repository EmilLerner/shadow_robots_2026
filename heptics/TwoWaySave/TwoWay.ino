// main file for running the code
// Emil Lerner 18/8/2026

// testing 
#define sampleNumber 200
#define ratioPC  50.0f

#define PUMP_PIN 26
#define VALVE_PIN  25
#define PRESURE_PIN  32
#define joyX_PIN 34
#include "pressureCtrl.h"
#include "motorCTR.h"
#include <array>

float Kp = 7;
float Ki = 1;
float hold = 5;

float baseSetpoint = 0.03;
float setpoint = baseSetpoint;

float pressure;
float inputPressure;
float addedPressure = 0;

float current = 0;
float currentPrev = 0;
float baseCurrent = 38.7;

int joyX_raw;
int joyX;

int position;




int i = 0;
float sum = 0.0;
std::array<float, sampleNumber> samples{};


float addSample(float x) {
    sum -= samples[i];  // remove oldest
    samples[i] = x;     // add newest
    sum += x;

    i = (i + 1) % sampleNumber;

    return sum / (float)sampleNumber;
}


pressureCtrl controler1(PRESURE_PIN, PUMP_PIN, VALVE_PIN, hold, Kp, Ki);
motorCTR motor;

void setup() {
    Serial.begin(115200);

    delay(1000);
    Serial.println("Starting...");
    motor.connect();
    Serial.println("Connected");

    controler1.ctrl_INIT();
    pinMode(joyX_PIN, INPUT);

    motor.position_INIT();
    delay(500);


}

void loop() {

  
  // read the joy stick values and the current values
  joyX_raw = analogRead(joyX_PIN);
  joyX = constrain(((float)(joyX_raw - 1770) / 20.0f), -100, 100);
  position =  constrain(joyX, 0, 100);

  currentPrev = current;
  current = motor.getCurrent() - baseCurrent;
  addedPressure = (current - currentPrev) /   50.0f;

  pressure = controler1.getPressure();
  inputPressure = pressure;
  if (position < 10) position = constrain(inputPressure * 200 - 40, 0, 100);

  
  setpoint = constrain(pressure, baseSetpoint, 1.5);


  motor.moveToPosition(position, 100);

  //Serial.print(" Current: ");
  Serial.print(current /   50.0f);
  Serial.print(",");
  Serial.print(addedPressure);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  controler1.ctrl_update(setpoint);
  controler1.ctrl_plot();


  delay(20);

}