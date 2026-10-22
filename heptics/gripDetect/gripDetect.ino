// main file for running the code
// Emil Lerner 18/8/2026

// testing 
#define sampleNumber 200
#define ratioPC  10.0f
#define stallCurrent  200.0f

#define PUMP_PIN 13
#define VALVE_PIN  14
#define PRESURE_PIN  35
#define joyX_PIN 34
#include "pressureCtrl.h"
#include "motorCTR.h"
#include <array>

float Kp = 5; 
float Ki = 1;
float hold = 100;

float baseSetPoint = 0.1;
float setPoint = baseSetPoint;

float speed;
float pressure;
float inputPressure;
float addedPressure = 0;
float goalPressure = 0;

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


pressureCtrl controler1(PRESURE_PIN, PUMP_PIN, VALVE_PIN, hold, Kp, Ki, baseSetPoint);
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
  joyX = constrain(((float)(joyX_raw - 2080) / 20.0f), -100, 100);


  speed =  constrain(joyX, 0, 100);

  inputPressure = controler1.ctrl_update(constrain(setPoint, baseSetPoint, 2), motor.gripDetected());

  if (speed < 10) speed = constrain(inputPressure * 150 - 10, 0, 100);

  if ((inputPressure > 2*baseSetPoint) || (joyX > 30)) {
    motor.closeGrip(speed);
  } else {
    motor.openGrip(70);
  }

  setPoint = inputPressure;

  Serial.print(setPoint);
  Serial.print(",");

  
  controler1.ctrl_plot();


  delay(20);

}