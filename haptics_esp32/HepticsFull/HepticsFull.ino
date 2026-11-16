// main file for running the code
// Emil Lerner 18/8/2026

// testing 

#define PUMP_PIN 13
#define VALVE_PIN  14
#define PRESURE_PIN  35
#include "pressureCtrl.h"
#include "ESPCommunication.h"
#include "ems.h"
#include "control.h"

float Kp = 5; 
float Ki = 1;
float hold = 100;

float baseSetPoint = 0.2 ;
float setPoint = baseSetPoint;

float speed;
float pressure;
float inputPressure;



pressureCtrl controler1(PRESURE_PIN, PUMP_PIN, VALVE_PIN, hold, Kp, Ki, baseSetPoint);
ESPCommunication rightArm;

void setup() {
    Serial.begin(115200);
    EMS::begin();
    delay(1000);
    Serial.println("Starting...");
    controler1.ctrl_INIT();
    rightArm.INIT();
    delay(500);
}

void loop() {

  inputPressure = controler1.ctrl_update(rightArm.gripDetect());
  speed = constrain(inputPressure * 150 - 10, 0, 100);
  if ((inputPressure > 1.5 *baseSetPoint)) {
    rightArm.gripRight(1.0f - ((float)speed/100.0f));
  } else {
    rightArm.gripRight(1);
  }


  float torque = rightArm.getTorque();
  Serial.print("Torque:  ");
  Serial.println(torque);
  Control::onTorque(torque);
  Control::update();

  delay(20);
}



