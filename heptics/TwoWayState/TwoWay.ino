// main file for running the code
// Emil Lerner 18/8/2026

// testing 

#define PUMP_PIN 26
#define VALVE_PIN  25
#define PRESURE_PIN  32
#define joyX_PIN 34
#include "pressureCtrl.h"
#include "motorCTR.h"

float Kp = 7;
float Ki = 1;
float hold = 20;

float baseSetpoint = 0.03;
float setpoint = baseSetpoint;

float pressure;
float inputPressure;
float addedPressure = 0;

float current;
float baseCurrent = 38.7;

int joyX_raw;
int joyX;

int position;


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

  pressure = controler1.getPressure();
  inputPressure = pressure - addedPressure;
  if (position < 10) position = constrain(inputPressure * 200 - 20, 0, 100);

  current = motor.getCurrent() - baseCurrent;
  addedPressure = current / 30;
  setpoint = constrain(addedPressure, baseSetpoint, 1.5);


  motor.moveToPosition(position, 100);

  Serial.print(" Current: ");
  Serial.println(current);

  controler1.ctrl_update(setpoint);
  controler1.ctrl_plot();


  delay(20);

}