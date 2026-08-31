// main file for running the code
// Emil Lerner 18/8/2026

// testing 

#define PUMP_PIN 26
#define VALVE_PIN  25
#define PRESURE_PIN  32
#define joyX_PIN 34
#include "PID.h"
#include "motorCTR.h"

float Kp = 7;
float Ki = .5;
float hold = 5;

float setpoint = 0.2;

float baseCurrent = 38.1;

PID controler1PID(PRESURE_PIN, PUMP_PIN, VALVE_PIN, hold, Kp, Ki);


motorCTR motor;


void setup() {
    Serial.begin(115200);

    delay(1000);
    Serial.println("Starting...");
    motor.connect();
    Serial.println("Connected");

    controler1PID.PID_INIT();
    pinMode(joyX_PIN, INPUT);

}

void loop() {

  
  // read the joy stick values and the current values
  int joyX_raw = analogRead(joyX_PIN);
  int speed = constrain(((float)(joyX_raw - 1770) / 8.0f), -255, 255);


  float current = motor.getCurrent() - baseCurrent;

  if(speed > 0) {
    setpoint = current / 400;
  } else {
    setpoint = 0.02;
  }
  //int current = current_raw - 512;

  // use the current to find the set point 
  //setpoint = (float)((joyX_raw - 1800)) / 8192.0f;
  setpoint = constrain(setpoint, 0.02, 1);
  //setpoint = 1;
  
  motor.turn(speed);

  Serial.print(" Current: ");
  Serial.println(current);

  // get PID controler running
  controler1PID.PID_update(setpoint);
  controler1PID.PID_plot();
  // Serial.print(speed);
  // Serial.print(", ");
  // Serial.println(current);


  delay(10);

}