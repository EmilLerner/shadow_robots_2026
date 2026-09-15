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

float baseCurrent = 38.7;


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
  int joyX_raw = analogRead(joyX_PIN);
  int joyX = constrain(((float)(joyX_raw - 1770) / 20.0f), -100, 100);
  int speed = joyX;

  int position =  constrain(joyX, 0, 100);
  float current = motor.getCurrent() - baseCurrent;

  if(speed > 0) {
    setpoint = current / 30;
  } else {
    setpoint = baseSetpoint;
  }
  //int current = current_raw - 512;

  // use the current to find the set point 
  //setpoint = (float)((joyX_raw - 1800)) / 8192.0f;
  setpoint = constrain(setpoint, baseSetpoint, 1.5);
  //setpoint = 1;
  
  //motor.turn(speed);
  motor.moveToPosition(position, 100);
  //Serial.print(" Current: ");
  //Serial.println(current);

  // get PID controler running
  controler1.ctrl_update(setpoint);
  controler1.ctrl_plot();


  delay(20);

}