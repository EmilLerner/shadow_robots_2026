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
float Ki = .5;
float hold = 5;

float setpoint = 0.2;

float baseCurrent = 38.1;

int p0 = 0;

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

    p0 = motor.getPosition();

}

void loop() {

  
  // read the joy stick values and the current values
  int joyX_raw = analogRead(joyX_PIN);
  int joyX = constrain(((float)(joyX_raw - 1770) / 8.0f), -255, 255);
  int speed = joyX;

  int position = p0 + joyX;

  // if (fabs(joyX) < 20) {
  //   speed = constrain(((controler1.getPressure() - 0.1) * 400), -255, 255);
  //   speed = speed * -1;
  // }


  float current = motor.getCurrent() - baseCurrent;

  if(speed < 0) {
    setpoint = current / 400;
  } else {
    setpoint = 0.02;
  }
  //int current = current_raw - 512;

  // use the current to find the set point 
  //setpoint = (float)((joyX_raw - 1800)) / 8192.0f;
  setpoint = constrain(setpoint, 0.05, 1);
  //setpoint = 1;
  
  //motor.turn(speed);
  motor.moveToPosition(position, 255);
  Serial.print(" Current: ");
  Serial.println(current);

  // get PID controler running
  controler1.ctrl_update(setpoint);
  controler1.ctrl_plot();
  // Serial.print(speed);
  // Serial.print(", ");
  // Serial.println(current);


  delay(10);

}