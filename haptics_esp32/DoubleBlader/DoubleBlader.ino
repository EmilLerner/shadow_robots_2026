// Final heptics code for both EMS and air bladder. Made for double bladder configuration.
// Put esp1 code on an esp connected to the computer through USB. Make sure that the mac address on
// the ESPCommunication matches the esp connected to the computer and the mac address in the esp1 file matches the main esp 32


// Emil Lerner 16/11/2026



#define CTR_PUMP_PIN 13 
#define CTR_DRAIN_PIN  14
#define CTR_PRESURE_PIN  35

#define FDBK_PUMP_PIN 33
#define FDBK_DRAIN_PIN  25
#define FDBK_PRESURE_PIN  32
#include "pressureCtrl.h"
#include "ESPCommunication.h"



#include "ems.h"
#include "control.h"

float Kp = 4; 
float Ki = 0.5;
float hold = 100;

float baseSetPoint = 0.2 ;
float setPoint = baseSetPoint;

float speed;
float pressure;
float inputPressure;
float addedPressure;



pressureCtrl gripperCtrl(CTR_PRESURE_PIN, CTR_PUMP_PIN, CTR_DRAIN_PIN, 0, 20000, hold, Kp, Ki, baseSetPoint);
pressureCtrl feedbackCtrl(FDBK_PRESURE_PIN, FDBK_PUMP_PIN, FDBK_DRAIN_PIN,  2, 5, hold, Kp, Ki, baseSetPoint);
ESPCommunication rightArm;

void setup() {
    Serial.begin(115200);
    EMS::begin();

    delay(1000);
    Serial.println("Starting...");
    gripperCtrl.ctrl_INIT();
    feedbackCtrl.ctrl_INIT();
    rightArm.INIT();
    delay(500);
}

void loop() {

  // if grip detected inflate the feedback air blader and messure the added pressure

  if (rightArm.gripDetect()) {
    addedPressure = feedbackCtrl.ctrl_set(1);
  } else {
    addedPressure = 0;
    feedbackCtrl.ctrl_set(0);
  }

  // maintain pressure on the mian bladder and calculate the humman input
  inputPressure = gripperCtrl.ctrl_maintain() - (0.0f * addedPressure);
  speed = constrain(inputPressure * 150 - 10, 0, 100);
  // if the value is high enogh move the grip
  if ((inputPressure > 1.5 *baseSetPoint)) {
     rightArm.gripRight(1.0f - ((float)speed/100.0f));
  } else {
    rightArm.gripRight(1);
  }

  

  // get torue from the arm and then use ot to control the ems
  float torque = rightArm.getTorque();
  // Serial.print("Torque:  ");
  // Serial.println(torque);
  Control::onTorque(torque);
  Control::update();

  delay(20);
}



