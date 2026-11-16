#include "pressureCtrl.h"

#define pwmResolution 8   


pressureCtrl::pressureCtrl(int pressurePin, int pumpPin, int drainPin, int drainChannel,int drainFreq, int hold, float Kp, float Ki, float minPressure)
    : pressurePin(pressurePin),
      pumpPin(pumpPin),
      drainPin(drainPin),
      drainChannel(drainChannel),
      drainFreq(drainFreq),
      hold(hold),
      Kp(Kp),
      Ki(Ki),
      minPressure(minPressure),
      minReading(1095),
      maxReading(4095),
      reading(0){}

void pressureCtrl::ctrl_INIT() {
  ledcSetup(drainChannel, 5, pwmResolution);
  ledcAttachPin(drainPin, drainChannel);
  ledcSetup(drainChannel + 1, 20000, pwmResolution);
  ledcAttachPin(pumpPin, drainChannel + 1);

  analogReadResolution(12);
  analogSetPinAttenuation(pressurePin, ADC_0db);

  for (int i = 0; i < 100; i++) {
    reading = analogRead(pressurePin);
    if (reading < minReading) minReading = reading;
    reading = analogRead(pressurePin);
  }
  maxPressure = 2.0f * (maxReading - minReading) /
                     (float)(maxReading - minReading);
  Serial.print("Min Reading: ");
  Serial.println(minReading);

}



float pressureCtrl::getPressure() {
  reading = analogRead(pressurePin);
  pressure = 2.0f * (reading - minReading) /
                     (float)(maxReading - minReading);
  pressure = constrain(pressure, 0.0, 2.0);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  return pressure;
}






void pressureCtrl::ctrl_plot() {
  Serial.print(pressure);
  Serial.print(",");
  Serial.print(motorSpeed / 255);
  Serial.print(",");
  Serial.print(-2);
  Serial.print(",");
  Serial.println(2); 
}



float pressureCtrl::ctrl_maintain() {
  
  getPressure();
  if (pressure < minPressure)  {
    error = minPressure - pressure;
    integral += error * 0.02; 
    integral = constrain(integral, -1, 1);
    output = Kp * error + Ki * integral;
  } else if (pressure >= maxPressure) {
    output = 0;
  } else {
    output = 0;
  }
  
  motorSpeed = constrain(output * 1024, -255, 255);


  if (fabs(motorSpeed) < hold) {
    ledcWrite(drainChannel, 255);
    ledcWrite(drainChannel + 1,0);
  } else if (motorSpeed > 0) {
    ledcWrite(drainChannel, 254);
    ledcWrite(drainChannel + 1, (int)motorSpeed);
  } else {
    ledcWrite(drainChannel, constrain(200 + (int)motorSpeed, 0, 255));
    ledcWrite(drainChannel + 1,0);
  }

  return pressure;

}

float pressureCtrl::ctrl_set(float setPoint) {
  getPressure();

  if (setPoint == 0)  {
    output = -1;
  } else if (pressure < setPoint) {
    error = setPoint - pressure;
    integral += error * 0.02; 
    integral = constrain(integral, -1, 1);
    output = Kp * error + Ki * integral;
  } else {
    output = 0;
  }
  motorSpeed = constrain(output * 1024, -255, 255);
  
  if (fabs(motorSpeed) < hold) {
    ledcWrite(drainChannel, 255);
    ledcWrite(drainChannel + 1,0);
  } else if (motorSpeed > 0) {
    ledcWrite(drainChannel, 254);
    ledcWrite(drainChannel + 1, (int)motorSpeed);
  } else {
    ledcWrite(drainChannel, constrain(200 + (int)motorSpeed, 0, 255));
    ledcWrite(drainChannel + 1,0);
  }

  return pressure;

}
