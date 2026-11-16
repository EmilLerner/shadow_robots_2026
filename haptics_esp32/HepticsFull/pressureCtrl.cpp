#include "pressureCtrl.h"

#define pwmChannel 0
#define pwmFreq 5       // Hz — change to valve requirement
#define pwmResolution 8   // 0–255


pressureCtrl::pressureCtrl(int pressurePin,  int pumpPin, int valvePin, int hold, float Kp, float Ki, float minPressure)
    : pressurePin(pressurePin),
      pumpPin(pumpPin),
      valvePin(valvePin),
      hold(hold),
      Kp(Kp),
      Ki(Ki),
      minPressure(minPressure),
      minReading(1095),
      maxReading(4095),
      reading(0),
      pressureAdded(0),
      reset(false) {  
}

void pressureCtrl::ctrl_INIT() {
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(valvePin, pwmChannel);

  pinMode(pumpPin, OUTPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(pressurePin, ADC_0db);

  for (int i = 0; i < 100; i++) {
    reading = analogRead(pressurePin);
    if (reading < minReading) minReading = reading;
    reading = analogRead(pressurePin);
  }
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
  //Serial.print(output);
  //Serial.print(",");
  Serial.print(motorSpeed / 255);
  Serial.print(",");
  Serial.print(-2);
  Serial.print(",");
  Serial.println(2);   // Only the last value uses println()
}



float pressureCtrl::ctrl_update(bool gripDetected) {
  getPressure();


  if (pressure < minPressure)  {
    error = minPressure - pressure;
    integral += error * 0.02; 
    integral = constrain(integral, -1, 1);
    output = Kp * error + Ki * integral;
    if (fabs(output) < (hold / 1024)) reset = false;
  } else if (gripDetected) {
    output = 0;

  } else {
    output = 0;
  }
  
  motorSpeed = constrain(output * 1024, -255, 255);


  if (fabs(motorSpeed) < hold) {
    ledcWrite(pwmChannel, 255);
    analogWrite(pumpPin, 0);
  } else if (motorSpeed > 0) {
    ledcWrite(pwmChannel, 255);
    analogWrite(pumpPin, (int)motorSpeed);
  } else {
    ledcWrite(pwmChannel, constrain(200 + (int)motorSpeed, 0, 255));
    analogWrite(pumpPin, 0);
  }

  return pressure - fabs(integral);

}
