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
      minReading(100),
      maxReading(4095),
      reading(0) {  
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

// void PID::PID_MAXSET(int swtchPin) {


// }

float pressureCtrl::getPressure() {
  reading = analogRead(pressurePin);
  pressure = 2.0f * (reading - minReading) /
                     (float)(maxReading - minReading);
  // pressure = constrain(pressure, 0.0, 2.0);
  Serial.print("Pressure: ");
  Serial.println(reading);
  return pressure;
}

float pressureCtrl::ctrl_update(float setpoint) {
  getPressure();

  error = setpoint - pressure;
  integral += error * 0.02; 
  integral = constrain(integral, -1, 1);
  output = Kp * error + Ki * integral;
  if ((error < 0.01) )  {
    integral = 0;
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

  return pressure - integral;

}


float pressureCtrl::ctrl_updateSum(float addedPressure, float goalPressure) {
  if (goalPressure < 10) goalPressure = 0;
  getPressure();
  
  float setpoint = constrain(pressure + addedPressure, 0.03, 1.5);
  error = setpoint - pressure;
  integral += error * 0.01; 
  integral = constrain(integral, -1.5, 1.5);
  output = Kp * error + Ki * integral;
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

  float inputPressure = pressure - integral;
  return inputPressure;
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



float pressureCtrl::ctrl_update(float setpoint, bool gripDetected) {
  getPressure();

  if (gripDetected) {
    setpoint = setpoint * 2;
  } else {
    setpoint = setpoint / 2 ;
  }

  error = setpoint - pressure;
  integral += error * 0.02; 
  integral = constrain(integral, -1, 1);
  output = Kp * error + Ki * integral;



  // if (!gripDetected) {
  //   output = output / 2;
  // }

  if (pressure < minPressure) {
    error = minPressure - pressure;
    integral += error * 0.02; 
    integral = constrain(integral, -1, 1);
    output = Kp * error + Ki * integral;
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

  return pressure - integral;

}
