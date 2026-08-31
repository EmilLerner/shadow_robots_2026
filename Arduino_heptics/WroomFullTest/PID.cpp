#include "PID.h"

#define pwmChannel 0
#define pwmFreq 5       // Hz — change to valve requirement
#define pwmResolution 8   // 0–255


PID::PID(int pressurePin,  int pumpPin, int valvePin, int hold, float Kp, float Ki)
    : pressurePin(pressurePin),
      pumpPin(pumpPin),
      valvePin(valvePin),
      hold(hold),
      Kp(Kp),
      Ki(Ki),
      minReading(20),
      maxReading(2096),
      reading(0) {  
}

void PID::PID_INIT() {
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(valvePin, pwmChannel);

  pinMode(pumpPin, OUTPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(pressurePin, ADC_0db);

}

// void PID::PID_MAXSET(int swtchPin) {


// }

void PID::PID_update(float setpoint) {
  reading = analogRead(pressurePin);
  pressure = (reading - minReading) /
                     (float)(maxReading - minReading);
  pressure = constrain(pressure, 0.0, 2.0);

  error = setpoint - pressure;
  integral += error * 0.02; 
  integral = constrain(integral, -1, 1);
  output = Kp * error + Ki * integral;
  motorSpeed = output * 1024;
  motorSpeed = constrain(motorSpeed, -255, 255);

  if(reading > maxReading) output = -1;

  if (fabs(motorSpeed) < hold) {
    ledcWrite(pwmChannel, 255);
    dacWrite(pumpPin, 0);
  } else if (motorSpeed > 0) {
    ledcWrite(pwmChannel, 255);
    dacWrite(pumpPin, (int)motorSpeed);
  } else {
    ledcWrite(pwmChannel, (int)(255 + motorSpeed));
    dacWrite(pumpPin, 0);
  }

}

void PID::PID_plot() {
  Serial.print(pressure);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.print(motorSpeed / 255);
  Serial.print(",");
  Serial.print(0.0);
  Serial.print(",");
  Serial.println(0.7);   // Only the last value uses println()
}
