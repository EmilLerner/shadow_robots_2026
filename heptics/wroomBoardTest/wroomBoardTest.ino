#define valvePin 25
#define pumpPin 26
#define joyPin 34
#define pressurePin 32

const int hold = 10;
const int pwmChannel = 0;
const int pwmFreq = 5;       // Hz — change to valve requirement
const int pwmResolution = 8;   // 0–255

void setup() {
  Serial.begin(115200);

  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(valvePin, pwmChannel);

  pinMode(pumpPin, OUTPUT);
  pinMode(joyPin, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(pressurePin, ADC_0db);

  // dacWrite(D25, 128);
  // dacWrite(D26, 200);


}

void loop() {
  int pressure = analogRead(pressurePin);
  float joyXraw = (float)analogRead(joyPin);

  float joyXscaled = (joyXraw - 1800) / 8;
  joyXscaled = constrain(joyXscaled, -255, 255);

  if (fabs(joyXscaled) < hold) {
    ledcWrite(pwmChannel, 255);
    dacWrite(pumpPin, 0);
  } else if (joyXscaled > 0) {
    ledcWrite(pwmChannel, 255);
    dacWrite(pumpPin, (int)joyXscaled);
  } else {
    ledcWrite(pwmChannel, (int)(225 + joyXscaled));
    dacWrite(pumpPin, 0);
  }

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.print("  , JoyX: ");
  Serial.println((int)joyXscaled);

  delay(100);
}