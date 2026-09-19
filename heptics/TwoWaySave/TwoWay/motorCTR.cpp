#include "motorCTR.h"

motorCTR *motorCTR::_instance = nullptr;


motorCTR::motorCTR() {
    _instance = this;
}


void motorCTR::connect() {

    // Initialise the LPF2 hub
    _hub.init();
    delay(100);

    // connecting to the hub
    while (!_hub.isConnected()) {
        if (!_hub.isConnecting()) {
            _hub.init();
            delay(100);
        }
        if (_hub.isConnecting()) {
            _hub.connectHub();
        }
        delay(100);
    }

    delay(500); 

    //Activate motor and current sensor
    _hub.activatePortDevice( MOTOR_PORT, portCallback);
    delay(100);
    _hub.activatePortDevice( CURRENT_PORT, portCallback);
    delay(500);
}

void motorCTR::position_INIT() {
    if (!_hub.isConnected()) return;


    turn(30);
    delay(500);

    getPosition();

    turn(-30);
    delay(500);
    getPosition();
    
    Serial.print("Position Max:  ");
    Serial.print(maxPosition);
    Serial.print("Position Mix:  ");
    Serial.println(minPosition);
    moveToPosition(0, 100);

}

void motorCTR::turn(float speed) {
    if (!_hub.isConnected()) return;
    _hub.setTachoMotorSpeed( MOTOR_PORT,constrain(speed, -100, 100));
}

void motorCTR::moveToPosition(float position, float speed) {
    if (!_hub.isConnected()) return;
    position = constrain(95 - position, -5, 90); 

    int32_t positionAbsolue = (int32_t) ((float)minPosition + (((float)maxPosition - (float)minPosition) * position / 100.0f));
    _hub.setAbsoluteMotorPosition(MOTOR_PORT, constrain(speed, 0, 100), positionAbsolue);
    getPosition();
}


double motorCTR::getCurrent() {
    return _current;
}


int motorCTR::getPosition() {
    if (_position > maxPosition) maxPosition = _position;
    if (_position < minPosition) minPosition = _position;
    return _position;
}


bool motorCTR::isConnected() {
    return _hub.isConnected();
}


void motorCTR::portCallback(
    void *hub,
    byte portNumber,
    DeviceType deviceType,
    uint8_t *pData)
{
    if (_instance == nullptr) return;
    if (hub != &_instance->_hub) return;
    _instance->handleCallback( portNumber, deviceType, pData);
}


void motorCTR::handleCallback(
    byte portNumber,
    DeviceType deviceType,
    uint8_t *pData)
{

    // Motor position
    if (portNumber == MOTOR_PORT && deviceType == DeviceType::TECHNIC_LARGE_LINEAR_MOTOR)   {
        _position = _hub.parseTachoMotor(pData);
    }


    // Current sensor
    if (portNumber == CURRENT_PORT && deviceType == DeviceType::CURRENT_SENSOR)    {
        _current = _hub.parseCurrentSensor(pData);
    }
}