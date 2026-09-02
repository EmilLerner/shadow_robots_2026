#include "motorCTR.h"

motorCTR *motorCTR::_instance = nullptr;


motorCTR::motorCTR()
{
    _instance = this;
}


void motorCTR::connect()
{
    /*
     * Initialise the LPF2 hub
     */
    _hub.init();

    delay(100);

    /*
     * Connect to the hub
     */
    while (!_hub.isConnected())
    {
        if (!_hub.isConnecting())
        {
            _hub.init();
            delay(100);
        }

        if (_hub.isConnecting())
        {
            _hub.connectHub();
        }

        delay(100);
    }

    /*
     * Give the connection a moment to settle
     */
    delay(500);

    /*
     * Activate motor
     */
    _hub.activatePortDevice(
        MOTOR_PORT,
        portCallback
    );

    delay(100);

    /*
     * Activate current sensor
     */
    _hub.activatePortDevice(
        CURRENT_PORT,
        portCallback
    );

    delay(500);
}


void motorCTR::turn(float speed)
{
    if (!_hub.isConnected())
    {
        return;
    }

    _hub.setTachoMotorSpeed(
        MOTOR_PORT,
        constrain(speed * 0.4, -100, 100)
    );
}



double motorCTR::getCurrent()
{
    return _current;
}


int motorCTR::getPosition()
{
    return _position;
}


bool motorCTR::isConnected()
{
    return _hub.isConnected();
}


void motorCTR::portCallback(
    void *hub,
    byte portNumber,
    DeviceType deviceType,
    uint8_t *pData)
{
    if (_instance == nullptr)
    {
        return;
    }

    /*
     * Make sure this callback belongs
     * to our hub.
     */
    if (hub != &_instance->_hub)
    {
        return;
    }

    _instance->handleCallback(
        portNumber,
        deviceType,
        pData
    );
}


void motorCTR::handleCallback(
    byte portNumber,
    DeviceType deviceType,
    uint8_t *pData)
{
    /*
     * Motor position
     */
    if (portNumber == MOTOR_PORT &&
        deviceType ==
            DeviceType::TECHNIC_LARGE_LINEAR_MOTOR)
    {
        _position = _hub.parseTachoMotor(pData);
    }

    /*
     * Current sensor
     */
    if (portNumber == CURRENT_PORT &&
        deviceType == DeviceType::CURRENT_SENSOR)
    {
        _current = _hub.parseCurrentSensor(pData);
    }
}