#ifndef MOTOR_CTR_H
#define MOTOR_CTR_H

#include "Lpf2Hub.h"

class motorCTR
{
public:
    motorCTR();

    void connect();
    void turn(float speed);
    void moveToPosition(int position, float speed);

    double getCurrent();
    int getPosition();

    bool isConnected();

private:
    Lpf2Hub _hub;

    static const byte MOTOR_PORT =
        (byte)PoweredUpHubPort::B;

    static const byte CURRENT_PORT = 59;

    volatile int _position = 0;
    volatile double _current = 0.0;

    static motorCTR *_instance;

    static void portCallback(
        void *hub,
        byte portNumber,
        DeviceType deviceType,
        uint8_t *pData);

    void handleCallback(
        byte portNumber,
        DeviceType deviceType,
        uint8_t *pData);
};

#endif