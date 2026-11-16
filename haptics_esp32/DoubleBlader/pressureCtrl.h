#ifndef PRESSURECTRL_H
#define PRESSURECTRL_H

#include <Arduino.h>



// main class for the PID control aspect. Responsible for both reading presure 
// and running the ouptuts for simplicity 
class pressureCtrl {
private:
    const int pressurePin;
    const int pumpPin;
    const int drainPin;

    const int drainChannel;
    const int drainFreq;


    int minReading;
    int maxReading;
    int reading;

    int hold;

    float Kp;
    float Ki;

    float minPressure;
    float maxPressure;
    float pressure;
    float error;
    float integral;
    float output;
    float motorSpeed;





public:
    pressureCtrl(int pressurePin, int pumpPin, int valvePin, int drainChannel,int drainFreq, int hold, float Kp, float Ki, float minPressure);

    void ctrl_INIT();

    float getPressure();

    void ctrl_plot();

    float ctrl_maintain();

    float ctrl_set(float setPoint);
};

#endif