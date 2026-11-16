#ifndef PRESSURECTRL_H
#define PRESSURECTRL_H

#include <Arduino.h>



// main class for the PID control aspect. Responsible for both reading presure 
// and running the ouptuts for simplicity 
class pressureCtrl {
private:
    const int pressurePin;
    const int pumpPin;
    const int valvePin;


    int minReading;
    int maxReading;
    int reading;

    int hold;

    float Kp;
    float Ki;

    float minPressure;
    float pressure;
    float error;
    float integral;
    float output;
    float motorSpeed;

    float pressureAdded;

    bool reset;




public:
    pressureCtrl(int pressurePin, int pumpPin, int valvePin, int hold, float Kp, float Ki, float minPressure);

    void ctrl_INIT();

    float getPressure();

    void ctrl_plot();

    float ctrl_update(bool gripDetected);
};

#endif