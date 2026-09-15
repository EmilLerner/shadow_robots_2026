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

    float pressure;
    float error;
    float integral;
    float output;
    float motorSpeed;




public:
    pressureCtrl(int pressurePin, int pumpPin, int valvePin, int hold, float Kp, float Ki);

    void ctrl_INIT();

    float getPressure();

    // void sensorRead();

    // void PID_MAXSET(int swtchPin);

    void ctrl_update(float setpoint);

    void ctrl_plot();
};

#endif