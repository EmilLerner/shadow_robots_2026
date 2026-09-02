#ifndef PID_H
#define PID_H

#include <Arduino.h>


// main class for the PID control aspect. Responsible for both reading presure 
// and running the ouptuts for simplicity 
class PID {
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
    PID(int pressurePin, int pumpPin, int valvePin, int hold, float Kp, float Ki);

    void PID_INIT();

    // void sensorRead();

    // void PID_MAXSET(int swtchPin);

    void PID_update(float setpoint);

    void PID_plot();
};

#endif