#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Arduino.h"

#define MANUAL 0
#define AUTOMATIC 1

#define DIRECT 0
#define REVERSE 1

/*
PID Controller class using the fix point math in signed 32-bit arithmetics
*/
class PIDController
{
public:
    PIDController();
    PIDController(float kp, float ki, float kd);

    void compute();
    void setTuning(float kp, float ki, float kd);

    void setSampleTime(float newSampleTime);

    void setOutputLimits(float min, float max);

    void setMode(int Mode);

    void Initialize();

    void SetControllerDirection(int Direction);

    /* ---  working variables --- */

    float kp, ki, kd;

    float input, output, setpoint;
    float iTerm, lastInput;

    int sampleTime; //microseconds

    unsigned long lastTime;

    float outMin, outMax;

    bool inAuto;

    int controllerDirection = DIRECT;
};

#endif //PIDCONTROLLER_H