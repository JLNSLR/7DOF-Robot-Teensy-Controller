#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "Arduino.h"
#include "Math/FixPointMath.h"

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
    PIDController(int32_t kp, int32_t ki, int32_t kd);

    void compute();
    void setTuning(int32_t kp, int32_t ki, int32_t kd);

    void setSampleTime(int32_t newSampleTime);

    void setOutputLimits(int32_t min, int32_t max);

    void setMode(int Mode);

    void Initialize();

    void SetControllerDirection(int Direction);

    /* ---  working variables --- */
    

    int32_t kp, ki, kd;


    int32_t input, output, setpoint;
    int32_t iTerm, lastInput;

    int32_t sampleTime;

    unsigned long lastTime;

    int32_t outMin, outMax;

    bool inAuto;

    int controllerDirection = DIRECT;

    
};


#endif //PIDCONTROLLER_H