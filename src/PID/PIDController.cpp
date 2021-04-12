#include "PIDController.h"

PIDController::PIDController() : kp(0), ki(0), kd(0), input(0), output(0), lastTime(0), iTerm(0), lastInput(0), sampleTime(1), inAuto(true){

                                                                                                                               };

PIDController::PIDController(float kp, float ki, float kd) : PIDController()
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
};

void PIDController::compute()
{

    if (!inAuto)
    {
        return;
    }

    /* Compute error variables */
    float error = setpoint - input;
    iTerm += ki * error;

    //Clamp iTerm against windup
    if (iTerm > outMax)
        iTerm = outMax;
    else if (iTerm < outMin)
        iTerm = outMin;

    float dInput = input - lastInput;

    /* Compute PID Output */

    output = kp * error + iTerm - kd * dInput;

    //Clamp output against windup
    if (output > outMax)
        output = outMax;
    else if (output < outMin)
        output = outMin;

    /* Save Variables for next step */
    lastInput = input;
};

void PIDController::computePeriodic()
{
    if (!inAuto)
        return;
    unsigned long now = micros();

    int timeChange = now - lastTime;

    if (timeChange >= sampleTime)
    {

        /* Compute error variables */
        float error = setpoint - input;
        iTerm += ki * error;

        //Clamp iTerm against windup
        if (iTerm > outMax)
            iTerm = outMax;
        else if (iTerm < outMin)
            iTerm = outMin;

        float dInput = input - lastInput;

        /* Compute PID Output */

        output = kp * error + iTerm - kd * dInput;

        //Clamp output against windup
        if (output > outMax)
            output = outMax;
        else if (output < outMin)
            output = outMin;

        /* Save Variables for next step */
        lastInput = input;
        lastTime = now;
    }
}

void PIDController::setTuning(float kp, float ki, float kd)
{

    if (kp < 0 || ki < 0 || kd < 0)
        return;

    float sampleTimeInSec = ((float)sampleTime / (1000 * 1000));

    kp = kp;
    ki = (int32_t)(((float)ki) * sampleTimeInSec);
    kd = (int32_t)(((float)kd) / sampleTimeInSec);

    if (controllerDirection == REVERSE)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
}

void PIDController::setSampleTime(int newSampleTime)
{

    if (newSampleTime > 0)
    {

        float ratio = (float)newSampleTime / (float)sampleTime;

        ki = (int32_t)((float)ki * ratio);
        kd = (int32_t)((float)kd / ratio);

        sampleTime = newSampleTime;
    }
}

void PIDController::setOutputLimits(float min, float max)
{

    if (min > max)
        return;
    outMin = min;
    outMax = max;

    if (output > outMax)
        output = outMax;
    else if (output < outMin)
        output = outMin;

    if (iTerm > outMax)
        iTerm = outMax;
    else if (iTerm < outMin)
        iTerm = outMin;
}

void PIDController::setMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto)
    { /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

void PIDController::Initialize()
{
    lastInput = input;
    iTerm = output;
    if (iTerm > outMax)
        iTerm = outMax;
    else if (iTerm < outMin)
        iTerm = outMin;
}

void PIDController::SetControllerDirection(int Direction)
{
    controllerDirection = Direction;
}
