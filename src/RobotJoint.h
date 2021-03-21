#ifndef ROBOTJOINT_H
#define ROBOTJOINT_H

#include <MotorDrivers.h>
#include <CircularBuffer.h>
#include <PIDController.h>
#include <IIRFilter.h>

#define CURRENT_FILTER_ORDER 4
#define POSITION_FILTER_ORDER 6

class RobotJoint
{
public:
    RobotJoint();

    uint8_t joint_id;
    uint8_t motorControllerId;
    uint8_t currentSensorId;

    int16_t limit_left;
    int16_t limit_right;

    float maxSpeed;        // in rad/s
    float maxAcceleration; // in rad/s^2
    float maxTorque;
    float maxCurrent;

    CircularBuffer<int16_t, 100> position;
    CircularBuffer<int16_t, 100> velocity;
    CircularBuffer<int16_t, 100> acceleration;
    CircularBuffer<int32_t, 100> torque;
    CircularBuffer<int16_t, 100> current;

    PIDController positionController;
    PIDController velocityController;
    PIDController currentController;

    int16_t motorCommand;
    int32_t velocity_target;
    int32_t posiion_target;
    int32_t current_target;

    int32_t torque_target;

    static float a_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1];
    static float b_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1];

    static float a_coefficients_positionFilter[POSITION_FILTER_ORDER + 1];
    static float b_coefficients_positionFilter[POSITION_FILTER_ORDER + 1];

    IIRFilter<CURRENT_FILTER_ORDER> currentFilter;
    IIRFilter<POSITION_FILTER_ORDER> positonFilter;

    void drive(int16_t motorCommand);
};

#endif //ROBOTJOINT_H