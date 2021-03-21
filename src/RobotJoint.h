#ifndef ROBOTJOINT_H
#define ROBOTJOINT_H

#include <MotorDrivers.h>
#include <CircularBuffer.h>
#include <PIDController.h>
#include <IIRFilter.h>
#include <RobotBuffers.h>
#include <Arduino.h>

#define CURRENT_FILTER_ORDER 4
#define POSITION_FILTER_ORDER 6

#define CONVERSION_FACTOR_13BITPOS_RADIAN 0.0003835
#define DEG2RAD 0.0017453
#define RAD2DEG 57.29578

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

    CircularBuffer<int32_t, 100> position;
    CircularBuffer<int32_t, 100> velocity;
    CircularBuffer<int32_t, 100> acceleration;
    CircularBuffer<int32_t, 100> torque;
    CircularBuffer<int32_t, 100> current;

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

    void processPositionInput();

    void setAngleOffsetRad(float angle_offset);
    void setAngleOffsetDeg(float angle_offset);
    void setTorqueOffsetNm(float torqueOffset);

    float getAngleRad();
    float getVelocityRad();
    float getAccelerationRad();

    float getAngleDeg();
    float getVelocityDeg();
    float getAccelerationDeg();

    float getTorque();
    float getCurrent();

private:
    int lastTimePositionInput = 0;
    int lastTimeTorqueInput = 0;
    int lastTimeCurrentInput = 0;

    static int positionInputPeriod = 3333;
    static int currentInputPeriod = 400;
    static int torqueInputPeruid = 3333;

    int32_t angle_offset = 0;
    int32_t torque_offset = 0;

    void calcSpeed();
    void calcAcceleration();

    int32_t convertPositionInput(int16_t rawPosition);
    int32_t convertTorqueInput(int32_t rawTorque);
};

#endif //ROBOTJOINT_H