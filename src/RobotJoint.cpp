#include <RobotJoint.h>
#include <FixPointMath.h>

// --- Filter Coefficients --- //
float RobotJoint::a_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1] = {1, 2.369513007182038, 2.313988414415880, 1.054665405878567, 0.187379492368185};
float RobotJoint::b_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1] = {0.432846644990292, 1.731386579961168, 2.597079869941751, 1.731386579961168, 0.432846644990292};

float RobotJoint::a_coefficients_positionFilter[POSITION_FILTER_ORDER + 1] = {0.816751544997935, 4.900509269987608, 12.251273174969020, 16.335030899958692, 12.251273174969020, 4.900509269987608, 0.816751544997935};
float RobotJoint::b_coefficients_positionFilter[POSITION_FILTER_ORDER + 1] = {1, 5.595430092802001, 13.058177871415467, 16.268238545772697, 11.410789023781913, 4.272380259839228, 0.667083086256514};

// --- Constructors --- //
RobotJoint::RobotJoint()
{
    currentFilter.setCoefficients(a_coefficients_currentFilter, b_coefficients_currentFilter);
    positionFilter.setCoefficients(a_coefficients_positionFilter, b_coefficients_positionFilter);

    firstDerivative.setFrequency(POS_FREQ);
    secondDerivative.setFrequency(POS_FREQ);
}

// --- Methods to process Sensor Inputs --- //
void RobotJoint::processPositionInput()
{
    if (micros() - lastTimePositionInput > positionInputPeriod)
    {
        lastTimePositionInput = micros();
        positionFilter.input = convertPositionInput(jointPositionInput[joint_id].pop());
        positionFilter.compute();
        position.unshift(positionFilter.output);

        firstDerivative.setInput(positionFilter.output);
        firstDerivative.differentiate();
        velocity.unshift(firstDerivative.getOutput());
        secondDerivative.setInput(firstDerivative.getOutput());
        secondDerivative.differentiate();
        acceleration.unshift(secondDerivative.getOutput());

        if (posLimit)
        {
            if ((position.first() < limit_left) ^ (position.first() > limit_right))
            {
                drive(0);
            }
        }
    }
}

void RobotJoint::processCurrentInput()
{
    if (micros() - lastTimeCurrentInput > currentInputPeriod)
    {
        lastTimeCurrentInput = micros();
        currentFilter.input = jointCurrentInput[joint_id].pop();
        currentFilter.compute();
        current.unshift(currentFilter.output);
    }
}
void RobotJoint::processTorqueInput()
{
    if (micros() - lastTimeTorqueInput > currentInputPeriod)
    {
        lastTimeTorqueInput = micros();
        torque.unshift(convertTorqueInput(jointTorqueInput[joint_id].pop()));
    }
}

/* --- Methods to control the Joint Actuation --- */
void RobotJoint::drive(int16_t motorCommand)
{
    controlMotorDriver(joint_id, motorCommand);
}

void RobotJoint::setPosLimits(float limit_right, float limit_left)
{
    posLimit = true;
    this->limit_left = float2Fix(limit_left);
    this->limit_right = float2Fix(limit_right);

    if (limit_right == 0 && limit_left == 0)
    {
        posLimit = false;
    }
}

/* --- Implementation of Utility SensorDataHandling Methods --- */

int32_t RobotJoint::convertPositionInput(int16_t rawPosition)
{
    return float2Fix(CONVERSION_FACTOR_13BITPOS_RADIAN * rawPosition) - angle_offset;
}

int32_t RobotJoint::convertTorqueInput(int32_t rawTorque)
{
    return rawTorque; //TODO
}

void RobotJoint::setAngleOffsetRad(float angle_offset)
{
    angle_offset = float2Fix(angle_offset);
}
void RobotJoint::setAngleOffsetDeg(float angle_offset)
{
    angle_offset = float2Fix(angle_offset * DEG2RAD);
}
void RobotJoint::setTorqueOffsetNm(float torqueOffset)
{
    torque_offset = float2Fix(torqueOffset);
}

float RobotJoint::getAngleRad()
{
    return fix2Float(position.last());
}
float RobotJoint::getVelocityRad()
{
    return fix2Float(velocity.last());
}
float RobotJoint::getAccelerationRad()
{
    return fix2Float(acceleration.last());
}

float RobotJoint::getAngleDeg()
{
    return fix2Float(position.last()) * RAD2DEG;
}
float RobotJoint::getVelocityDeg()
{
    return fix2Float(velocity.last()) * RAD2DEG;
}
float RobotJoint::getAccelerationDeg()
{
    return fix2Float(acceleration.last()) * RAD2DEG;
}

float RobotJoint::getTorque()
{
    return fix2Float(torque.last());
}
float RobotJoint::getCurrent()
{
    return fix2Float(current.last());
}