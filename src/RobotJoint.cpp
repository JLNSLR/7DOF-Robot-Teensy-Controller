#include <RobotJoint.h>

// --- Filter Coefficients --- //
float RobotJoint::a_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1] = {
    1, -0.782095198023338, 0.679978526916300, -0.182675697753032, 0.030118875043169};
float RobotJoint::b_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1] = {
    0.046582906636444, 0.186331626545775, 0.279497439818662, 0.186331626545775, 0.046582906636444};

float RobotJoint::a_coefficients_positionFilter[POSITION_FILTER_ORDER + 1] = {1, -2.778697797137459, 3.695151114840509, -2.808948781412266, 1.269735725150560, -0.319109994335365, 0.034611206062462};
float RobotJoint::b_coefficients_positionFilter[POSITION_FILTER_ORDER + 1] = {0.001449085518257, 0.008694513109541, 0.021736282773853, 0.028981710365138, 0.021736282773853, 0.008694513109541, 0.001449085518257};

float RobotJoint::a_coefficients_torqueFilter[TORQUE_FILTER_ORDER + 1] = {1, -2.638627743891246, 2.769309786151484, -1.339280761265202, 0.249821669810126};
float RobotJoint::b_coefficients_torqueFilter[TORQUE_FILTER_ORDER + 1] = {0.002576434425323, 0.010305737701291, 0.015458606551936, 0.010305737701291, 0.002576434425323};
// --- Constructors --- //
RobotJoint::RobotJoint()
{
  currentFilter.setCoefficients(a_coefficients_currentFilter,
                                b_coefficients_currentFilter);
  positionFilter.setCoefficients(a_coefficients_positionFilter,
                                 b_coefficients_positionFilter);

  firstDerivative.setFrequency(POS_FREQ);
  secondDerivative.setFrequency(POS_FREQ);

  currentPID.setSampleTime(currentControllerSampleTime);
  currentPID.setOutputLimits(-4096, 4096);
  currentPID.setMode(AUTOMATIC);

  velocityPID.setSampleTime(velocityControllerSampleTime);
  velocityPID.setOutputLimits(-10000, 10000);
  velocityPID.setMode(AUTOMATIC);

  positionPID.setSampleTime(positionControllerSampleTime);
  positionPID.setOutputLimits(-360, 360);
  velocityPID.setMode(AUTOMATIC);
}

void RobotJoint::initRobotJoint()
{
  currentFilter.setCoefficients(a_coefficients_currentFilter,
                                b_coefficients_currentFilter);
  positionFilter.setCoefficients(a_coefficients_positionFilter,
                                 b_coefficients_positionFilter);
  torqueFilter.setCoefficients(a_coefficients_torqueFilter, b_coefficients_torqueFilter);

  for (int i = 0; i < 20; i++)
  {
    currentAverage.unshift(0);
  }
}

// --- Methods to process Sensor Inputs --- //
void RobotJoint::processPositionInput()
{
  if (micros() - lastTimePositionInput > positionInputPeriod)
  {
    lastTimePositionInput = micros();
    positionFilter.input = convertPositionInput(jointPositionInput[joint_id].shift());
    positionFilter.compute();
    position.unshift(positionFilter.output);
    //Serial.println(positionFilter.output);

    firstDerivative.setInput(positionFilter.output);
    firstDerivative.differentiate();
    velocity.unshift(firstDerivative.getOutput());
    //Serial.println(firstDerivative.getOutput());
    secondDerivative.setInput(firstDerivative.getOutput());
    secondDerivative.differentiate();
    acceleration.unshift(secondDerivative.getOutput());

    if (posLimit)
    {
      if (hitJointLimit())
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
    currentFilter.input = readCurrentSensor(currentSensorId);
    currentFilter.compute();
    current.unshift(currentFilter.output);

    /*
    float averageValue = 0;
    for (int i = 0; i < CURRENT_MOV_AVERAGE; i++)
    {
      averageValue += currentAverage[i];
    }
    current.unshift(averageValue / CURRENT_MOV_AVERAGE);
    */
    currentCounter++;
  }
}
void RobotJoint::processTorqueInput()
{
  if (micros() - lastTimeTorqueInput > currentInputPeriod)
  {
    lastTimeTorqueInput = micros();
    torqueFilter.input = (jointTorqueInput[joint_id].shift());
    torqueFilter.compute();
    torque.unshift(torqueFilter.output);
  }
}

/* --- Methods to control the Joint Actuation --- */
void RobotJoint::drive(int16_t motorCommand)
{
  if (posLimit)
  {
    int jointLimitHit = hitJointLimit();
    if (jointLimitHit == POSITIVE_DIRECTION && motorCommand < 0 || jointLimitHit == NEGATIVE_DIRECTION && motorCommand > 0)
    {
      controlMotorDriver(motorControllerId, 0);
      return;
    }
  }
  controlMotorDriver(motorControllerId, motorCommand * command_direction);
}

void RobotJoint::processCurrentController()
{
  if (micros() - lastTimeCurrentController > currentControllerSampleTime)
  {
    lastTimeCurrentController = micros();
    currentPID.input = current.last();
    currentPID.compute();
    motorCommand = (int16_t)round(currentPID.output) + motorVoltage_feedforward;
    drive(motorCommand);
  }
}

void RobotJoint::processVelocityController()
{
  if (micros() - lastTimeVelocityController > velocityControllerSampleTime)
  {
    lastTimeVelocityController = micros();
    velocityPID.input = velocity.last();
    velocityPID.compute();
    currentPID.setpoint = velocityPID.output + current_feedforward;
  }
}

void RobotJoint::processPositionController()
{
  if (micros() - lastTimePositionController > positionControllerSampleTime)
  {
    lastTimePositionController = micros();
    positionPID.setpoint = position_target;
    positionPID.input = position.last();
    positionPID.compute();
    velocityPID.setpoint = positionPID.output + velocity_feedforward;
  }
}

void RobotJoint::setPosLimits(float limit_right, float limit_left)
{
  posLimit = true;
  this->limit_left = limit_left;
  this->limit_right = limit_right;

  if (limit_right == 0 && limit_left == 0)
  {
    posLimit = false;
  }
}

/* --- Implementation of Utility SensorDataHandling Methods --- */

float RobotJoint::convertPositionInput(int16_t rawPosition)
{
  return (float)(double(CONVERSION_FACTOR_13BITPOS_DEG) * double(rawPosition)) * angle_direction -
         angle_offset;
}

float RobotJoint::convertTorqueInput(int32_t rawTorque)
{
  return (float)torqueFactor * (float)rawTorque - torque_offset;
}

void RobotJoint::setAngleOffsetRad(float angle_offset)
{
  this->angle_offset = angle_offset;
}
void RobotJoint::setAngleOffsetDeg(float angle_offset)
{
  this->angle_offset = angle_offset;
}
void RobotJoint::setTorqueOffsetNm(float torqueOffset)
{
  torque_offset = torqueOffset;
}

float RobotJoint::getAngleRad() { return position.last(); }
float RobotJoint::getVelocityRad() { return velocity.last(); }
float RobotJoint::getAccelerationRad()
{
  return acceleration.last();
}

float RobotJoint::getAngleDeg() { return position.last(); }
float RobotJoint::getVelocityDeg()
{
  return velocity.last();
}
float RobotJoint::getAccelerationDeg()
{
  return acceleration.last();
}

float RobotJoint::getTorque() { return torque.last(); }
float RobotJoint::getCurrent() { return current.last(); }

float RobotJoint::getLimitR() { return limit_right; };
float RobotJoint::getLimitL() { return limit_left; }

int RobotJoint::hitJointLimit()
{
  float pos = position.last();

  if (pos < limit_left)
  {
    //Serial.print("#Hit Joint Limit: ");
    //Serial.println(joint_id);
    return NEGATIVE_DIRECTION;
  }
  else if (pos > limit_right)
  {
    //Serial.print("#Hit Joint Limit: ");
    //Serial.println(joint_id);
    return POSITIVE_DIRECTION;
  }
  return -1;
}

void RobotJoint::setAngleDirection(int angle_direction)
{
  if (angle_direction == 1 || angle_direction == -1)
  {
    this->angle_direction = angle_direction;
  }
}

void RobotJoint::setMotorCommandDirection(int motor_direction)
{
  if (motor_direction == 1 || motor_direction == -1)
  {
    this->command_direction = motor_direction;
  }
}

float RobotJoint::getAngleOffset()
{
  return angle_offset;
}
float RobotJoint::getTorqueOffset()
{
  return torque_offset;
}

void RobotJoint::setAccelerationtarget(float acceleration_target)
{
  current_feedforward = acceleration_target * acc2CurrentFactor;
}