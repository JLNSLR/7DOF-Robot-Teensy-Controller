#include <RobotJoint.h>

// --- Filter Coefficients --- //
float RobotJoint::a_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1] = {
    1, -2.048395137764509, 1.841785841678162, -0.782440103120936, 0.131680715038692};
float RobotJoint::b_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1] = {
    0.008914457239463, 0.035657828957852, 0.053486743436778, 0.035657828957852, 0.008914457239463};

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
}

void RobotJoint::initRobotJoint()
{
  currentFilter.setCoefficients(a_coefficients_currentFilter,
                                b_coefficients_currentFilter);
  positionFilter.setCoefficients(a_coefficients_positionFilter,
                                 b_coefficients_positionFilter);
  torqueFilter.setCoefficients(a_coefficients_torqueFilter, b_coefficients_torqueFilter);

  for(int i = 0; i<20;i++){
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
        Serial.println("JOINT LIMIT HIT");
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
    currentAverage.unshift(currentFilter.output);

    float averageValue = 0;
    for (int i = 0; i < 10; i++)
    {
      averageValue += currentAverage[i];
    }
    current.unshift(averageValue / 10);
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
    if (hitJointLimit())
    {
      controlMotorDriver(motorControllerId, 0);
      return;
    }
  }
  controlMotorDriver(motorControllerId, motorCommand);
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
  return (float)(double(CONVERSION_FACTOR_13BITPOS_DEG) * double(rawPosition)) -
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

float RobotJoint::getAngleDeg() { return position.last() * RAD2DEG; }
float RobotJoint::getVelocityDeg()
{
  return velocity.last() * RAD2DEG;
}
float RobotJoint::getAccelerationDeg()
{
  return acceleration.last() * RAD2DEG;
}

float RobotJoint::getTorque() { return torque.last(); }
float RobotJoint::getCurrent() { return current.last(); }

float RobotJoint::getLimitR() { return limit_right; };
float RobotJoint::getLimitL() { return limit_left; }

bool RobotJoint::hitJointLimit()
{
  float pos = position.last();

  if ((pos < limit_left) || (pos > limit_right))
  {
    Serial.print("Hit Joint Limit: ");
    Serial.println(joint_id);
    return true;
  }
  return false;
}