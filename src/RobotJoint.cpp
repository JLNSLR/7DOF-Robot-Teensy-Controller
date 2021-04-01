#include <RobotJoint.h>

// --- Filter Coefficients --- //
float RobotJoint::a_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1] = {
    1, 2.369513007182038, 2.313988414415880, 1.054665405878567,
    0.187379492368185};
float RobotJoint::b_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1] = {
    0.432846644990292, 1.731386579961168, 2.597079869941751, 1.731386579961168,
    0.432846644990292};

float RobotJoint::a_coefficients_positionFilter[POSITION_FILTER_ORDER + 1] = {1,-1.981610736719567,2.252379618156809,-1.469282954186778,0.596262579806017,-0.135440342911379,0.013563684610107};
float RobotJoint::b_coefficients_positionFilter[POSITION_FILTER_ORDER + 1] = {0.004310497636800,0.025862985820801,0.064657464552002,0.086209952736003,0.064657464552002,0.025862985820801,0.004310497636800};

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

  firstDerivative.setFrequency(POS_FREQ);
  secondDerivative.setFrequency(POS_FREQ);
}

// --- Methods to process Sensor Inputs --- //
void RobotJoint::processPositionInput()
{
  if (micros() - lastTimePositionInput > positionInputPeriod)
  {
    lastTimePositionInput = micros();
    Serial.print("Raw pos: ");
    Serial.println(jointPositionInput[joint_id].last());

    //positionFilter.setInput(convertPositionInput(jointPositionInput[joint_id].pop()));
    positionFilter.input = convertPositionInput(jointPositionInput[joint_id].pop());
    Serial.print("Converted: ");
    Serial.println(positionFilter.input);
    positionFilter.compute();
    Serial.print("Filtered: ");
    Serial.println(positionFilter.output);
    position.unshift(positionFilter.output);

    firstDerivative.setInput(positionFilter.output);
    firstDerivative.differentiate();
    velocity.unshift(firstDerivative.getOutput());
    secondDerivative.setInput(firstDerivative.getOutput());
    secondDerivative.differentiate();
    acceleration.unshift(secondDerivative.getOutput());

    if (posLimit)
    {
      if ((position.first() < limit_left) || (position.first() > limit_right))
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
  return float(torqueFactor * rawTorque) - torque_offset;
}

void RobotJoint::setAngleOffsetRad(float angle_offset)
{
  this->angle_offset = angle_offset;
}
void RobotJoint::setAngleOffsetDeg(float angle_offset)
{
  this->angle_offset = angle_offset * DEG2RAD;
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