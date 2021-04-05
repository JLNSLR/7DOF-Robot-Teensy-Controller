#ifndef ROBOTJOINT_H
#define ROBOTJOINT_H

#include <MotorDrivers.h>
#include <CircularBuffer.h>
#include <PID/PIDController.h>
#include <Filters/IIRFilter.h>
#include <RobotBuffers.h>
#include <Math/differentiator.h>
#include <Arduino.h>
#include <EEPROM.h>

#define CURRENT_FILTER_ORDER 4
#define POSITION_FILTER_ORDER 6
#define TORQUE_FILTER_ORDER 4

#define CONVERSION_FACTOR_13BITPOS_DEG 0.02197
#define CONVERSIoN_FACTOR_13BITPOS_RAD 0.0003834951
#define DEG2RAD 0.0017453
#define RAD2DEG 57.29578

#define POS_FREQ 300
#define TORQUE_FREQ 300

class RobotJoint
{
public:
  RobotJoint();

  RobotJoint(const RobotJoint &) = delete;

  uint8_t joint_id;
  uint8_t motorControllerId;
  uint8_t currentSensorId;

  bool posLimit = false;
  bool speedLimit = false;
  bool currentLimit = false;
  bool torqueLimit = false;

  float maxSpeed;        // in deg/s
  float maxAcceleration; // in deg/s^2
  float maxTorque;
  float maxCurrent;

  CircularBuffer<float, 5> position;
  CircularBuffer<float, 5> velocity;
  CircularBuffer<float, 5> acceleration;
  CircularBuffer<float, 5> torque;
  CircularBuffer<float, 5> current;

  PIDController positionController;
  PIDController velocityController;
  PIDController currentController;

  int16_t motorCommand;
  float velocity_target;
  float posiion_target;
  float current_target;
  float acceleration_target;

  float torque_target;

  static float a_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1];
  static float b_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1];

  static float a_coefficients_positionFilter[POSITION_FILTER_ORDER + 1];
  static float b_coefficients_positionFilter[POSITION_FILTER_ORDER + 1];

  static float a_coefficients_torqueFilter[TORQUE_FILTER_ORDER + 1];
  static float b_coefficients_torqueFilter[TORQUE_FILTER_ORDER + 1];

  IIRFilter<CURRENT_FILTER_ORDER> currentFilter;
  IIRFilter<POSITION_FILTER_ORDER> positionFilter;

  IIRFilter<TORQUE_FILTER_ORDER> torqueFilter;

  void drive(int16_t motorCommand);

  void processPositionInput();
  void processCurrentInput();
  void processTorqueInput();

  void initRobotJoint();

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

  void setPosLimits(float limit_right, float limit_left);
  float getLimitR();
  float getLimitL();

private:
  int lastTimePositionInput = 0;
  int lastTimeTorqueInput = 0;
  int lastTimeCurrentInput = 0;

  float torqueFactor = 0.01;

  const static int positionInputPeriod = 3333;
  const static int currentInputPeriod = 400;
  const static int torqueInputPeruid = 3333;

  float angle_offset = 0;
  float torque_offset = 0;

  float limit_left = 0;
  float limit_right = 0;

  float convertPositionInput(int16_t rawPosition);
  float convertTorqueInput(int32_t rawTorque);

  Differentiator firstDerivative;
  Differentiator secondDerivative;

  CircularBuffer<float, 10> currentAverage;

  bool hitJointLimit();
};

#endif // ROBOTJOINT_H