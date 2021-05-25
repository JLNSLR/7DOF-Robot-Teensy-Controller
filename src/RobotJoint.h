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
#include <RobotCANHandler.h>

#define CURRENT_FILTER_ORDER 4
#define POSITION_FILTER_ORDER 6
#define TORQUE_FILTER_ORDER 4

#define CONVERSION_FACTOR_13BITPOS_DEG 0.02197
#define CONVERSIoN_FACTOR_13BITPOS_RAD 0.0003834951
#define DEG2RAD 0.0017453
#define RAD2DEG 57.29578

#define POS_FREQ 300
#define TORQUE_FREQ 300

#define CURRENT_MOV_AVERAGE 10

#define POSITIVE_DIRECTION 1
#define NEGATIVE_DIRECTION 2

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

  bool sensorError = false;

  float maxSpeed;        // in deg/s
  float maxAcceleration; // in deg/s^2
  float maxTorque;
  float maxCurrent;

  CircularBuffer<float, 10> position;
  CircularBuffer<float, 10> velocity;
  CircularBuffer<float, 10> acceleration;
  CircularBuffer<float, 10> torque;
  CircularBuffer<float, 10> current;

  PIDController positionPID;
  PIDController velocityPID;
  PIDController currentPID;

  bool baseLineController = true;

  int16_t motorCommand = 0;
  float position_target = 0;
  float acceleration_target = 0;

  int16_t motorVoltage_feedforward = 0;
  float current_feedforward = 0;
  float velocity_feedforward = 0;

  void processCurrentController();
  void processVelocityController();
  void processPositionController();

  void processCascadeController();


  float torque_target;

  float limit_deadband = 0;
  float limit_slope_width = 15;

  static float a_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1];
  static float b_coefficients_currentFilter[CURRENT_FILTER_ORDER + 1];

  static float a_coefficients_positionFilter[POSITION_FILTER_ORDER + 1];
  static float b_coefficients_positionFilter[POSITION_FILTER_ORDER + 1];

  static float a_coefficients_torqueFilter[TORQUE_FILTER_ORDER + 1];
  static float b_coefficients_torqueFilter[TORQUE_FILTER_ORDER + 1];

  static const int checkEncoderErrorSampleTime = 50;

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

  void setAccelerationtarget(float acceleration_target);

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

  void setAngleDirection(int ange_direction);

  void setMotorCommandDirection(int motor_direction);

  float getAngleOffset();
  float getTorqueOffset();

  int currentCounter = 0;

  float acc2CurrentFactor = 100;

  void checkSensorError();

private:
  int lastTimePositionInput = 0;
  int lastTimeTorqueInput = 0;
  int lastTimeCurrentInput = 0;

  int lastTimeCurrentController = 0;
  int lastTimeVelocityController = 0;
  int lastTimePositionController = 0;

  int currentControllerSampleTime = 1000;
  int velocityControllerSampleTime = 3333;
  int positionControllerSampleTime = 6666;

  float torqueFactor = 0.01;

  const static int positionInputPeriod = 3333;
  const static int currentInputPeriod = 400;
  const static int torqueInputPeruid = 3333;

  float angle_offset = 0;
  float torque_offset = 0;

  float limit_neg = 0;
  float limit_pos = 0;

  float convertPositionInput(int16_t rawPosition);
  float convertTorqueInput(int32_t rawTorque);

  int angle_direction = 1;
  int command_direction = 1;

  Differentiator firstDerivative;
  Differentiator secondDerivative;

  CircularBuffer<float, CURRENT_MOV_AVERAGE> currentAverage;

  int hitJointLimit();
  int restrictOutputSignal(int16_t u);
};

#endif // ROBOTJOINT_H