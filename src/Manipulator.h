#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <RobotJoint.h>
#include <EEPROM.h>

struct PIDGains
{
  float Kp;
  float Ki;
  float Kd;
};

struct PositionControllerGains
{
  uint8_t validity;
  PIDGains positionGains[7];
  PIDGains velocityGains[7];
  PIDGains currentGains[7];
};

struct RobotOffsets
{
  uint8_t validity;
  float angleOffsets[7];
  float torqueOffsets[7];
};

struct RobotLimits
{
  uint8_t validity;
  float limits_r[7];
  float limits_l[7];
};

class Manipulator
{

public:
  Manipulator();

  Manipulator(const Manipulator &) = delete;

  static const int numberOfJoints = 7;

  RobotJoint robotJoints[numberOfJoints];
  // --- Default Offsets --- //
  float angle_offsets[numberOfJoints] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float torqueOffsets[numberOfJoints] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  void initManipulator();

  void processJointSensors();

  void saveOffsetData();
  void saveGainData();
  void saveLimitData();

private:
  const int offset_addresses = 0;
  const int limitAdresses = offset_addresses + sizeof(RobotOffsets);
  const int controllerGain_address = limitAdresses + sizeof(RobotLimits);
  const int validity_key = 17;
  void readOffsetDataFromMemory();
  void readGainDataFromMemory();
  void readLimitDataFromMemory();
};

#endif // MANIPULATOR_H