#include <Manipulator.h>

Manipulator::Manipulator()
{

  // Init Joints with standard values
  for (int i = 0; i < 7; i++)
  {
    robotJoints[i].setAngleOffsetRad(angle_offsets[i]);
    robotJoints[i].setTorqueOffsetNm(torqueOffsets[i]);
  }
};

void Manipulator::initManipulator()
{

  // Set Motor and Current Sensor Ids
  robotJoints[0].joint_id = 0;
  robotJoints[0].motorControllerId = 0;
  robotJoints[0].currentSensorId = 0;

  robotJoints[1].joint_id = 1;
  robotJoints[1].motorControllerId = 1;
  robotJoints[1].currentSensorId = 1;

  robotJoints[2].joint_id = 2;
  robotJoints[2].motorControllerId = 2;
  robotJoints[2].currentSensorId = 2;

  robotJoints[3].joint_id = 3;
  robotJoints[3].motorControllerId = 3;
  robotJoints[3].currentSensorId = 3;

  robotJoints[4].joint_id = 4;
  robotJoints[4].motorControllerId = 4;
  robotJoints[4].currentSensorId = 4;

  robotJoints[5].joint_id = 5;
  robotJoints[5].motorControllerId = 5;
  robotJoints[5].currentSensorId = 5;

  robotJoints[6].joint_id = 6;
  robotJoints[6].motorControllerId = 6;
  robotJoints[6].currentSensorId = 6;

  // --- Read Initializaton Data from Memory
  readOffsetDataFromMemory();
  readGainDataFromMemory();
  readLimitDataFromMemory();
}

void Manipulator::processJointSensors()
{
  for (int i = 0; i < numberOfJoints; i++)
  {
    robotJoints[i].processCurrentInput();
    robotJoints[i].processPositionInput();
    robotJoints[i].processTorqueInput();
  }
}

void Manipulator::readOffsetDataFromMemory()
{
  struct RobotOffsets offsets;

  EEPROM.get(offset_addresses, offsets);

  if (offsets.validity == validity_key)
  {
    for (int i = 0; i < 7; i++)
    {
      robotJoints[i].setAngleOffsetRad(offsets.angleOffsets[i]);
      robotJoints[i].setTorqueOffsetNm(offsets.torqueOffsets[i]);
    }
  }
};

void Manipulator::readGainDataFromMemory()
{
  struct PositionControllerGains gains;

  EEPROM.get(controllerGain_address, gains);

  if (gains.validity == validity_key)
  {
    for (int i = 0; i < 7; i++)
    {
      robotJoints[i].positionController.kp =
          gains.positionGains[i].Kp;
      robotJoints[i].positionController.ki =
          gains.positionGains[i].Ki;
      robotJoints[i].positionController.kd =
          gains.positionGains[i].Kd;

      robotJoints[i].velocityController.kp =
          gains.velocityGains[i].Kp;
      robotJoints[i].velocityController.ki =
          gains.velocityGains[i].Ki;
      robotJoints[i].velocityController.kd =
          gains.velocityGains[i].Kd;

      robotJoints[i].currentController.kp =
          gains.currentGains[i].Kp;
      robotJoints[i].velocityController.ki =
          gains.velocityGains[i].Ki;
      robotJoints[i].velocityController.kd =
          gains.velocityGains[i].Kd;
    }
  }
}

void Manipulator::readLimitDataFromMemory()
{
  struct RobotLimits limits;

  EEPROM.get(limitAdresses, limits);

  if (limits.validity == validity_key)
  {
    for (int i = 0; i < 7; i++)
    {
      robotJoints[i].setPosLimits(limits.limits_r[i], limits.limits_l[i]);
    }
  }
}

void Manipulator::saveOffsetData()
{
  struct RobotOffsets offsetData;
  offsetData.validity = validity_key;
  for (int i = 0; i < 7; i++)
  {
    offsetData.angleOffsets[i] = angle_offsets[i];
  }
  EEPROM.put(offset_addresses, offsetData);
};

void Manipulator::saveLimitData()
{
  struct RobotLimits limits;
  limits.validity = validity_key;
  for (int i = 0; i < 7; i++)
  {
    limits.limits_r[i] = robotJoints[i].getLimitR();
    limits.limits_l[i] = robotJoints[i].getLimitL();
  }
  EEPROM.put(limitAdresses, limits);
};

void Manipulator::saveGainData()
{
  PositionControllerGains gainMemory;
  gainMemory.validity = validity_key;

  for (int i = 0; i < 7; i++)
  {
    gainMemory.positionGains[i].Kp =
        robotJoints[i].positionController.kp;
    gainMemory.positionGains[i].Ki =
        robotJoints[i].positionController.ki;
    gainMemory.positionGains[i].Kd =
        robotJoints[i].positionController.kd;

    gainMemory.velocityGains[i].Kp =
        robotJoints[i].velocityController.kp;
    gainMemory.velocityGains[i].Ki =
        robotJoints[i].velocityController.ki;
    gainMemory.velocityGains[i].Kd =
        robotJoints[i].velocityController.kd;

    gainMemory.currentGains[i].Kp =
        robotJoints[i].currentController.kp;
    gainMemory.currentGains[i].Ki =
        robotJoints[i].currentController.ki;
    gainMemory.currentGains[i].Kd =
        robotJoints[i].currentController.kd;
  }

  EEPROM.put(controllerGain_address, gainMemory);
}