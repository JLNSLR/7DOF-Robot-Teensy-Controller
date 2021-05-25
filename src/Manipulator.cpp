#include <Manipulator.h>
#include <math.h>

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
  float standard_offset_J0 = 90.4;
  robotJoints[0].setAngleOffsetDeg(standard_offset_J0);

  robotJoints[1].joint_id = 1;
  robotJoints[1].motorControllerId = 1;
  robotJoints[1].currentSensorId = 1;

  float standard_offset_J1 = -151.65;
  robotJoints[1].setAngleOffsetDeg(standard_offset_J1);

  robotJoints[2].joint_id = 2;
  robotJoints[2].motorControllerId = 2;
  robotJoints[2].currentSensorId = 2;

  float standard_offset_J2 = -36.5;
  robotJoints[2].setAngleOffsetDeg(standard_offset_J2);
  robotJoints[2].setPosLimits(0, 0);
  robotJoints[2].setMotorCommandDirection(-1);

  robotJoints[3].joint_id = 3;
  robotJoints[3].motorControllerId = 3;
  robotJoints[3].currentSensorId = 3;

  float standard_offset_J3 = -114.45;
  robotJoints[3].setAngleOffsetDeg(standard_offset_J3);

  robotJoints[4].joint_id = 4;
  robotJoints[4].motorControllerId = 6;
  robotJoints[4].currentSensorId = 4;

  float standard_offset_J4 = -63;
  robotJoints[4].setAngleOffsetDeg(standard_offset_J4);
  //robotJoints[4].setPosLimits(0, 0);

  robotJoints[5].joint_id = 5;
  robotJoints[5].motorControllerId = 4;
  robotJoints[5].currentSensorId = 5;

  float standard_offset_J5 = -103.9;
  robotJoints[5].setAngleOffsetDeg(standard_offset_J5);

  robotJoints[6].joint_id = 6;
  robotJoints[6].motorControllerId = 5;
  robotJoints[6].currentSensorId = 6;

  float standard_offset_J6 = 0;
  robotJoints[6].setAngleOffsetDeg(standard_offset_J6);
  robotJoints[6].setAngleDirection(1);
  robotJoints[6].setMotorCommandDirection(-1);

  for (int i = 0; i < numberOfJoints; i++)
  {
    robotJoints[i].initRobotJoint();
  }

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

   // robotJoints[i].checkSensorError();
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
      robotJoints[i].positionPID.kp =
          gains.positionGains[i].Kp;
      robotJoints[i].positionPID.ki =
          gains.positionGains[i].Ki;
      robotJoints[i].positionPID.kd =
          gains.positionGains[i].Kd;

      robotJoints[i].velocityPID.kp =
          gains.velocityGains[i].Kp;
      robotJoints[i].velocityPID.ki =
          gains.velocityGains[i].Ki;
      robotJoints[i].velocityPID.kd =
          gains.velocityGains[i].Kd;

      robotJoints[i].currentPID.kp =
          gains.currentGains[i].Kp;
      robotJoints[i].velocityPID.ki =
          gains.velocityGains[i].Ki;
      robotJoints[i].velocityPID.kd =
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
        robotJoints[i].positionPID.kp;
    gainMemory.positionGains[i].Ki =
        robotJoints[i].positionPID.ki;
    gainMemory.positionGains[i].Kd =
        robotJoints[i].positionPID.kd;

    gainMemory.velocityGains[i].Kp =
        robotJoints[i].velocityPID.kp;
    gainMemory.velocityGains[i].Ki =
        robotJoints[i].velocityPID.ki;
    gainMemory.velocityGains[i].Kd =
        robotJoints[i].velocityPID.kd;

    gainMemory.currentGains[i].Kp =
        robotJoints[i].currentPID.kp;
    gainMemory.currentGains[i].Ki =
        robotJoints[i].currentPID.ki;
    gainMemory.currentGains[i].Kd =
        robotJoints[i].currentPID.kd;
  }

  EEPROM.put(controllerGain_address, gainMemory);
}

void Manipulator::generateSingleJointRectanglePosSignal(uint8_t joint_id,float startValue,float endValue, int period){
  int static time = 0;
  int flankTime = 1000*period/2;

  if(millis()-time<flankTime){
    this->robotJoints[joint_id].position_target = startValue;
  }else if(millis()-time>flankTime){
    this->robotJoints[joint_id].position_target = endValue;
  }
  time = millis();
  Serial.println(this->robotJoints[joint_id].position_target);
}

void Manipulator::generateSingleJointSinusPosSignal(uint8_t joint_id,float startValue,float endValue, int period){
  int time = 0;

  time = millis() - time;

  float T = (float) period;
  float x = M_PI*2 * (float) (1/T)*time;

  this->robotJoints[joint_id].position_target = endValue*sin(x) + startValue;

  time = millis();
}

