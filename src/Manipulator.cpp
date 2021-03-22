#include <Manipulator.h>

Manipulator::Manipulator() {

  // Init Joints with standard values
  for (int i = 0; i < 7; i++) {
    robotJoints[i].setAngleOffsetRad(angle_offsets[i]);
    robotJoints[i].setTorqueOffsetNm(torqueOffsets[i]);
  }
};

void Manipulator::initManipulator() {

  // --- Read Initializaton Data from Memory
  readOffsetDataFromMemory();
  readGainDataFromMemory();
  readLimitDataFromMemory();
}

void Manipulator::processJointSensors() {
  for (int i = 0; i < numberOfJoints; i++) {
    robotJoints[i].processCurrentInput();
    robotJoints[i].processPositionInput();
    robotJoints[i].processTorqueInput();
  }
}

void Manipulator::readOffsetDataFromMemory() {
  struct RobotOffsets offsets;

  EEPROM.get(offset_addresses, offsets);

  if (offsets.validity == validity_key) {
    for (int i = 0; i < 7; i++) {
      robotJoints[i].setAngleOffsetRad(offsets.angleOffsets[i]);
      robotJoints[i].setTorqueOffsetNm(offsets.torqueOffsets[i]);
    }
  }
};

void Manipulator::readGainDataFromMemory() {
  struct PositionControllerGains gains;

  EEPROM.get(controllerGain_address, gains);

  if (gains.validity == validity_key) {
    for (int i = 0; i < 7; i++) {
      robotJoints[i].positionController.kp =
          float2Fix(gains.positionGains[i].Kp);
      robotJoints[i].positionController.ki =
          float2Fix(gains.positionGains[i].Ki);
      robotJoints[i].positionController.kd =
          float2Fix(gains.positionGains[i].Kd);

      robotJoints[i].velocityController.kp =
          float2Fix(gains.velocityGains[i].Kp);
      robotJoints[i].velocityController.ki =
          float2Fix(gains.velocityGains[i].Ki);
      robotJoints[i].velocityController.kd =
          float2Fix(gains.velocityGains[i].Kd);

      robotJoints[i].currentController.kp =
          float2Fix((gains.currentGains[i].Kp));
      robotJoints[i].velocityController.ki =
          float2Fix(gains.velocityGains[i].Ki);
      robotJoints[i].velocityController.kd =
          float2Fix(gains.velocityGains[i].Kd);
    }
  }
}

void Manipulator::readLimitDataFromMemory() {
  struct RobotLimits limits;

  EEPROM.get(limitAdresses, limits);

  if (limits.validity == validity_key) {
    for (int i = 0; i < 7; i++) {
      robotJoints[i].setPosLimits(limits.limits_r[i], limits.limits_l[i]);
    }
  }
}

void Manipulator::saveOffsetData() {
  struct RobotOffsets offsetData;
  offsetData.validity = validity_key;
  for (int i = 0; i < 7; i++) {
    offsetData.angleOffsets[i] = angle_offsets[i];
  }
  EEPROM.put(offset_addresses, offsetData);
};

void Manipulator::saveLimitData() {
  struct RobotLimits limits;
  limits.validity = validity_key;
  for (int i = 0; i < 7; i++) {
    limits.limits_r[i] = robotJoints[i].getLimitR();
    limits.limits_l[i] = robotJoints[i].getLimitL();
  }
  EEPROM.put(limitAdresses, limits);
};

void Manipulator::saveGainData() {
  PositionControllerGains gainMemory;
  gainMemory.validity = validity_key;

  for (int i = 0; i < 7; i++) {
    gainMemory.positionGains[i].Kp =
        fix2Float(robotJoints[i].positionController.kp);
    gainMemory.positionGains[i].Ki =
        fix2Float(robotJoints[i].positionController.ki);
    gainMemory.positionGains[i].Kd =
        fix2Float(robotJoints[i].positionController.kd);

    gainMemory.velocityGains[i].Kp =
        fix2Float(robotJoints[i].velocityController.kp);
    gainMemory.velocityGains[i].Ki =
        fix2Float(robotJoints[i].velocityController.ki);
    gainMemory.velocityGains[i].Kd =
        fix2Float(robotJoints[i].velocityController.kd);

    gainMemory.currentGains[i].Kp =
        fix2Float(robotJoints[i].currentController.kp);
    gainMemory.currentGains[i].Ki =
        fix2Float(robotJoints[i].currentController.ki);
    gainMemory.currentGains[i].Kd =
        fix2Float(robotJoints[i].currentController.kd);
  }

  EEPROM.put(controllerGain_address, gainMemory);
}