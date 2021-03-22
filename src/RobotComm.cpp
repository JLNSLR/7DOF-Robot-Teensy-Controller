#include <RobotComm.h>

String *RobotComm::decomposeMsg(String input) {
  int length = input.length();
  int pos = 0;
  int pos_next = 0;
  int element = 0;
  bool end = false;
#define MAX_ELEMENTS 10
  String output[MAX_ELEMENTS];
  do {
    pos_next = input.indexOf(',', pos + 1);
    output[element] = input.substring(pos, pos_next);
    element++;
    pos = pos_next;

  } while (input.charAt(pos + 1) != ';' || element > MAX_ELEMENTS);

  return output;
}

void RobotComm::parseStringCommand(String *msg) {
  char msg_type = msg[0].charAt(0);

  switch (msg_type) {
  case 'u':
    processActuationCommand(msg);
    break;
  case 'o':
    processOffsetCommand(msg);
    break;
  case 'l':
    processLimitCommand(msg);
    break;
  case 'c':
    processPIDTuneCommand(msg);
    break;
  case 'p':
    processTargetCommand(msg);
    break;
  }
}

void RobotComm::processActuationCommand(String command[3]) {
  int joint_id = command[1].toInt();
  int16_t motorCommand = command[2].toInt();
  robot->robotJoints[joint_id].drive(motorCommand);

#ifdef COMMUNICATION_DEBUG
  Serial.print("Received Actuation command: ");
  Serial.print(joint_id);
  Serial.print(",");
  Serial.println(motorCommand);
#endif
}

void RobotComm::processOffsetCommand(String command[4]) {
  char type = command[1].charAt(0);
  int joint_id = command[2].toInt();
  switch (type) {
  case 'p':
    float angle_offset = command[3].toFloat();
    robot->robotJoints[joint_id].setAngleOffsetRad(angle_offset);
    robot->angle_offsets[joint_id] = angle_offset;
    robot->saveOffsetData();
    break;
  case 't':
    float torqueOffset = command[3].toFloat();
    robot->robotJoints[joint_id].setTorqueOffsetNm(torqueOffset);
    robot->torqueOffsets[joint_id] = torqueOffset;
    robot->saveOffsetData();
    break;
  }

#ifdef COMMUNICATION_DEBUG
  Serial.print("Received Offset command: ");
  Serial.print(joint_id);
  Serial.print(",");
  Serial.println(type);
#endif
}

void RobotComm::processLimitCommand(String command[4]) {
  int joint_id = command[1].toInt();
  float limit_r = command[2].toFloat();
  float limit_l = command[3].toFloat();
  robot->robotJoints[joint_id].setPosLimits(limit_r, limit_l);
  robot->saveLimitData();
#ifdef COMMUNICATION_DEBUG
  Serial.print("Received Limit command: ");
  Serial.print(joint_id);
  Serial.print(",");
  Serial.print(limit_r);
  Serial.print(",");
  Serial.println(limit_l);
#endif
}

void RobotComm::processPIDTuneCommand(String command[6]) {
  char type = command[1].charAt(0);
  int joint_id = command[2].toInt();

  float kp = command[3].toFloat();
  float ki = command[4].toFloat();
  float kd = command[5].toFloat();

  switch (type) {
  case 'p':
    robot->robotJoints[joint_id].positionController.kp = float2Fix(kp);
    robot->robotJoints[joint_id].positionController.ki = float2Fix(ki);
    robot->robotJoints[joint_id].positionController.kd = float2Fix(kd);
    break;
  case 'v':
    robot->robotJoints[joint_id].velocityController.kp = float2Fix(kp);
    robot->robotJoints[joint_id].velocityController.ki = float2Fix(ki);
    robot->robotJoints[joint_id].velocityController.kd = float2Fix(kd);
    break;
  case 'i':
    robot->robotJoints[joint_id].currentController.kp = float2Fix(kp);
    robot->robotJoints[joint_id].currentController.ki = float2Fix(ki);
    robot->robotJoints[joint_id].currentController.kd = float2Fix(kd);
    break;
  }
}

void RobotComm::processTargetCommand(String command[5]) {
  int joint_id = command[1].toInt();
  float position_target = command[2].toFloat();
  float velocity_target = command[3].toFloat();
  float acceleration_target = command[4].toFloat();

  robot->robotJoints[joint_id].posiion_target = float2Fix(position_target);
  robot->robotJoints[joint_id].velocity_target = float2Fix(velocity_target);
  robot->robotJoints[joint_id].acceleration_target =
      float2Fix(acceleration_target);
}

void RobotComm::activateSerialSensorOutput(int joint_id, bool position,
                                           bool velocity, bool acceleration,
                                           bool torque, bool current) {}