#include <RobotComm.h>

RobotComm::RobotComm()
{
}
RobotComm::RobotComm(Manipulator *robot)
{
  this->robot = robot;
}
String *RobotComm::decomposeMsg(String input)
{
  int length = input.length();
  int pos = 0;
  int pos_next = 0;
  int element = 0;
  bool end = false;
#define MAX_ELEMENTS 20
  String output[MAX_ELEMENTS];
  do
  {
    pos_next = input.indexOf(',', pos + 1);
    output[element] = input.substring(pos, pos_next);
    element++;
    pos = pos_next;

    if (pos >= input.length() || pos_next == -1)
    {
      end = true;
    }

  } while (input.charAt(pos + 1) != ';' || element < MAX_ELEMENTS || !end);

  return output;
}

void RobotComm::parseStringCommand(String *msg)
{
  char msg_type = msg[0].charAt(0);

  switch (msg_type)
  {
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
  case 't':
    processTargetCommand(msg);
    break;
  case 'a':
    processActivateOutputCommand(msg);
    break;
  }
}

void RobotComm::processActuationCommand(String command[3])
{
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

void RobotComm::processOffsetCommand(String command[4])
{
  char type = command[1].charAt(0);
  int joint_id = command[2].toInt();
  switch (type)
  {
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

void RobotComm::processLimitCommand(String command[4])
{
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

void RobotComm::processPIDTuneCommand(String command[6])
{
  char type = command[1].charAt(0);
  int joint_id = command[2].toInt();

  float kp = command[3].toFloat();
  float ki = command[4].toFloat();
  float kd = command[5].toFloat();

  switch (type)
  {
  case 'p':
    robot->robotJoints[joint_id].positionController.kp = kp;
    robot->robotJoints[joint_id].positionController.ki = ki;
    robot->robotJoints[joint_id].positionController.kd = kd;
    break;
  case 'v':
    robot->robotJoints[joint_id].velocityController.kp = kp;
    robot->robotJoints[joint_id].velocityController.ki = ki;
    robot->robotJoints[joint_id].velocityController.kd = kd;
    break;
  case 'i':
    robot->robotJoints[joint_id].currentController.kp = kp;
    robot->robotJoints[joint_id].currentController.ki = ki;
    robot->robotJoints[joint_id].currentController.kd = kd;
    break;
  }
}

void RobotComm::processTargetCommand(String command[5])
{
  int joint_id = command[1].toInt();
  float position_target = command[2].toFloat();
  float velocity_target = command[3].toFloat();
  float acceleration_target = command[4].toFloat();

  robot->robotJoints[joint_id].posiion_target = position_target;
  robot->robotJoints[joint_id].velocity_target = velocity_target;
  robot->robotJoints[joint_id].acceleration_target =
      acceleration_target;
}

void RobotComm::processActivateOutputCommand(String command[7])
{
  output_joint_id = command[1].toInt();
  mode = (outputMode)command[2].toInt();

  printLegend();
}

void RobotComm::readInputCommands()
{
  if (micros() - last_input > serialInputPeriod)
  {
    last_input = micros();
    String msg = Serial.readStringUntil(';');
    parseStringCommand(decomposeMsg(msg));
  }
}

void RobotComm::periodicSerialOutput()
{
  if (micros() - last_output > serialOutputPeriod)
    last_output = micros();
  {

    switch (mode)
    {
    case position:
      printSerialPosition(output_joint_id);
      break;
    case traj:
      printSerialTrajectory(output_joint_id);
      break;
    case torque:
      printSerialTorque(output_joint_id);
      break;
    case curr:
      printSerialCurrent(output_joint_id);
      break;
    case joint:
      printSerialJointData(output_joint_id);
      break;
    case allPosition:
      printSerialAllPositionData();
      break;
    case allTraj:
      printSerialAllTrajectories();
      break;
    case allData:
      printSerialAllData();
      break;
    default:
      break;
    }
  }
}

void RobotComm::printSerialPosition(int joint_id)
{
  Serial.println(robot->robotJoints[output_joint_id].getAngleRad());
}

void RobotComm::printSerialTrajectory(int joint_id)
{
  Serial.print(robot->robotJoints[output_joint_id].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[output_joint_id].getVelocityRad());
  Serial.print(',');
  Serial.println(robot->robotJoints[output_joint_id].getAccelerationRad());
}

void RobotComm::printSerialTorque(int joint_id)
{
  Serial.println(robot->robotJoints[output_joint_id].getTorque());
}

void RobotComm::printSerialCurrent(int joint_id)
{
  Serial.println(robot->robotJoints[output_joint_id].getCurrent());
}

void RobotComm::printSerialJointData(int joint_id)
{
  Serial.print(robot->robotJoints[output_joint_id].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[output_joint_id].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[output_joint_id].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[output_joint_id].getTorque());
  Serial.print(',');
  Serial.println(robot->robotJoints[output_joint_id].getCurrent());
}

void RobotComm::printSerialAllPositionData()
{
  Serial.print(robot->robotJoints[0].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getAngleRad());
  Serial.print(',');
  Serial.println(robot->robotJoints[6].getAngleRad());
}

void RobotComm::printSerialAllTrajectories()
{
  //Position Data
  Serial.print(robot->robotJoints[0].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[6].getAngleRad());

  Serial.print(',');
  //Velocity Data
  Serial.print(robot->robotJoints[0].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[6].getVelocityRad());

  Serial.print(',');
  //Acceleration Data
  Serial.print(robot->robotJoints[0].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getAccelerationRad());
  Serial.print(',');
  Serial.println(robot->robotJoints[6].getAccelerationRad());
}

void RobotComm::printSerialAllData()
{
  Serial.print(robot->robotJoints[0].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getAngleRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[6].getAngleRad());

  Serial.print(',');
  //Velocity Data
  Serial.print(robot->robotJoints[0].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getVelocityRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[6].getVelocityRad());

  Serial.print(',');
  //Acceleration Data
  Serial.print(robot->robotJoints[0].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getAccelerationRad());
  Serial.print(',');
  Serial.print(robot->robotJoints[6].getAccelerationRad());

  Serial.print(',');
  //Torque Data
  Serial.print(robot->robotJoints[0].getTorque());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getTorque());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getTorque());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getTorque());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getTorque());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getTorque());
  Serial.print(',');
  Serial.print(robot->robotJoints[6].getTorque());

  Serial.print(',');
  //Curren Data
  Serial.print(robot->robotJoints[0].getCurrent());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getCurrent());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getCurrent());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getCurrent());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getCurrent());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getCurrent());
  Serial.print(',');
  Serial.println(robot->robotJoints[6].getCurrent());
}

void RobotComm::printLegend()
{
  String legend = "";
  switch (mode)
  {
  case position:
    legend = "Joint ";
    legend += output_joint_id + " Angle [rad]:";
    Serial.println(legend);
    break;
  case traj:
    legend = "Joint ";
    legend += output_joint_id + " Angle [rad]:";
    legend += ",Velocity [rad/s]:,Acceleration [rad/s^2]:";
    Serial.println(legend);
    break;
  case torque:
    legend = "Joint ";
    legend += output_joint_id + " Torque [Nm]:";
    Serial.println(legend);
    break;
  case curr:
    legend = "Joint ";
    legend += output_joint_id + " Current [mA]:";
    Serial.println(legend);
    break;
  case joint:
    legend = "Joint ";
    legend += output_joint_id + " Angle [rad]:";
    legend += ",Velocity [rad/s]:,Acceleration [rad/s^2]:,Torque [Nm]:,Current [mA]:";
    Serial.println(legend);
    break;
  case allPosition:
    legend = "J0:,J1:,J2:,J3:,J4:,J5:,J6:";
    Serial.println(legend);
    break;
  case allTraj:
    break;
  case allData:
    break;
  default:
    break;
  }
}