#include <RobotComm.h>

RobotComm::RobotComm()
{
}
RobotComm::RobotComm(Manipulator *robot)
{
  this->robot = robot;
}
robotMsg RobotComm::decomposeMsg(String input)
{
  int pos = 0;
  int pos_next = 0;
  int element = 0;
  bool end = false;

  robotMsg r_msg;

  int posValue = input.indexOf(':');
  String types = input.substring(0, posValue);
  String values = input.substring(posValue + 1);

  r_msg.top_type = types.charAt(0);
  if (types.length() > 2)
  {
    r_msg.medium_type = types.charAt(2);

    if (types.length() > 3)
    {
      r_msg.low_type = types.charAt(4);
    }
  }

  do
  {
    pos_next = values.indexOf(',', pos + 1);
    if (pos_next < 0)
    {
      r_msg.values[element] = values.substring(pos).toFloat();
    }
    else
    {
      r_msg.values[element] = values.substring(pos, pos_next).toFloat();
    }
    element++;
    pos = pos_next + 1;

    if (pos >= values.length() || pos_next == -1)
    {
      end = true;
    }

  } while (!end);

  return r_msg;
}

void RobotComm::parseRobotCommand(robotMsg msg)
{
  char msg_type = msg.top_type;

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

void RobotComm::processActuationCommand(robotMsg command)
{
  int joint_id = (int)round(command.values[0]);
  int16_t motorCommand = (int16_t)round(command.values[1]);
  robot->robotJoints[joint_id].drive(motorCommand);

#ifdef COMMUNICATION_DEBUG
  Serial.print("#Received Actuation command: ");
  Serial.print(joint_id);
  Serial.print(",");
  Serial.println(motorCommand);
#endif
}

void RobotComm::processOffsetCommand(robotMsg command)
{
  char type = command.medium_type;
  int joint_id = (int)round(command.values[0]);
  switch (type)
  {
  case 'p':
    float angle_offset = command.values[1];
    float previousAngleOffset = robot->robotJoints[joint_id].getAngleOffset();
    robot->robotJoints[joint_id]
        .setAngleOffsetDeg(angle_offset + previousAngleOffset);
    robot->angle_offsets[joint_id] = angle_offset;
    robot->saveOffsetData();
    break;
  case 't':
    float torqueOffset = command.values[1];
    float previousTorqueOffset = robot->robotJoints[joint_id].getTorqueOffset();
    robot->robotJoints[joint_id].setTorqueOffsetNm(torqueOffset + previousTorqueOffset);
    robot->torqueOffsets[joint_id] = torqueOffset;
    robot->saveOffsetData();
    break;
  }

#ifdef COMMUNICATION_DEBUG
  Serial.print("#Received Offset command: ");
  Serial.print(joint_id);
  Serial.print(",");
  Serial.println(type);
#endif
}

void RobotComm::processLimitCommand(robotMsg command)
{
  int joint_id = (int)round(command.values[0]);
  float limit_r = command.values[0];
  float limit_l = command.values[1];
  robot->robotJoints[joint_id].setPosLimits(limit_r, limit_l);
  robot->saveLimitData();
#ifdef COMMUNICATION_DEBUG
  Serial.print("#Received Limit command: ");
  Serial.print(joint_id);
  Serial.print(",");
  Serial.print(limit_r);
  Serial.print(",");
  Serial.println(limit_l);
#endif
}

void RobotComm::processPIDTuneCommand(robotMsg command)
{
  char type = command.medium_type;
  int joint_id = (int)round(command.values[0]);

  float kp = command.values[1];
  float ki = command.values[2];
  float kd = command.values[3];

  switch (type)
  {
  case 'p':
    robot->robotJoints[joint_id].positionPID.kp = kp;
    robot->robotJoints[joint_id].positionPID.ki = ki;
    robot->robotJoints[joint_id].positionPID.kd = kd;
    break;
  case 'v':
    robot->robotJoints[joint_id].velocityPID.kp = kp;
    robot->robotJoints[joint_id].velocityPID.ki = ki;
    robot->robotJoints[joint_id].velocityPID.kd = kd;
    break;
  case 'i':
    robot->robotJoints[joint_id].currentPID.kp = kp;
    robot->robotJoints[joint_id].currentPID.ki = ki;
    robot->robotJoints[joint_id].currentPID.kd = kd;
    break;
  }
}

void RobotComm::processTargetCommand(robotMsg command)
{
  int joint_id = (int)round(command.values[0]);
  float position_target = command.values[1];
  float velocity_target = command.values[2];
  float acceleration_target = command.values[3];

  robot->robotJoints[joint_id].position_target = position_target;
  robot->robotJoints[joint_id].velocityPID.setpoint = velocity_target;
  robot->robotJoints[joint_id].acceleration_target =
      acceleration_target;
}

void RobotComm::processActivateOutputCommand(robotMsg command)
{
  Serial.println("#processing output command");
  output_joint_id = (int)round(command.values[0]);
  mode = outputMode((int)round(command.values[1]));

  Serial.print("#Outputmode: ");
  Serial.println(mode);

  printLegend();
}

void RobotComm::readInputCommands()
{
  if (micros() - last_input > serialInputPeriod)
  {
    last_input = micros();
    {
      if (Serial.available())
      {
        String msg = Serial.readStringUntil('\n');
        if (msg.endsWith(';'))
        {
          writeStatusMsg(msg);
          parseRobotCommand(decomposeMsg(msg));
        }
      }
    }
  }
}

void RobotComm::periodicSerialOutput()
{
  if (micros() - last_output > serialOutputPeriod)
    last_output = micros();
  {
    Serial.print("d:");
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
  Serial.println(robot->robotJoints[output_joint_id].getAngleDeg());
}

void RobotComm::printSerialTrajectory(int joint_id)
{
  Serial.print(robot->robotJoints[output_joint_id].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[output_joint_id].getVelocityDeg());
  Serial.print(',');
  Serial.println(robot->robotJoints[output_joint_id].getAccelerationDeg());
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

  Serial.print(robot->robotJoints[joint_id].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[joint_id].getVelocityDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[joint_id].getAccelerationDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[joint_id].getTorque());
  Serial.print(',');
  Serial.println(robot->robotJoints[joint_id].getCurrent());
  Serial.print(',');
  Serial.println(robot->robotJoints[joint_id].motorCommand);
}

void RobotComm::printSerialAllPositionData()
{
  Serial.print(robot->robotJoints[0].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getAngleDeg());
  Serial.print(',');
  Serial.println(robot->robotJoints[6].getAngleDeg());
}

void RobotComm::printSerialAllTrajectories()
{
  //Position Data
  Serial.print(robot->robotJoints[0].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getAngleDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[6].getAngleDeg());

  Serial.print(',');
  //Velocity Data
  Serial.print(robot->robotJoints[0].getVelocityDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getVelocityDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getVelocityDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getVelocityDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getVelocityDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getVelocityDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[6].getVelocityDeg());

  Serial.print(',');
  //Acceleration Data
  Serial.print(robot->robotJoints[0].getAccelerationDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[1].getAccelerationDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[2].getAccelerationDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[3].getAccelerationDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[4].getAccelerationDeg());
  Serial.print(',');
  Serial.print(robot->robotJoints[5].getAccelerationDeg());
  Serial.print(',');
  Serial.println(robot->robotJoints[6].getAccelerationDeg());
}

void RobotComm::printSerialAllData()
{
  //Angle Data
  for (int i = 0; i < robot->numberOfJoints; i++)
  {
    Serial.print(robot->robotJoints[i].getAngleDeg());
    Serial.print(',');
  }
  //Velocity Data
  for (int i = 0; i < robot->numberOfJoints; i++)
  {
    Serial.print(robot->robotJoints[i].getVelocityDeg());
    Serial.print(',');
  }
  //Acceleration Data
  for (int i = 0; i < robot->numberOfJoints; i++)
  {
    Serial.print(robot->robotJoints[i].getAccelerationDeg());
    Serial.print(',');
  }

  //Torque Data
  for (int i = 0; i < robot->numberOfJoints; i++)
  {
    Serial.print(robot->robotJoints[i].getTorque());
    Serial.print(',');
  }

  //Current Data
  for (int i = 0; i < robot->numberOfJoints; i++)
  {
    Serial.print(robot->robotJoints[i].getCurrent());
    Serial.print(',');
  }
  for (int i = 0; i < robot->numberOfJoints - 1; i++)
  {
    Serial.print(robot->robotJoints[i].motorCommand);
    Serial.print(',');
  }
  Serial.println(robot->robotJoints[6].motorCommand);
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

void RobotComm::writeStatusMsg(String msg)
{
  String message = "#";
  Serial.println(message + msg);
}