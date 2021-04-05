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
  Serial.print("Received Actuation command: ");
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
    robot->robotJoints[joint_id].setAngleOffsetRad(angle_offset);
    robot->angle_offsets[joint_id] = angle_offset;
    robot->saveOffsetData();
    break;
  case 't':
    float torqueOffset = command.values[1];
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

void RobotComm::processLimitCommand(robotMsg command)
{
  int joint_id = (int)round(command.values[0]);
  float limit_r = command.values[0];
  float limit_l = command.values[1];
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

void RobotComm::processTargetCommand(robotMsg command)
{
  int joint_id = (int)round(command.values[0]);
  float position_target = command.values[1];
  float velocity_target = command.values[2];
  float acceleration_target = command.values[3];

  robot->robotJoints[joint_id].posiion_target = position_target;
  robot->robotJoints[joint_id].velocity_target = velocity_target;
  robot->robotJoints[joint_id].acceleration_target =
      acceleration_target;
}

void RobotComm::processActivateOutputCommand(robotMsg command)
{
  Serial.println("processing output command");
  output_joint_id = (int)round(command.values[0]);
  mode = (outputMode)round(command.values[1]);

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
        Serial.print("read input: ");
        String msg = Serial.readStringUntil('\n');
        Serial.println(msg);
        if (msg.endsWith(';'))
        {
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