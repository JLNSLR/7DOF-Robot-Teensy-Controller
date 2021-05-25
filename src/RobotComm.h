#ifndef ROBOTCOMM_H
#define ROBOTCOMM_H
#include <RobotJoint.h>
#include <Manipulator.h>

#define POS_CONTROLLER 0
#define VEL_CONTROLLER 1
#define CURRENT_CONTROLLER 2

#define COMMUNICATION_DEBUG

struct actuationCommand
{
  uint8_t joint_id;
  int16_t motorCommand;
};

struct positionOffsetCommand
{
  uint8_t joint_id;
  float offset;
};
struct torqueOffsetCommand
{
  uint8_t joint_id;
  float offset;
};
struct jointLimitCommand
{
  uint8_t joint_id;
  float limit_r;
  float limit_l;
};

struct jointPIDGainsCommand
{
  uint8_t joint_id;
  uint8_t controllerType;
  float Kp;
  float Ki;
  float Kd;
};

struct robotStateVector
{
  char identifier = 's';
  float joint_angles[7];
  float joint_velocities[7];
  float joint_accelerations[7];
  float jointTorques[7];
  float jointCurrents[7];
};

struct robotMsg
{
  char top_type;
  char medium_type;
  char low_type;
  float values[35];
};

class RobotComm
{

public:
  RobotComm();
  RobotComm(Manipulator *robot);

  Manipulator *robot;

  int serialOutputPeriod = 3333;
  int serialInputPeriod = 3333;

  void periodicSerialOutput();

  void readInputCommands();

  void writeStatusMsg(String msg);

private:
  robotMsg decomposeMsg(String msg);
  void parseRobotCommand(robotMsg msg);

  void processActuationCommand(robotMsg commands);

  void processOffsetCommand(robotMsg commands);

  void processLimitCommand(robotMsg commands);

  void processPIDTuneCommand(robotMsg commands);

  void processTargetCommand(robotMsg commands);

  void processActivateOutputCommand(robotMsg commands);

  void printSerialPosition(int joint_id);

  void printSerialTrajectory(int joint_id);

  void printSerialTorque(int joint_id);

  void printSerialCurrent(int joint_id);

  void printSerialJointData(int joint_id);

  void printSerialAllPositionData();

  void printSerialAllTrajectories();

  void printSerialAllData();

  void printLegend();

  int output_joint_id = 0;
  enum outputMode
  {
    position,
    traj,
    torque,
    curr,
    joint,
    allPosition,
    allTraj,
    allData
  };

  outputMode mode = allPosition;

  int last_output = 0;
  int last_input = 0;
};

#endif // ROBOTCOMM_H