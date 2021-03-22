#ifndef ROBOTCOMM_H
#define ROBOTCOMM_H
#include <RobotJoint.h>
#include <Manipulator.h>

#define POS_CONTROLLER 0
#define VEL_CONTROLLER 1
#define CURRENT_CONTROLLER 2

#define COMMUNICATION_DEBUG

struct actuationCommand {
  uint8_t joint_id;
  int16_t motorCommand;
};

struct positionOffsetCommand {
  uint8_t joint_id;
  float offset;
};
struct torqueOffsetCommand {
  uint8_t joint_id;
  float offset;
};
struct jointLimitCommand {
  uint8_t joint_id;
  float limit_r;
  float limit_l;
};

struct jointPIDGainsCommand {
  uint8_t joint_id;
  uint8_t controllerType;
  float Kp;
  float Ki;
  float Kd;
};

struct robotStateVector {
  char identifier = 's';
  float joint_angles[7];
  float joint_velocities[7];
  float joint_accelerations[7];
  float jointTorques[7];
  float jointCurrents[7];
};

class RobotComm {

public:
  RobotComm();
  RobotComm(Manipulator *robot);

  Manipulator *robot;

  int serialOutputPeriod = 3333;

  String *decomposeMsg(String msg);
  void parseStringCommand(String *msg);

  void processActuationCommand(String commands[3]);

  void processOffsetCommand(String commands[4]);

  void processLimitCommand(String commands[4]);

  void processPIDTuneCommand(String commands[5]);

  void processTargetCommand(String commands[5]);

  void activateSerialSensorOutput(int joint_id, bool position, bool velocity,
                                  bool acceleration, bool torque, bool current);
  void periodicSerialOutput();

private:
  int output_joint_id;
  bool position_output;
  bool velocity_output;
  bool acceleration_output;
  bool torque_output;
  bool current;
};

#endif // ROBOTCOMM_H