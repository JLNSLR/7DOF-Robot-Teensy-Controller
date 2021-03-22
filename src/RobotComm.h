#ifndef ROBOTCOMM_H
#define ROBOTCOMM_H
#include <RobotJoint.h>

#define POS_CONTROLLER 0
#define VEL_CONTROLLER 1
#define CURRENT_CONTROLLER 2

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
}

struct jointPIDGainsCommand
{
    uint8_t joint_id;
    uint8_t controllerType;
    float Kp;
    float Ki;
    float Kd;

}

struct robotStateVector
{
    char identifier = 's';
    float joint_angles[7];
    float joint_velocities[7];
    float joint_accelerations[7];
    float jointTorques[7];
    float jointCurrents[7];
}

class RobotComm
{
    RobotComm();

    Serial.write();

    String *decomposeMsg(String msg);
};

#endif //ROBOTCOMM_H