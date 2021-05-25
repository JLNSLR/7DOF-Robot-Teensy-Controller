#include <Arduino.h>
#include "Filters/IIRFilter.h"
#include "Filters/FIRFilter.h"
#include "FlexCAN_T4.h"
#include "RobotDataStructures.h"
#include <CircularBuffer.h>
#include <ControlBoard_PinDefines.h>
#include <RobotCANHandler.h>
#include <MotorDrivers.h>
#include <Manipulator.h>
#include <RobotComm.h>
#include <Math.h>

CAN_message_t msg;
int lastCount = 0;

int lastdirection_1 = 0;
int lastdirection_2 = 0;
int lastCurrent = 0;
bool direction = true;
int n = 0;

Manipulator robot;
RobotComm communication;

RobotJoint joint_test;

void setup()
{

  Serial.begin(115200);
  Serial.setTimeout(1);
  canInit();
  initializeMotorControlPins();
  delay(10);
  initCurrentSensorADCPins();
  sei();

  sendRGBCommand(1, 128, 0, 0);
  sendRGBCommand(2, 128, 0, 0);
  sendRGBCommand(3, 128, 0, 0);
  sendRGBCommand(4, 128, 0, 0);
  sendRGBCommand(6, 128, 0, 0);

  robot.initManipulator();
  communication.robot = &robot;

  delay(1000);

  sendRGBCommand(1, 128, 128, 0);
  sendRGBCommand(2, 128, 128, 0);
  sendRGBCommand(3, 128, 128, 0);
  sendRGBCommand(4, 128, 128, 0);
  sendRGBCommand(6, 128, 128, 0);
  /*
  joint_test.joint_id = 0;
  joint_test.motorControllerId = 0;
  joint_test.currentSensorId = 0;

  joint_test.initRobotJoint();
  */

  robot.robotJoints[6].positionPID.kp = 1000;
  robot.robotJoints[6].positionPID.ki = 0;
  robot.robotJoints[6].positionPID.kd = 0;
}

void loop()
{
  analogWrite(PWM_FAN_1, 3000);
  canBus.events(); //Push received interrupt frames from queue to callback
  robot.processJointSensors();
  communication.periodicSerialOutput();
  communication.readInputCommands();

  delayMicroseconds(100);


  //robot.robotJoints[6].processPositionController();

  robot.generateSingleJointRectanglePosSignal(6,0,60,3000);
}
