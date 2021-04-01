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

CAN_message_t msg;
int lastCount = 0;

int lastdirection_1 = 0;
int lastdirection_2 = 0;
bool direction = true;

//Manipulator robot;
//RobotComm communication;

RobotJoint joint_test;

void setup()
{

  Serial.begin(115200);
  Serial.setTimeout(1);
  canInit();
  initializeMotorControlPins();
  delay(1000);
  initCurrentSensorADCPins();
  sei();

  sendRGBCommand(1, 128, 0, 0);
  sendRGBCommand(2, 128, 0, 0);
  sendRGBCommand(3, 128, 0, 0);
  sendRGBCommand(4, 128, 0, 0);
  sendRGBCommand(6, 128, 0, 0);

  //robot.initManipulator();
  //communication.robot = &robot;

  delay(1000);

  sendRGBCommand(1, 128, 128, 0);
  sendRGBCommand(2, 128, 128, 0);
  sendRGBCommand(3, 128, 128, 0);
  sendRGBCommand(4, 128, 128, 0);
  sendRGBCommand(6, 128, 128, 0);

  joint_test.joint_id = 1;
  joint_test.motorControllerId = 1;
  joint_test.currentSensorId = 1;

  joint_test.initRobotJoint();
}

void loop()
{
  canBus.events(); //Push received interrupt frames from queue to callback
  //robot.processJointSensors();
  //communication.periodicSerialOutput();
  //communication.readInputCommands();

  joint_test.processPositionInput();

}

