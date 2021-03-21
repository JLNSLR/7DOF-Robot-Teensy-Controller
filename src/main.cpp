#include <Arduino.h>
#include "FixPointMath.h"
#include "IIRFilter.h"
#include "FIRFilter.h"
#include "FlexCAN_T4.h"
#include "RobotDataStructures.h"
#include <CircularBuffer.h>
#include <ControlBoard_PinDefines.h>
#include <RobotCANHandler.h>
#include <MotorDrivers.h>

CAN_message_t msg;
int lastCount = 0;

int lastdirection_1 = 0;
int lastdirection_2 = 0;
bool direction = true;

void setup()
{

  Serial.begin(115200);
  canInit();
  initializeMotorControlPins();

  delay(1000);
  initCurrentSensorADCPins();
  sei();
}

void loop()
{

  canBus.events(); //Push received interrupt frames from queue to callback

  if (millis() - lastCount > 1)
  {
    /*
    Serial.print("encoder count: ");
    Serial.println(encoderCount);
    Serial.print("torque count: ");
    Serial.println(torqueCount);
    encoderCount = 0;
    torqueCount = 0;
    lastCount = millis();

    sendRGBCommand(3, 0, 0, 255);
    */
   // Serial.println("Read current sensors: ");
    Serial.println(readCurrentSensor(0));
    lastCount = millis();
  }

  if (millis() - lastdirection_1 > 500)
  {
    if (direction)
    {
      direction = false;
    }
    else
    {
      direction = true;
    }
    lastdirection_1 = millis();
  }

  if (direction)
  {
    controlMotorDriver(0, -850);
  }
  else
  {
    controlMotorDriver(0, 850);
  }

}
