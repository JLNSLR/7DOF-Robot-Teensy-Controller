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

void setup() {

canInit();
initializeMotorControlPins();
initCurrentSensorADCPins();

}

void loop(){

analogWrite(PWM_FAN_1,4096);
analogWrite(PWM_FAN_2,2048);
analogWrite(PWM_FAN_3,2048);
analogWrite(PWM_FAN_4,500);
analogWrite(PWM_FAN_5,2048);
analogWrite(PWM_FAN_6,2048);



controlMotorDriver(0,2048);
controlMotorDriver(1,2048);
controlMotorDriver(2,2048);
controlMotorDriver(3,2048);
controlMotorDriver(4,2048);
controlMotorDriver(5,2048);
controlMotorDriver(6,2048);
delay(500);
controlMotorDriver(0,-2048);
controlMotorDriver(1,-2048);
controlMotorDriver(2,-2048);
controlMotorDriver(3,-2048);
controlMotorDriver(4,-2048);
controlMotorDriver(5,-2048);
controlMotorDriver(6,-2048);
delay(500);

Serial.println("Read current sensors: ");
Serial.println(readCurrentSensor(0));






  if(canBus.read(msg)){
    /*
    Serial.print("CAN BUS receiving: ");
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
    Serial.print("  EXT: "); Serial.print(msg.flags.extended );
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    digitalWrite(13,HIGH);
    */
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.print(msg.buf[i],HEX); Serial.print(" ");
    }


    Serial.print("  TS: "); Serial.println(msg.timestamp);
       if(msg.id == 0x00){
         
        Serial.print("Encoder_msg:");
        encoderDataPacket paket;
      

        deSerializeEncoderData(msg.buf,&paket);
        Serial.print(paket.joint_id);
        Serial.print(" value: ");
        Serial.println(paket.encoderValue);

      }
      else if(msg.id == 0x01){
        torqueDataPacket torqueData;

        uint8_t bytes[4];
        bytes[0] = msg.buf[0];
        bytes[1] = msg.buf[1];
        bytes[2] = msg.buf[2];
        bytes[3] = msg.buf[3];

        deSerializeTorqueData(msg.buf,&torqueData);
        Serial.print("Torque: ");
        Serial.print(" value: ");
        Serial.print(torqueData.torqueValue);
        Serial.print("   ");

        Serial.print(torqueData.joint_id);
        Serial.println("");


      }


  }

}




  

