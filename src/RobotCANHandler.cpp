#include <Arduino.h>
#include "FlexCAN_T4.h"
#include "RobotDataStructures.h"
#include <CircularBuffer.h>
#include <RobotBuffers.h>
#include <RobotCANHandler.h>

#define CAN_DEBUG

#define CAN_BAUDRATE 1000000

void processCanMsgs(const CAN_message_t &can_msg);

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canBus;

Circular_Buffer<CAN_message_t*, 1000> canBuffer;

CircularBuffer<int16_t,100> jointPositionInput[7];
CircularBuffer<int32_t,100> jointTorqueInput[7];
CircularBuffer<int16_t,100> jointCurrentInput[7];


void canInit(){
    canBus.begin();
    canBus.setBaudRate(CAN_BAUDRATE);
    canBus.enableFIFO();
    canBus.enableMBInterrupt(FIFO);
    canBus.onReceive(processCanMsgs);
    canBus.setFIFOFilter(REJECT_ALL);
    canBus.setFIFOFilterRange(0,CAN_ID_ENCODERDATA,CAN_ID_STATUS_MSG,STD);
}


void processCanMsgs(const CAN_message_t &can_msg){

  switch(can_msg.id){
      case CAN_ID_ENCODERDATA:
            encoderDataPacket encoderData;
            deSerializeEncoderData(can_msg.buf,&encoderData); 
            jointPositionInput[encoderData.joint_id].push(encoderData.encoderValue);
            #ifdef CAN_DEBUG
            Serial.print("Joint ");
            Serial.print(encoderData.joint_id);
            Serial.print(" encoder Value: ");
            Serial.println(encoderData.encoderValue);
            #endif
        break;
        case CAN_ID_TORQUESENSOR:
            torqueDataPacket torqueData;
            deSerializeTorqueData(can_msg.buf,&torqueData);
            jointTorqueInput[torqueData.joint_id].push(torqueData.torqueValue);
            #ifdef CAN_DEBUG
            Serial.print("Joint ");
            Serial.print(torqueData.joint_id);
            Serial.print(" torque Value: ");
            Serial.println(torqueData.torqueValue);
            #endif
        break;
  }
}

void sendCanMsg(const CAN_message_t &can_msg){
    canBus.write(can_msg);
}

void bytes2CanMsg(const uint8_t* bytes, const int len, CAN_message_t &msg){
    for(int i = 0; i<len; i++){
        msg.buf[i] = bytes[i];
    }
}









