#ifndef ROBOTCANHANDLER_H
#define ROBOTCANHANDLER_H

#include <Arduino.h>
#include "FlexCAN_T4.h"
#include "RobotDataStructures.h"
#include <CircularBuffer.h>
#include <RobotBuffers.h>

#define CAN_DEBUG

void processCanMsgs(const CAN_message_t &can_msg);

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canBus;

extern Circular_Buffer<CAN_message_t*, 1000> canBuffer;



void canInit();

void processCanMsgs(const CAN_message_t &can_msg);

void sendCanMsg(const CAN_message_t &can_msg);

void bytes2CanMsg(const uint8_t* bytes, const int len, CAN_message_t &msg);

#endif //ROBOTCANHANDLER_H