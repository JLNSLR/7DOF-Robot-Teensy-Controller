#ifndef ROBOTBUFFERS_H
#define ROBOTBUFFERS_H

#include <Arduino.h>
#include <CircularBuffer.h>

// --- Raw Input Data Buffers of the Robot Sensors --- //
extern CircularBuffer<int16_t,15> jointPositionInput[];
extern CircularBuffer<int32_t,15> jointTorqueInput[];
extern CircularBuffer<int16_t,15> jointCurrentInput[];

// Test shit around

enum sensorDataType {jointPos, jointTorque, jointCurrent};

#endif// ROBOTBUFFERS_H

