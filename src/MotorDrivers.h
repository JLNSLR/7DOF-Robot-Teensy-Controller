#ifndef MOTORDRIVERS_H
#define MOTORDRIVERS_H
#include <Arduino.h>
#include <ControlBoard_PinDefines.h>

const int pwm_frequency = 20000;
const int pwm_resolution = 12;

const int pwm_pins[23] = {PWM_FAN_1, PWM_FAN_2,PWM_FAN_3,PWM_FAN_4,PWM_FAN_5,PWM_FAN_6,
LPWM_1, LPWM_2, LPWM_3, LPWM_4, RPWM_1, RPWM_2, RPWM_3, RPWM_4, EN_DUAL_A, EN_DUAL_B, 
IN1_DUAL_A, IN1_DUAL_B, IN2_DUAL_A, IN2_DUAL_B, IN1, IN2, STATELED}; 

const uint8_t currentSensorPins[7] = {CURRENT_IN_1, CURRENT_IN_2, CURRENT_IN_3, CURRENT_IN_4, CURRENT_IN_5, CURRENT_IN_6, CURRENT_IN_7};

void initializeMotorControlPins();

void controlMotorDriver(int driver_id, int16_t pwmValue);

void driveHBridge_0(int16_t pwmValue);
void driveHBridge_1(int16_t pwmValue);
void driveHBridge_2(int16_t pwmValue);
void driveHBridge_3(int16_t pwmValue);
void driveHBridge_4(int16_t pwmValue);
void driveHBridge_5(int16_t pwmValue);
void driveHBridge_6(int16_t pwmValue);

enum currentSensorSpec { I5A , I30A };

void initCurrentSensorADCPins();

int16_t readCurrentSensor(int sensorId);

int16_t processCurrentSensor(currentSensorSpec spec, uint8_t pin);


#endif //MOTORDRIVERS_H