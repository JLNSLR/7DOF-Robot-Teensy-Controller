#ifndef CONTROLBOARD_PINDEFINES_H
#define CONTROLBOARD_PINDEFINES_H

#include <Arduino.h>

/*--- Motion Control Board Teensy Pin Definitions --- */

//Current Sensor Inputs
#define CURRENT_IN_1 14
#define CURRENT_IN_2 15
#define CURRENT_IN_3 16
#define CURRENT_IN_4 17
#define CURRENT_IN_5 20
#define CURRENT_IN_6 40
#define CURRENT_IN_7 41
//PWM-Pins to control cooling Fans
#define PWM_FAN_1 10
#define PWM_FAN_2 11
#define PWM_FAN_3 12
#define PWM_FAN_4 33
#define PWM_FAN_5 19
#define PWM_FAN_6 18

// H-Bridge-Pin Defintions

#define LPWM_1 0
#define RPWM_1 1

#define LPWM_2 2
#define RPWM_2 3

#define LPWM_3 4
#define RPWM_3 5

#define LPWM_4 6
#define RPWM_4 7

#define EN_DUAL_B 8
#define EN_DUAL_A 9
#define IN1_DUAL_B 28
#define IN2_DUAL_B 29
#define IN1_DUAL_A 34
#define IN2_DUAL_A 35

#define IN1 36
#define IN2 37

#define STATELED 13



#endif //CONTROLBOARD_PINDEFINES


