#ifndef FIXPOINTMATH_H
#define FIXPOINTMATH_H
#include <Arduino.h>


/*
To accelerate control and filter algorithms for the 7-Axis-Robot most of the calculations are done
width a fixed point arithmetic. To unify the data format for the control and filter algorithms a 32 bitsigned
Integer is used. Addition and subtraction of the fixed point format can rely on the standard integer addition and subtraction functions.
For multiplication and division additional steps are required these functions are defined here. Since these functions
are used everywhere in the project they will be used as global functions width inline property to avoid the overhead of a class.
*/
#define Q 14 //18 integer bits and 14 fractional bits Q18.14 system
#define K   (1 << (Q - 1))
/*
const int32_t toQFactor = 1 << Q;
const float toFloatFactor = (float) 1/toQFactor;
const double toDoubleFactor = (double) 1/toQFactor;
const int32_t posLimit = 2147483647;
const int32_t negLimit = -2147483648;
*/

int32_t sat32(int64_t value);

float  fix2Float(int32_t fix_value);

double  fix2Double(int32_t fix_value);

double  double2Fix(double d_val);

int32_t  float2Fix(float fl_val);

int32_t  multiply(int32_t a, int32_t b);

int32_t  divide(int32_t a, int32_t b);

#endif //FIXPOINTMATH_H
