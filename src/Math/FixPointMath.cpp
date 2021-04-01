#include "FixPointMath.h"


/*
To accelerate control and filter algorithms for the 7-Axis-Robot most of the calculations are done
with a fixed point arithmetic. To unify the data format for the control and filter algorithms a 32 bitsigned
Integer is used. Addition and subtraction of the fixed point format can rely on the standard integer addition and subtraction functions.
For multiplication and division additional steps are required these functions are defined here. Since these functions
are used everywhere in the project they will be used as global functions with inline property to avoid the overhead of a class.
*/

const int32_t toQFactor = 1 << Q;
const float toFloatFactor = (float) 1/toQFactor;
const double toDoubleFactor = (double) 1/toQFactor;
const int32_t posLimit = 2147483647;
const int32_t negLimit = -2147483648;

int32_t sat32(int64_t value){
    if(value>posLimit) return posLimit;
    else if(value < negLimit) return negLimit;
    else return (int32_t) value;
}

float  fix2Float(int32_t fix_value){
    return fix_value*toFloatFactor;
}

double  fix2Double(int32_t fix_value){
    return fix_value*toDoubleFactor;
}

double  double2Fix(double d_val){
    return d_val*toDoubleFactor;
}

int32_t  float2Fix(float fl_val){
    return fl_val*toQFactor;
}

int32_t  multiply(int32_t a, int32_t b){
    int64_t intermediate =  (int64_t) a * (int64_t) b;
    //Rounding up
    intermediate += K;
    return sat32( intermediate >> Q);
}

int32_t  divide(int32_t a, int32_t b){
    int64_t intermediate = (int64_t) a << Q;

    if((intermediate >=0 && b>= 0) || (intermediate <0 && b<0)){
            intermediate+= b/2;
    }
        else{
            intermediate-= b/2;
        }
    return intermediate/b;;
}

