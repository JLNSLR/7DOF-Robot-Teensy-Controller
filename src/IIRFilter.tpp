#include "IIRFilter.h"


template<size_t order>
IIRFilter<order>::IIRFilter(){
    for(unsigned int i = 0; i < length; i++){
            buffer.unshift(0);
        }
};

template<size_t order>
IIRFilter<order>::IIRFilter(int32_t a[order+1] , int32_t b[order+1] ){
        this->a_coefficients[order+1] = *a;
        this->b_coefficients[order+1] = *b;

        for(unsigned int i = 0; i < length; i++){
            buffer.unshift(0);
        }

};
template<size_t order>
IIRFilter<order>::IIRFilter(float *a_coefficients_fl, float *b_coefficients_fl){
    if(sizeof(a_coefficients)/sizeof(float) == length && sizeof(b_coefficients)/sizeof(float) == length){
            for(unsigned int i = 0; i<order+1; i++){
                this->a_coefficients[i] = float2Fix(a_coefficients_fl[i]);
                this->b_coefficients[i] = float2Fix(b_coefficients_fl[i]);
            }
        }
        for(unsigned int i = 0; i < length; i++){
            buffer.unshift(0);
        }
}
template<size_t order>
IIRFilter<order>::IIRFilter(double *a_coefficients_fl, double *b_coefficients_fl){
    if(sizeof(a_coefficients)/sizeof(double) == length && sizeof(b_coefficients)/sizeof(double) == length){
            for(int i = 0; i<order+1; i++){
                this->a_coefficients[i] = double2Fix(a_coefficients_fl[i]);
                this->b_coefficients[i] = double2Fix(b_coefficients_fl[i]);
            }
        }
        for(unsigned int i = 0; i < length; i++){
            buffer.unshift(0);
        }
}


template<size_t order>
void IIRFilter<order>::setCoefficients(int32_t *a_coefficients , int32_t *b_coefficients){
     if(sizeof(a_coefficients)/sizeof(int32_t) == length && sizeof(b_coefficients)/sizeof(int32_t) == length){
            this->a_coefficients = a_coefficients;
            this->b_coefficients = b_coefficients;
        }
};

template<size_t order>
void IIRFilter<order>::setCoefficients(float *a_coefficients_fl, float *b_coefficients_fl){
    if(sizeof(a_coefficients)/sizeof(float) == length && sizeof(b_coefficients)/sizeof(float) == length){
            for(int i = 0; i<order+1;i++){
                this->a_coefficients[i] = float2Fix(a_coefficients_fl[i]);
                this->b_coefficients[i] = float2Fix(b_coefficients_fl[i]);
            }
        }
}

template<size_t order>
void IIRFilter<order>::setCoefficients(double *a_coefficients_fl, double *b_coefficients_fl){
    if(sizeof(a_coefficients)/sizeof(double) == length && sizeof(b_coefficients)/sizeof(double) == length){
            for(int i = 0; i<order+1;i++){
                this->a_coefficients[i] = double2Fix(a_coefficients_fl[i]);
                this->b_coefficients[i] = double2Fix(b_coefficients_fl[i]);
            }
        }
}



template<size_t order>
void IIRFilter<order>::compute(){

    #ifdef IIRDEBUG
    for(int i = 0; i < order+1; i++){
        Serial.print("a: ");
        Serial.println(fix2Float(a_coefficients[i]));
        Serial.print("b: ");
        Serial.println(fix2Float(b_coefficients[i]));
    }
    #endif
    int32_t w_n = 0;
    for(unsigned int n = 1; n < length; n++){
        w_n += multiply(a_coefficients[n],buffer[n]);
    }
    w_n += input;
    w_n = multiply(w_n,a_coefficients[0]);
    
    #ifdef IIRDEBUG
    Serial.print("w_n: ");
    Serial.println(fix2Float(w_n));
    #endif
    buffer.unshift(w_n);
    output = 0;
    for(unsigned int n = 0; n< length; n++){
        output += multiply(b_coefficients[n],buffer[n]);
    }
}


template<size_t order>
void IIRFilter<order>::setInput(int32_t input){
    this->input = input;
}
template<size_t order>
void IIRFilter<order>::setInput(float input){
    this->input = float2Fix(input);
}
template<size_t order>
void IIRFilter<order>::setInput(double input){
    this->input = double2Fix(input);
}
template<size_t order>
int32_t IIRFilter<order>::getOutput(){
    return output;
}
template<size_t order>
float IIRFilter<order>::getOutputFloat(){
    return fix2Float(output);
}
template<size_t order>
double IIRFilter<order>::getOutputDouble(){
    return fix2Double(output);
}
template<size_t order>
void IIRFilter<order>::computePeriodic(){
    if(micros - lasttime >= this->period){
        lasttime = micros();
        this->compute();
    }
}




