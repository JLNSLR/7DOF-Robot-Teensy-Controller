#include "FIRFilter.h"


template<size_t order>
FIRFilter<order>::FIRFilter(){
    for(unsigned int i = 0; i < length; i++){
            buffer.unshift(0);
        }
};

template<size_t order>
FIRFilter<order>::FIRFilter(int32_t b[order+1] ){
        this->b_coefficients[order+1] = *b;

        for(unsigned int i = 0; i < length; i++){
            buffer.unshift(0);
        }

};
template<size_t order>
FIRFilter<order>::FIRFilter(float *b_coefficients_fl){
    if(sizeof(b_coefficients)/sizeof(float) == length){
            for(int i = 0; i<order+1; i++){
                this->b_coefficients[i] = float2Fix(b_coefficients_fl[i]);
            }
        }
        for(unsigned int i = 0; i < length; i++){
            buffer.unshift(0);
        }
}
template<size_t order>
FIRFilter<order>::FIRFilter(double *b_coefficients_fl){
    if(sizeof(b_coefficients)/sizeof(double) == length){
            for(int i = 0; i<order+1; i++){
                this->b_coefficients[i] = double2Fix(b_coefficients_fl[i]);
            }
        }
        for(unsigned int i = 0; i < length; i++){
            buffer.unshift(0);
        }
}


template<size_t order>
void FIRFilter<order>::setCoefficients(int32_t *b_coefficients){
     if(sizeof(b_coefficients)/sizeof(int32_t) == length){
            this->b_coefficients = b_coefficients;
        }
};

template<size_t order>
void FIRFilter<order>::setCoefficients(float *b_coefficients_fl){
    if(sizeof(b_coefficients)/sizeof(float) == length){
            for(int i = 0; i<order+1;i++){
                this->b_coefficients[i] = float2Fix(b_coefficients_fl[i]);
            }
        }
}

template<size_t order>
void FIRFilter<order>::setCoefficients( double *b_coefficients_fl){
    if(sizeof(b_coefficients)/sizeof(double) == length){
            for(int i = 0; i<order+1;i++){
                this->b_coefficients[i] = double2Fix(b_coefficients_fl[i]);
            }
        }
}



template<size_t order>
void FIRFilter<order>::compute(){

    #ifdef FIRDEBUG
    for(int i = 0; i < order+1; i++){
        Serial.print("b: ");
        Serial.println(fix2Float(b_coefficients[i]));
    }
    #endif

    buffer.unshift(input);
    for(int n = 0; n<length; n++){
        output +=multiply(b_coefficients,buffer[n]);
    }
};


template<size_t order>
void FIRFilter<order>::setInput(int32_t input){
    this->input = input;
}
template<size_t order>
void FIRFilter<order>::setInput(float input){
    this->input = float2Fix(input);
}
template<size_t order>
void FIRFilter<order>::setInput(double input){
    this->input = double2Fix(input);
}
template<size_t order>
int32_t FIRFilter<order>::getOutput(){
    return output;
}
template<size_t order>
float FIRFilter<order>::getOutputFloat(){
    return fix2Float(output);
}
template<size_t order>
double FIRFilter<order>::getOutputDouble(){
    return fix2Double(output);
}

template<size_t order>
void FIRFilter<order>::computePeriodic(){
    if(micros - lastTime >= this->period){
        lastTime = micros();
        compute();
    }
}




