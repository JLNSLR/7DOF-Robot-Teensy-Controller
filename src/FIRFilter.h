#ifndef FIRFILTER_H
#define FIRFILTER_H

#include "CircularBuffer.h"
#include "FixPointMath.h."

//#define FIRDEBUG

template<size_t order>
class FIRFilter
{
    public:
    FIRFilter();
    FIRFilter(int32_t *b_coefficients );
    FIRFilter(float *b_coefficients);
    FIRFilter(double *b_coefficients);

    void setCoefficients(int32_t *b_coefficients );
    void setCoefficients(float *b_coefficients);
    void setCoefficients(double *b_coefficients);

    void inline compute();

    void inline computePeriodic();

    void setInput(int32_t input);
    void setInput(float input);
    void setInput(double input);

    int32_t getOutput();
    float getOutputFloat();
    double getOutputDouble();

    int32_t input = 0;
    int32_t output = 0;

    unsigned long period = 0; //us

    private:

    unsigned long lastTime = 0;
    static constexpr size_t length = order + 1;

    int32_t b_coefficients[order+1];

    CircularBuffer<int32_t,order+1> buffer;


};

#include "FIRFilter.tpp"
#endif //FIRFILTER_H