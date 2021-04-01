#ifndef FIRFILTER_H
#define FIRFILTER_H

#include "CircularBuffer.h"

//#define FIRDEBUG

template <size_t order>
class FIRFilter
{
public:
    FIRFilter();
    FIRFilter(float *b_coefficients);
    FIRFilter(double *b_coefficients);

    void setCoefficients(float *b_coefficients);
    void setCoefficients(double *b_coefficients);

    void inline compute();

    void inline computePeriodic();

    void setInput(float input);
    void setInput(double input);

    float getOutput();

    float input = 0;
    float output = 0;

    unsigned long period = 0; //us

private:
    unsigned long lastTime = 0;
    static constexpr size_t length = order + 1;

    float b_coefficients[order + 1];

    CircularBuffer<float, order + 1> buffer;
};

#include "FIRFilter.tpp"
#endif //FIRFILTER_H