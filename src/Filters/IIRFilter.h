#ifndef IIRFILTER_H
#define IIRFILTER_H

#include "CircularBuffer.h"
#include "Math/FixPointMath.h."

//#define IIRDEBUG

template<size_t order>
class IIRFilter
{
    public:
    IIRFilter();
    IIRFilter(int32_t *a_coefficients , int32_t *b_coefficients );
    IIRFilter(float *a_coefficients, float *b_coefficients);
    IIRFilter(double *a_coefficients, double *b_coefficients);

    void setCoefficients(int32_t *a_coefficients, int32_t *b_coefficients );
    void setCoefficients(float *a_coefficients, float *b_coefficients);
    void setCoefficients(double *a_coefficients, double *b_coefficients);

    void compute();

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

    unsigned long lasttime = 0;

    static constexpr size_t length = order + 1;

    int32_t a_coefficients[order+1];
    int32_t b_coefficients[order+1];

    CircularBuffer<int32_t,order+1> buffer;



};

#include "IIRFilter.tpp"
#endif //IIRFILTER_H