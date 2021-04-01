#ifndef DIFFERENTIATOR_H
#define DIFFERENTIATOR_H
#include <Math/FixPointMath.h>
#include <CircularBuffer.h>

class Differentiator
{
public:
    Differentiator();
    Differentiator(int32_t timestep);
    Differentiator(float timestep);
    Differentiator(int frequency);

    void differentiate();
    void setTimeStep(float timestep);
    void setTimeStep(int32_t timestep);
    void setFrequency(int frequency);

    void setInput(int32_t input);
    int32_t getOutput();

private:
    float timestep = 0.003333;

    int32_t timestep_f = 1;

    CircularBuffer<int32_t, 3> buffer;

    int32_t input = 0;
    int32_t output = 0;
};

#endif //DIFFERENTIATOR_H