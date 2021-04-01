#ifndef DIFFERENTIATOR_H
#define DIFFERENTIATOR_H
#include <CircularBuffer.h>

class Differentiator
{
public:
    Differentiator();
    Differentiator(float timestep);
    Differentiator(int frequency);

    void differentiate();
    void setTimeStep(float timestep); //seconds
    void setTimeStep(int timestep);   //microseconds
    void setFrequency(int frequency);

    void setInput(float input);
    float getOutput();

private:
    float timestep = 0.003333;

    int timestep_micro = 3333;

    CircularBuffer<float, 3> buffer;

    int32_t input = 0;
    int32_t output = 0;
};

#endif //DIFFERENTIATOR_H