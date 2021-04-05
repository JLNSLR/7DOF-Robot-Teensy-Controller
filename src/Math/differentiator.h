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
    void setTimeStep(double timestep); //seconds
    void setTimeStep(int timestep);    //microseconds
    void setFrequency(int frequency);

    void setInput(float input);
    float getOutput();

private:
    double timestep = 0.003333;

    int timestep_micro = 3333;

    CircularBuffer<float, 3> buffer;
    int smoother_length = 20;
    CircularBuffer<float, 20> smoother;


    float input = 0;
    float output = 0;
};

#endif //DIFFERENTIATOR_H