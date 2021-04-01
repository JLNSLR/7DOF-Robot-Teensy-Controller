#include <Math/differentiator.h>

Differentiator::Differentiator()
{
    buffer.unshift(0);
    buffer.unshift(0);
    buffer.unshift(0);
};
Differentiator::Differentiator(float timestep)
{
    Differentiator();
    timestep_micro = timestep * 1000 * 1000;
}
Differentiator::Differentiator(int frequency)
{
    timestep = 1 / frequency;
    setTimeStep(timestep);
}

void Differentiator::setTimeStep(float timestep)
{
    timestep_micro = timestep * 1000 * 1000;
    this->timestep = timestep;
}
void Differentiator::setTimeStep(int timestep)
{
    this->timestep_micro = timestep;
    timestep = timestep_micro / (1000 * 1000);
}
void Differentiator::setFrequency(int frequency)
{
    timestep = 1 / frequency;
    setTimeStep(timestep);
}

void Differentiator::setInput(float input)
{
    buffer.unshift(input);
}

void Differentiator::differentiate()
{
    int32_t y_timeStepPlus = buffer.first();
    int32_t y_timeStepMinus = buffer.pop();

    output = (y_timeStepPlus - y_timeStepMinus) / timestep * 2;
}

float Differentiator::getOutput()
{
    return output;
}
