#include <Math/differentiator.h>

Differentiator::Differentiator()
{
    buffer.unshift(0);
    buffer.unshift(0);
    buffer.unshift(0);
};
Differentiator::Differentiator(int32_t timestep)
{
    Differentiator();
    timestep_f = timestep;
};
Differentiator::Differentiator(float timestep)
{
    Differentiator();
    timestep_f = float2Fix(timestep);
}
Differentiator::Differentiator(int frequency)
{
    timestep = 1 / frequency;
    setTimeStep(timestep);
}

void Differentiator::setTimeStep(float timestep)
{
    timestep_f = float2Fix(timestep);
}
void Differentiator::setTimeStep(int32_t timestep_f)
{
    this->timestep_f = timestep_f;
}
void Differentiator::setFrequency(int frequency)
{
    timestep = 1 / frequency;
    setTimeStep(timestep);
}

void Differentiator::setInput(int32_t input)
{
    buffer.unshift(input);
}

void Differentiator::differentiate()
{
    int32_t y_timeStepPlus = buffer.first();
    int32_t y_timeStepMinus = buffer.pop();

    output = divide(y_timeStepPlus - y_timeStepMinus, multiply(timestep_f, 2));
}

int32_t Differentiator::getOutput()
{
    return output;
}
