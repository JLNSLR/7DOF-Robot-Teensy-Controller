#include <Math/differentiator.h>
#include <Arduino.h>

Differentiator::Differentiator()
{
    buffer.unshift(0);
    buffer.unshift(0);
    buffer.unshift(0);

    for (int i = 0; i < smoother_length; i++)
    {
        smoother.unshift(0);
    }
};
Differentiator::Differentiator(float timestep)
{
    Differentiator();
    this->timestep = timestep;
    timestep_micro = timestep * 1000 * 1000;
}
Differentiator::Differentiator(int frequency)
{
    timestep = double(1 / frequency);
    setTimeStep(timestep);
}

void Differentiator::setTimeStep(double timestep)
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
    timestep = double(1 / (double)frequency);
    setTimeStep(timestep);
}

void Differentiator::setInput(float input)
{
    buffer.unshift(input);
}

void Differentiator::differentiate()
{
    float y_timeStepPlus = buffer.first();
    float y_timeStepMinus = buffer.pop();

    float intermediate = 0;

    output = (double)((y_timeStepPlus - y_timeStepMinus) / (timestep * 2));

    smoother.unshift(output);
    for (int i = 0; i < smoother_length; i++)
    {
        intermediate += smoother[i];
    }
    output = intermediate / smoother_length;
}

float Differentiator::getOutput()
{
    return output;
}
