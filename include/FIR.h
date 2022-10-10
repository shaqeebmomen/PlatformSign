#include <stddef.h>

template <unsigned int TAPS>
class FIR
{
protected:
    float *constants;    // Filter Constants
    float history[TAPS]; // Previous inputs
    float output;
    void shiftInput(float input); // Shift in the input data
    void multiAccum();          // Perform multiply + accumulate operation

public:
    FIR<TAPS>(float taps[TAPS]);
    float getOutput(float input);
    float getOutput();
};

template <unsigned int TAPS>
FIR<TAPS>::FIR(float *consts)
{
    for (size_t i = 0; i < TAPS; i++)
    {
        this->history[i] = 0;
    }
    this->constants = consts;
}

template <unsigned int TAPS>
float FIR<TAPS>::getOutput()
{
    return this->output;
}

template <unsigned int TAPS>
float FIR<TAPS>::getOutput(float input)
{
    shiftInput(input);
    multiAccum();
    return this->output;
}

template <unsigned int TAPS>
void FIR<TAPS>::shiftInput(float input)
{
    this->history[0] = input; // Shift latest input into the zeroth index
    for (size_t i = TAPS; i > 0; i--)
    {
        this->history[i] = this->history[i - 1];
    }
}

template <unsigned int TAPS>
void FIR<TAPS>::multiAccum()
{
    this->output = 0; // Reset output
    for (size_t i = 0; i < TAPS; i++)
    {
        output += history[i] * constants[i]; // Accumulate the product of the historic input and the corresponding constant
    }
}
