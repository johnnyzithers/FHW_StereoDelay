#pragma once
#include "z_dsp_utilities.h"

enum LfoType {
    lfoTriangle,
    lfoCos,
    lfoSquare,
    lfoSlopedSquare
};


class LFO
{
public:
    LFO(
          float sr = DEFAULT_SAMPLE_RATE
        , double freq = DEFAULT_LFO_FREQ
        , int depth = DEFAULT_LFO_WIDTH
        , double phase = 0.0f
        , LfoType startType = LfoType::lfoCos);
    ~LFO();

    double tick();
    double lfo(double phase, LfoType type);
    LfoType setType(LfoType newType);
    double setFreq(double newFreq);
    double setDepth(double newDepth);
    
    float getSampleRate();
    double getDepth();

private:
    float    _sr;
    double   _freq;
    double   _depth;
    double   _phase;
    LfoType _currType;
};




