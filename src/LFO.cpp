#define _USE_MATH_DEFINES
#include <math.h>

#include "LFO.h"

LFO::LFO(
    float sr
    , double freq
    , int depth
    , double phase
    , LfoType startType)
    : _sr(sr)
    , _freq(freq)
    , _depth(depth)
    , _phase(phase)
    , _currType(startType)
{
    
}

LFO::~LFO()
{
}

double LFO::lfo(double phase, LfoType type)
{
    switch (type)
    {
    case lfoTriangle:
        if (phase < 0.25f)
            return 0.5f + 2.0f * phase;
        else if (phase < 0.75f)
            return 1.0f - 2.0f * (phase - 0.25f);
        else
            return 2.0f * (phase - 0.75f);
    case lfoSquare:
        if (phase < 0.5f)
            return 1.0f;
        else
            return 0.0f;
    case lfoSlopedSquare:
        if (phase < 0.48f)
            return 1.0f;
        else if (phase < 0.5f)
            return 1.0f - 50.f * (phase - 0.48f);
        else if (phase < 0.98f)
            return 0.0f;
        else
            return 50.0f * (phase - 0.98f);
    case lfoCos:
    default:
        return 0.5f + 0.5f * sinf(2.0f * M_PI * phase);
    }
}

double LFO::tick()
{
    // calculate lfo value
    double lfov = lfo(_phase, _currType);
    
    // update LFO phase
    _phase += _freq * (1.0 / 44100.0);
    if (_phase >= 1.0)
        _phase -= 1.0;

    return lfov * _depth;
}

LfoType LFO::setType(LfoType newType)
{
    return _currType = newType;
}

double LFO::setFreq(double newFreq)
{
    if (newFreq <= LFO_MIN_FREQ)
        // throw
        newFreq = 1.0f;
    if (newFreq >= LFO_MAX_FREQ)
        // throw
        newFreq = 20.0f;

    return _freq = newFreq;
}

double LFO::setDepth(double newDepth)
{
    return _depth = newDepth;
}

float LFO::getSampleRate()
{
    return _sr;
}

double LFO::getDepth()
{
    return _depth;
}
    
