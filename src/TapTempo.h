#include <math_neon.h>
#include <vector>

#define DEFAULT_TAP_ARRAY_SIZE 3
/***** TapTempo.h *****/
class TapTempo
{
public:
    TapTempo(int size = DEFAULT_TAP_ARRAY_SIZE)
    {
        _tempoMsIntervals.resize(size);
    }
    
    ~TapTempo() { }
    
    void tap()
    {
        // calculate time between this tap and last tap
        float tick2MS = _tapTimerTicks * _inverseSampleRate * 1000.0f;
        _tempoMsIntervals[_ndx] = tick2MS;
        float diff = fabsf_neon(_tempoMsIntervals[_ndx] - _tempoMsIntervals[_lastNdx]);
        _lastNdx = _ndx;
        _ndx++;
        // make sure no negative difference
        if (diff < 0.0) { diff = 0.0; }
        // asign the tapped tempo to the time since the last 2 taps
        _tapTempoMS = diff;
        // wrap index
        if (_ndx == _tempoMsIntervals.size()) { _ndx = 0; }
    }
    
    void tick()
    {
        _tapTimerTicks++;
    }
 
    void    setInverseSampleRate(float isr) { _inverseSampleRate = isr; }
    float    getTapTempoMS()                    { return _tapTempoMS; }
    
private:

    std::vector<float> _tempoMsIntervals;
    float _tapTempoMS;
    long _tapTimerTicks;
    float _inverseSampleRate;
    long _ndx;
    long _lastNdx;
};
