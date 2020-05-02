#pragma once

#include <math.h>

#define DEBUG_PRINT_SEQ false
#define DEBUG_PRINT_ADSR false
#define DEBUG_PRINT_GPIO true 


#define DEFAULT_LEN_SAMPS 22050   // 0.5 second delay default 
#define DEFAULT_FB 0.5            // 50% feedback default
#define DEFAULT_MIX 0.5           // 50% mix default
#define DEFAULT_LFO_FREQ 2.0
#define DEFAULT_LFO_WIDTH 22050
#define LFO_MAX_FREQ 20.0
#define LFO_MIN_FREQ 0.1

#define DEFAULT_LP_FREQ 200.0
#define DEFAULT_GAIN_DB 0
#define MAX_DELAY_LEN 44100*4
#define MAX_DELAY_LEN_CHORUS 44100

#define DEFAULT_SAMPLE_RATE 44100
#define DEFAULT_FRAME_SIZE 1024

#define DEFAULT_OSC_FREQ 440    
#define OSC_MIN_FREQ 20.0
#define OSC_MAX_FREQ 10000.0

#define DEFAULT_SEQUENCER_LENGTH 8
#define DEFAULT_BPM 90

inline int millisecsToSamples(float millisecs, int sr) { return int((millisecs / 1000.0f) * sr); }

// FIXME not exact
inline float noteNumToFreq(int noteNum) { return pow(2, noteNum/12) * 16.37f; }

// 9  = 27.5
// 12 = 32.75
// 21 = 55
// 24 = 65.5
// 33 = 110
// 36 = 131
// 45 = 220
// 48 = 262
// 57 =  440




