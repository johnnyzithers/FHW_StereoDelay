#include <vector>
#include <Bela.h>
#include <algorithm>    // std::min
#include <math_neon.h>
#include "LFO.h"
#include <libraries/Scope/Scope.h>
#include "TapTempo.h"

#define DELAY_MOD_SAMPLES 10000    // samples for LFO modulation
#define LFO_FREQ_RANGE 2        

#define DELAY_BUFFER_SIZE 176400
#define CONTROL_FRAME_SIZE 8192

#define DELAY_SAMPLE_ERR 60    // for potentiometer drift
#define DELAY_GROWTH_AMOUNT 0.100
#define DELAY_CROSSFADE_CUTOFF 20000
#define DELAY_TIME_ADJUST 0.001
#define DELAY_MAX_ADJUST_SAMPS 20
LFO z_LFO;
TapTempo tapt;

// instantiate the scope
Scope scope;

float gTapeStopCoeff = 1.0;

int momentaryVal = 0;

// prev samples buffer{
float gDelayBuffer_l[DELAY_BUFFER_SIZE] = {0};        
float gDelayBuffer_r[DELAY_BUFFER_SIZE] = {0};

int gDelayParamCount = 0;
int gDelayBufWritePtr = 0;
float gDelayBufReadPtr = 0;
float gNewDelayBufReadPtr = 0;
float gNewLastDelayBufReadPtr = 0;
float gLastDelayBufReadPtr = 0;
 
// delay vars
float gDelayGrowth = 0;
float gDelayTimeMS = 1000;    // has to have initial value for scaled potentiometer interaction 
float gDelayTimePot = 0; 
float gLastDelayTimePot = 0; 

float gDelayAmount = 1.0;
float gDelayFeedbackAmount = 0.999;
float gDelayAmountPre = 0.9;            // use this for delay volume
float gDelayInSamples = 44100.0;
float gTargetDelayInSamples = 44100;

float gEffectMix;
float gEffectParam;

// Butterworth coefficients for low-pass filter @ 8000Hz
float gDel_a0 = 0.1772443606634904;
float gDel_a1 = 0.3544887213269808;
float gDel_a2 = 0.1772443606634904;
float gDel_a3 = -0.5087156198145868;
float gDel_a4 = 0.2176930624685485;

// Previous two input and output values for each channel (required for applying the filter)
float gDel_x1_l = 0;
float gDel_x2_l = 0;
float gDel_y1_l = 0;
float gDel_y2_l = 0;
float gDel_x1_r = 0;
float gDel_x2_r = 0;
float gDel_y1_r = 0;
float gDel_y2_r = 0;

// for digital reading the right frames
int gAudioFramesPerAnalogFrame;
float gInverseSampleRate;
float gPhase;


// print vars
float    gInterval = 0.5;
float    gSecondsElapsed = 0;
int     gCount = 0;

std::vector<bool> a_buttonState = {0,0,0,0};
std::vector<bool> b_buttonState = {0,0,0,0};
std::vector<int> a_lastButtonValues = {0,0,0,0};
std::vector<int> b_lastButtonValues = {0,0,0,0};

// tap tempo 
float    gTapTempoMSIntervals[3] = {DELAY_BUFFER_SIZE,DELAY_BUFFER_SIZE,DELAY_BUFFER_SIZE};
int     gTapTimerTicks;
int     gTapNdx = 0;
int     gTapLastNdx = 0;
float    gTapTempoMS;
float    gDelayTimeArr[8] = {0,0,0,0,0,0,0,0};

AuxiliaryTask gSetDelayTimeTask;

int gCrossfadeFlag = 0;
int gCrossfadeCounter = 1;
int gCrossfadeLength = 1024;

int     gTempFlag = 0;
float    gLastTarget;

// this is responsible for all the tape stuff
void setDelayTime_Background(void* arg)
{
    if (momentaryVal == 1)
    {
        gLastTarget = gTargetDelayInSamples;
    
        // convert gDelayTimeMS set by pot to target number of delay samples
        gTargetDelayInSamples = *((float*)arg) / 1000.0 * (22050.0);
        
        // target delay is 
        float diff = gTargetDelayInSamples - gDelayInSamples;
        
        // if this is a "small" delay change
        if(fabsf_neon(gDelayInSamples - gTargetDelayInSamples) < DELAY_CROSSFADE_CUTOFF)
        {
            // delay is shrinking
            if (gDelayInSamples > gTargetDelayInSamples)  
            {
                        
                float delaySampleChange = fabsf_neon(diff) * DELAY_TIME_ADJUST;
                if (delaySampleChange > DELAY_MAX_ADJUST_SAMPS) { delaySampleChange = DELAY_MAX_ADJUST_SAMPS; }
                
                gDelayInSamples -= delaySampleChange;
                if( fabsf_neon(gDelayInSamples - gTargetDelayInSamples) > DELAY_SAMPLE_ERR )
                    gDelayGrowth = DELAY_GROWTH_AMOUNT;
                else
                    gDelayGrowth = 0;
            }
            // delay is growing
            else if (gDelayInSamples < gTargetDelayInSamples)
            {    
                
                float delaySampleChange = fabsf_neon(diff) * DELAY_TIME_ADJUST;
                if (delaySampleChange > DELAY_MAX_ADJUST_SAMPS) { delaySampleChange = DELAY_MAX_ADJUST_SAMPS; }
                
                
                gDelayInSamples += delaySampleChange;
                if( fabsf_neon(gDelayInSamples - gTargetDelayInSamples) > DELAY_SAMPLE_ERR )
                    gDelayGrowth = -1.0*DELAY_GROWTH_AMOUNT;
                else 
                    gDelayGrowth = 0;
            }
            else 
            {
                gDelayGrowth = 0.0;
            }
            gCrossfadeFlag = 0;
            gCrossfadeCounter = 0;
        }
        // otherwise this is a "big" delay change, so we while crossfade buffers instead
        else{
            gCrossfadeFlag = 1;
        }    
    }
    else{
        // float delaySampleChange = DELAY_MAX_ADJUST_SAMPS;
        // gDelayInSamples += 66;
        // gDelayGrowth = -1.0*DELAY_GROWTH_AMOUNT;
        // gDelayAmountPre = 0.0;
        // gDelayFeedbackAmount = 0.0;
    }
    
    if    (gDelayInSamples < 0)    { gDelayInSamples = 0; }
    if    (gDelayInSamples > 88200)    { gDelayInSamples = 88200; }
}

bool setup(BelaContext *context, void *userData)
{
    if((gSetDelayTimeTask = Bela_createAuxiliaryTask(&setDelayTime_Background, 90, "set-delay-time", (void*)&gDelayTimeMS)) == 0){
        return false;
    }
    
    // lfo params
    z_LFO.setFreq(1.0);
    z_LFO.setDepth(1.0);
    
    
    scope.setup(3, context->audioSampleRate);
      
    // Useful calculations
    gAudioFramesPerAnalogFrame = context->audioFrames / context->analogFrames;
    gInverseSampleRate = 1.0 / context->audioSampleRate;
    
    tapt.setInverseSampleRate(gInverseSampleRate);
    
    
    gPhase = 0.0;
    
    return true;
}

void zeroTapTempoArray() {
    gTapTempoMSIntervals[0] = DELAY_BUFFER_SIZE;
    gTapTempoMSIntervals[1] = DELAY_BUFFER_SIZE;
    gTapTempoMSIntervals[2] = DELAY_BUFFER_SIZE;
}

void render(BelaContext *context, void *userData)
{
    int toggleVal = 0; 
    std::vector<int> buttonPadValues = {0,0,0,0};

     for(unsigned int n = 0; n < context->audioFrames; n++) 
     {
             tapt.tick();
             gTapTimerTicks++;

            // There are twice as many audio frames as matrix frames
            if(!(n % (gAudioFramesPerAnalogFrame))) {
                //
                buttonPadValues[0] = digitalRead(context, 0, 14);
                buttonPadValues[1] = digitalRead(context, 0, 15);
                buttonPadValues[2] = digitalRead(context, 0, 13);
                buttonPadValues[3] = digitalRead(context, 0, 12);
                //
                momentaryVal = digitalRead(context, 0, 0);
                toggleVal = digitalRead(context, 0, 1);
                //
                gEffectMix = constrain(map(analogRead(context, n/gAudioFramesPerAnalogFrame, 0), 0, 0.85, 0, 1), 0, 1);
                gEffectParam = constrain(map(analogRead(context, n/gAudioFramesPerAnalogFrame, 1), 0, 0.85, 1, 0), 0, 1);
                gDelayFeedbackAmount = constrain(map(analogRead(context, n/gAudioFramesPerAnalogFrame, 3), 0, 0.8, 1.4, 0), 0, 1.4);
                gDelayAmountPre = constrain(map(analogRead(context, n/gAudioFramesPerAnalogFrame, 4), 0, 0.85, 1, 0), 0, 1);
                gDelayAmount = constrain(map(analogRead(context, n/gAudioFramesPerAnalogFrame, 5), 0, 0.85, 1, 0), 0, 0.9999);
                gDelayTimePot = constrain(map(analogRead(context, n/gAudioFramesPerAnalogFrame, 2), 0, 0.85, 0.0, (DELAY_BUFFER_SIZE/2)*gInverseSampleRate), 0.0, (DELAY_BUFFER_SIZE/2)*gInverseSampleRate);
                // gDelayTimePot = constrain(map(analogRead(context, n/gAudioFramesPerAnalogFrame, 2), 0, 0.85, gDelayTimeMS/2.0, gDelayTimeMS*2.0), gDelayTimeMS/2.0, gDelayTimeMS*2.0);
            
                z_LFO.setFreq((gEffectMix * LFO_FREQ_RANGE) + 1);
                
                // attempt to filter out pot wobble
                if( fabsf_neon((int)(gDelayTimePot * 100) - (int)(gLastDelayTimePot * 100)) > 4.0 ){
                    gDelayTimeMS = 1000.0 * gDelayTimePot;
                    gDelayTimeArr[0] = gDelayTimeMS;
                    gDelayTimeArr[1] = gDelayTimeMS / 2.0;            
                    gDelayTimeArr[2] = gDelayTimeMS / 4.0;
                    gDelayTimeArr[3] = gDelayTimeMS / 8.0;
                    gDelayTimeArr[4] = gDelayTimeMS * (2.0 / 3.0);
                    gDelayTimeArr[5] = gDelayTimeMS * (1.0 / 6.0);
                    gDelayTimeArr[6] = gDelayTimeMS * (1.0 / 6.0);
                    gLastDelayTimePot = gDelayTimePot;
                }
                
                // straight tempo divisions
                if (toggleVal == 0) {
                    // loop through buttons
                    for (int i = 0; i < 4; i++) {
                        // set the delay time if its different
                        if( buttonPadValues[i] == 1 && (a_lastButtonValues[i] == 0) ){
                            gDelayTimeMS = gDelayTimeArr[i];
                            zeroTapTempoArray();
                            a_buttonState[i] = !a_buttonState[i];
                        }    
                        // save last values
                        a_lastButtonValues[i] = buttonPadValues[i];
                        b_lastButtonValues[i] = buttonPadValues[i];
                    }
                } 
                // tap tempo and dotted tempo divisions
                else if (toggleVal == 1) {
                    // tap tempo
                    if( buttonPadValues[0] == 0 && (b_lastButtonValues[0] == 1) ) {
                                            
                                            
                                            
                        tapt.tap();
                        gTapTempoMS = tapt.getTapTempoMS();

                
                    }
                    // save last values
                    b_lastButtonValues[0] = buttonPadValues[0];

                    // DOTTED rhythms, skip 0 index 
                    for (int i = 1; i < 4; i++) {
                        //set delay times according to divisions
                        if( buttonPadValues[i] == 1 && (b_lastButtonValues[i] == 0) ){
                            gDelayTimeMS = gDelayTimeArr[i+3];
                            zeroTapTempoArray();
                            b_buttonState[i] = !b_buttonState[i];
                        }                        
                        b_lastButtonValues[i] = buttonPadValues[i];
                        a_lastButtonValues[i] = buttonPadValues[i];
                    }
                }
            
            }
        
            // schedule the task to change the delay time
            if(++gDelayParamCount > CONTROL_FRAME_SIZE){
                Bela_scheduleAuxiliaryTask(gSetDelayTimeTask);        // launch
            }
    
            float out_l = 0.0;
            float out_r = 0.0;
            float del_out_l = 0.0;
            float del_out_r = 0.0;
            float new_del_l = 0.0;
            float new_del_r = 0.0;
            float dry_l = 0.0;
            float dry_r = 0.0;
            
            // Read audio inputs
            dry_l = audioRead(context,n,0);
            dry_r = audioRead(context,n,1);
           
            // Increment delay buffer write pointer
            if(++gDelayBufWritePtr>=DELAY_BUFFER_SIZE)
                gDelayBufWritePtr = 0;
            
            // Calculate the sample that will be written into the delay buffer...
            float del_input_l = ((gDelayAmountPre * dry_l) + gDelayBuffer_l[(int)(gDelayBufWritePtr-gDelayInSamples+DELAY_BUFFER_SIZE)%DELAY_BUFFER_SIZE] * gDelayFeedbackAmount);
            float del_input_r = ((gDelayAmountPre * dry_r) + gDelayBuffer_r[(int)(gDelayBufWritePtr-gDelayInSamples+DELAY_BUFFER_SIZE)%DELAY_BUFFER_SIZE] * gDelayFeedbackAmount);
            
            // save values for later
            float temp_x_l = del_input_l;
            float temp_x_r = del_input_r;
            
            // Apply the butterworth filter (y = a0*x0 + a1*x1 + a2*x2 + a3*y1 + a4*y2)
            del_input_l = gDel_a0*del_input_l
                        + gDel_a1*gDel_x1_l
                        + gDel_a2*gDel_x2_l
                        + gDel_a3*gDelayBuffer_l[(gDelayBufWritePtr-1+DELAY_BUFFER_SIZE)%DELAY_BUFFER_SIZE]
                        + gDel_a4*gDelayBuffer_l[(gDelayBufWritePtr-2+DELAY_BUFFER_SIZE)%DELAY_BUFFER_SIZE];
            
            // Update previous values for next iteration of filter processing
            gDel_x2_l = gDel_x1_l;
            gDel_x1_l = temp_x_l;
            gDel_y2_l = gDel_y1_l;
            gDel_y1_l = del_input_l;
            
            // Repeat process for the right channel
            del_input_r = gDel_a0*del_input_r
                        + gDel_a1*gDel_x1_r
                        + gDel_a2*gDel_x2_r
                        + gDel_a3*gDelayBuffer_r[(gDelayBufWritePtr-1+DELAY_BUFFER_SIZE)%DELAY_BUFFER_SIZE]
                        + gDel_a4*gDelayBuffer_r[(gDelayBufWritePtr-2+DELAY_BUFFER_SIZE)%DELAY_BUFFER_SIZE];
        
            gDel_x2_r = gDel_x1_r;
            gDel_x1_r = temp_x_r;
            gDel_y2_r = gDel_y1_r;
            gDel_y1_r = del_input_r;
            //  Now we can write it into the delay buffer
            gDelayBuffer_l[gDelayBufWritePtr] = del_input_l;
            gDelayBuffer_r[gDelayBufWritePtr] = del_input_r;

            // we update the read pointer
             gLastDelayBufReadPtr = gDelayBufReadPtr;
             
             // tape stop / restart handled hear
            if (momentaryVal == 1) {
                if (gTapeStopCoeff < 1.0 ) {
                    gTapeStopCoeff = 1.001*gTapeStopCoeff;     
                    gDelayBufReadPtr += gTapeStopCoeff;
                } else {
                    gDelayBufReadPtr += (1 - gDelayGrowth);
                }
            }else {
                gTapeStopCoeff = 0.9999*gTapeStopCoeff;                
                gDelayBufReadPtr += gTapeStopCoeff;

            }

            scope.log(gDelayGrowth, gDelayTimeMS, momentaryVal);

               
            // wrap read pointer
            if(gDelayBufReadPtr >= DELAY_BUFFER_SIZE)
                gDelayBufReadPtr -= DELAY_BUFFER_SIZE; 
            if(gDelayBufReadPtr < 0)
                gDelayBufReadPtr += DELAY_BUFFER_SIZE; 
        
            // fractional delay computation
            long rpi = (long)floorf_neon(gDelayBufReadPtr);
            double a = gDelayBufReadPtr - (double)rpi;
            del_out_l = (a * gDelayBuffer_l[rpi]) + ((1-a) * gDelayBuffer_l[rpi+1]);
            del_out_r = (a * gDelayBuffer_r[rpi]) + ((1-a) * gDelayBuffer_r[rpi+1]);
      
            // and we interpolate fractionally from the last read pointer too!
            long lrpi = (long)floorf_neon(gLastDelayBufReadPtr);
            double la = gDelayBufReadPtr - (double)rpi;
            del_out_l += 0.5 * ( (la * gDelayBuffer_l[lrpi]) + ((1-la) * gDelayBuffer_l[lrpi+1]) );
            del_out_r += 0.5 * ( (la * gDelayBuffer_r[lrpi]) + ((1-la) * gDelayBuffer_r[lrpi+1]) );
           
            // // cross fade across large sample delay changes
            // if ((gCrossfadeFlag==1) && (gCrossfadeCounter <= gCrossfadeLength))
            // {
            //     float delaySampleChange = 50;
            //     // delay is shrinking
            //     if (gDelayInSamples > gTargetDelayInSamples)  
            //     {
            //         gDelayInSamples -= delaySampleChange;
            //         if( fabsf_neon(gDelayInSamples - gTargetDelayInSamples) > DELAY_SAMPLE_ERR )
            //         {
            //             gDelayGrowth = DELAY_GROWTH_AMOUNT;
            //         } else {
            //             gDelayGrowth = 0;
            //         }
            //     }
            //     // delay is growing
            //     else if (gDelayInSamples < gTargetDelayInSamples)
            //     {    
            //         gDelayInSamples += delaySampleChange;
            //         if( fabsf_neon(gDelayInSamples - gTargetDelayInSamples) > DELAY_SAMPLE_ERR )
            //         {
            //             gDelayGrowth = -1 * DELAY_GROWTH_AMOUNT;    
            //         } else {
            //             gDelayGrowth = 0;
            //         }
            //     }
            //     else 
            //     {
            //         gDelayGrowth = 0.0;
            //     }
                
            //     // we update the read pointer
         //        gNewLastDelayBufReadPtr = gNewDelayBufReadPtr;
            //     gNewDelayBufReadPtr = (gDelayBufWritePtr - gTargetDelayInSamples + DELAY_BUFFER_SIZE);
         //       gNewDelayBufReadPtr += (1 - gDelayGrowth);
        
            //     // wrap read pointer
            //     if(gNewDelayBufReadPtr >= DELAY_BUFFER_SIZE)
            //         gNewDelayBufReadPtr -= DELAY_BUFFER_SIZE; 
            //     if(gNewDelayBufReadPtr < 0)
            //         gNewDelayBufReadPtr += DELAY_BUFFER_SIZE; 
                    
         //       // fractional delay computation
         //       long rpi = (long)floorf_neon(gNewDelayBufReadPtr);
         //       double a = gNewDelayBufReadPtr - (double)rpi;
         //       new_del_l = (a * gDelayBuffer_l[rpi]) + ((1-a) * gDelayBuffer_l[rpi+1]);
         //       new_del_r = (a * gDelayBuffer_r[rpi]) + ((1-a) * gDelayBuffer_r[rpi+1]);

            //     // crossfade based on counter
         //       gCrossfadeCounter += 1;
         //       del_out_l = ((gCrossfadeCounter/gCrossfadeLength) * del_out_l) + ((gCrossfadeLength - gCrossfadeCounter)/gCrossfadeLength * new_del_l);
         //       del_out_r = ((gCrossfadeCounter/gCrossfadeLength) * del_out_r) + ((gCrossfadeLength - gCrossfadeCounter)/gCrossfadeLength * new_del_r);
            // }
            
            // if(gCrossfadeCounter == gCrossfadeLength)
            // {
            //     gCrossfadeCounter = 0;
            // }

            out_l += gDelayAmount*del_out_l;
            out_r += gDelayAmount*del_out_r;

            // Write the sample into the output buffer -- done!
            audioWrite(context, n, 0, del_out_l);
            audioWrite(context, n, 1, del_out_r);
            
            gCount++; 
            if(gCount % (int)(context->audioSampleRate*gInterval) == 0) {
                gSecondsElapsed += gInterval;
                rt_printf("samps %f %f %f %f\n",gTapTempoMSIntervals[0], gTapTempoMSIntervals[1], gDelayTimeMS, gTapTempoMS);
            }
        }
}

void cleanup(BelaContext *context, void *userData)
{

}
