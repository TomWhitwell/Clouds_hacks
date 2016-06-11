// Copyright 2014 Olivier Gillet.
// 
// Author: Olivier Gillet (ol.gillet@gmail.com)
// 
// Edited by Matthias 
// make -f clouds/makefile upload_combo_jtag 
// export PGM_INTERFACE=stlink-v2
// export PGM_INTERFACE_TYPE=hla
// See http://creativecommons.org/licenses/MIT/ for more information.

#include "clouds/drivers/codec.h"
#include "clouds/drivers/system.h"
#include "clouds/drivers/version.h"
#include "clouds/drivers/leds.h"
#include "clouds/drivers/switches.h"
#include "clouds/drivers/adc.h"
#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include <cmath>
#include "clouds/resources.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


using namespace clouds;
using namespace stmlib;

Leds leds;
Switches switches;
Adc adc;

int counter;

// Default interrupt handlers.
extern "C" {
  void NMI_Handler() { }
  void HardFault_Handler() { while (1); }
  void MemManage_Handler() { while (1); }
  void BusFault_Handler() { while (1); }
  void UsageFault_Handler() { while (1); }
  void SVC_Handler() { }
  void DebugMon_Handler() { }
  void PendSV_Handler() { }


// Setup Variables 

uint32_t delayLength = 48000; // length in samples 1 second 
int16_t delayLine[48000]; // should be the same as above 
double recordHead = 0;
double playHead1;

float leftIn;
float rightIn;

float leftOut; 
float rightOut; 

float headOne;
float headOneA;
float headWrite;

float mix;  // from Position knob
float feedback; // from Size knob 
float pitchPot1; 
float pitchScale = 5; 
float xfade = 0.5;

float pitchPotOld; 
float level1;
float level2;


// Quantise 0-1 pot position to a increments for delay speed changes 
float quantiseChromatic (float raw, int octaveRange) {
raw = raw*100.0;
float coefficient;
switch (octaveRange){

case 0:
 coefficient = lut_quantised_playback_1[(long)raw];
break;

case 1:
 coefficient = lut_quantised_playback_2[(long)raw];
break;

case 2:
 coefficient = lut_quantised_playback_3[(long)raw];
break;

case 3:
 coefficient = lut_quantised_playback_4[(long)raw];
break;

case 4:
 coefficient = lut_quantised_playback_5[(long)raw];
break;

case 5:
 coefficient = lut_quantised_playback_6[(long)raw];
break;

case 6:
 coefficient = lut_quantised_playback_7[(long)raw];
break;

case 7:
 coefficient = lut_quantised_playback_8[(long)raw];
break;


default:
 coefficient = lut_quantised_playback_9[(long)raw];
}



return coefficient; 
}



// Linear interpolation function 
int16_t interpolateLin(int16_t thisOne, int16_t nextOne, float progress){
int16_t addOn = (nextOne - thisOne) * (float)progress;
return thisOne + addOn;
}





  // called every 1ms
  void SysTick_Handler() {

    
// read Mix and Feedback pots     
mix = adc.float_value(ADC_POSITION_POTENTIOMETER_CV);
feedback = adc.float_value(ADC_SIZE_POTENTIOMETER);


// Read and quantise pitch pot 
float quantPot = adc.float_value(ADC_TEXTURE_POTENTIOMETER)*100.0;
int quantScale = quantPot/10; // divide by 10 to get 10 steps 
pitchPot1 = (quantiseChromatic(adc.float_value(ADC_PITCH_POTENTIOMETER),quantScale));
      adc.Convert();

// Update LEDs 
// LED 0 = record head 
// LED 2-4 = the playback heads 
leds.set_status(0, (float)recordHead/delayLength*255, 0);
leds.set_status(1, (float)playHead1/delayLength*255, (pitchPot1==1 || pitchPot1==-1)*32); 
leds.set_status(2, 0, 0);
leds.set_status(3, 0, 0);
    leds.Write();

  }

}



//
// called every time the codec needs one buffer of data
//
void FillBuffer(Codec::Frame* input, Codec::Frame* output, size_t n) {
  // for each sample
  while (n--) {

// read floats from inputs 
leftIn = static_cast<float>(input->l) / 32768.0;
rightIn = static_cast<float>(input->r) / 32768.0;


// Read delayLine to variables, with linear interpolation between points 
int16_t head1 = interpolateLin ( delayLine[(long)playHead1 % delayLength], delayLine[((long)playHead1+1) % delayLength], playHead1 - (long)playHead1 ); 
// 1a is a second head for crossfading opposite phase   
int16_t head1a = interpolateLin ( delayLine[((long)playHead1 + (delayLength/2)) % delayLength], delayLine[((long)playHead1+1 + (delayLength/2)) % delayLength], playHead1 - (long)playHead1 ); 


// Convert playback head signals into floats for DSP processing 
headOne = static_cast<float>(head1) / 32768.0;
headOneA = static_cast<float>(head1a) / 32768.0;

// Crossfade to avoid clicks when crossing the record head
float headOneProximity = abs( abs((long)recordHead % delayLength) -  abs((long)playHead1 % delayLength));
float headOneFade = (constrain(headOneProximity-500,0,2000)/2)/1000.0;
headOne = Crossfade(headOneA, headOne, headOneFade);




// Mix delay write head signals 
headWrite = (leftIn + (headOne * feedback))/1.1f;

 // Add playHead1 position to Left input (for feedback), and write to delay line
 // % delayLength = circular recording 
 // converting float from DSP to int for storage 
 // Clip to ensure no distortion 
 
 delayLine[(long)recordHead % delayLength] = static_cast<int16_t>(stmlib::SoftClip(headWrite) * 32768.0); 
  
// Mix output signals 
leftOut = (headOne + (leftIn * mix))/2;

// drive output
// converting float from DSP to int for output  
output->l = static_cast<int16_t>(stmlib::SoftClip(leftOut) * 32768.0);
output->r = input->l;  //******** MONO AT THE MOMENT 

  // Increment heads & buffers 
  	recordHead++;
  	playHead1 = playHead1 + pitchPot1;
    output++;
    input++;
  }
}



void Init() {
 
  // configure and initialize the internal ADC (for CV and pots) and
  // LED driver.
  leds.Init();
  adc.Init();

  // start the timer that calls SysTick_Handler (see system.h)
  System sys;
  sys.Init(true);

  // later versions of Clouds have slight hardware difference; this is
  // to differentiate them
  Version version;
  version.Init();
  bool master = !version.revised();

  // initialize the codec at 48kHz, with a buffer size of 32 samples
  Codec codec;
  codec.Init(master, 48000);

  // start the system 
  sys.StartTimers();
  codec.Start(32, &FillBuffer);

}

// this is the function that is called on startup
int main(void) {
  // initialize and configure all the devices
  Init();
  // and then do nothing (the timers will trigger the appropriate functions)
  while (1) {
  }
}
