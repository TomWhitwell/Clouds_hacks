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
double playHead2;
double playHead3; 

float leftIn;
float rightIn;

float leftOut; 
float rightOut; 

float headOne;
float headTwo; 
float headWrite;

float mix;
float feedback;
float pitchPot1; 
float pitchPot2; 
float pitchPot3; 
float pitchScale = 5; 
float xfade = 0.5;

float pitchPotOld; 
float level1;
float level2;



  // called every 1ms
  void SysTick_Handler() {
    adc.Convert();
    
// read Mix and Feedback pots     
mix = adc.float_value(ADC_POSITION_POTENTIOMETER_CV);
feedback = adc.float_value(ADC_SIZE_POTENTIOMETER);

// Read 3 x pitch pots, and scale to -1.5 to +1.5 
pitchPot1 = (((adc.float_value(ADC_PITCH_POTENTIOMETER)+(1.0-adc.float_value(ADC_SIZE_CV)))/2)*pitchScale)-(pitchScale/2.0);
pitchPot2 = (adc.float_value(ADC_BLEND_POTENTIOMETER)*pitchScale)-(pitchScale/2.0);
pitchPot3 = adc.float_value(ADC_TEXTURE_POTENTIOMETER);

// Lets try quantising these values 
int divisor = 6; // relationship between divisor and pitchScale determines melodic steps 
pitchPot1 = (int)(pitchPot1 * (float)divisor) / (float)divisor; 
pitchPot2 = (int)(pitchPot2 * (float)divisor) / (float)divisor; 

// Update LEDs 
// LED 0 = record head 
// LED 2-4 = the playback heads 
leds.set_status(0, (float)recordHead/delayLength*255, 0);
leds.set_status(1, (float)playHead1/delayLength*255, (pitchPot1==1 || pitchPot1==-1)*32); 
leds.set_status(2, (float)playHead2/delayLength*255, (pitchPot2==1 || pitchPot2==-1)*32);
leds.set_status(3, (float)playHead3/delayLength*255, (pitchPot3==1 || pitchPot3==-1)*32);
    leds.Write();

  }
}

// Linear interpolation function 
int16_t interpolateLin(int16_t thisOne, int16_t nextOne, float progress){
int16_t addOn = (nextOne - thisOne) * (float)progress;
return thisOne + addOn;
}

//
// called every time the codec needs one buffer of data
//
void FillBuffer(Codec::Frame* input, Codec::Frame* output, size_t n) {
  // for each sample
  while (n--) {

leftIn = static_cast<float>(input->l) / 32768.0;
rightIn = static_cast<float>(input->r) / 32768.0;


// Read delayLines to variables, with linear interpolation between points 
int16_t head1 = interpolateLin ( delayLine[(long)playHead1 % delayLength], delayLine[((long)playHead1+1) % delayLength], playHead1 - (long)playHead1 ); 
int16_t head2 = interpolateLin ( delayLine[(long)playHead2 % delayLength], delayLine[((long)playHead2+1) % delayLength], playHead2 - (long)playHead2 ); 
// int16_t head3 = interpolateLin ( delayLine[(long)playHead3 % delayLength], delayLine[((long)playHead3+1) % delayLength], playHead3 - (long)playHead3 ); 

// Convert playback head signals into floats for DSP processing 
headOne = static_cast<float>(head1) / 32768.0;
headTwo = static_cast<float>(head2) / 32768.0;




// Setup mix for delay write head  

headWrite = stmlib::SoftClip(leftIn + (headOne * feedback));


 // Add playHead1 position to Left input (for feedback), and write to delay line
 // % delayLength = circular recording 
 // converting float from DSP to int for storage 
 
 delayLine[(long)recordHead % delayLength] = static_cast<int16_t>(stmlib::SoftClip(headWrite) * 32768.0); 
 
 
//   delayLine[recordHead % delayLength] = ((float)input->l + ((delayLine[(long)playHead1 % delayLength]*feedback))); 



// Read delayLines to output; 1 to left, 2 to right and 3 to both, adding clean signal if required 
// NB converting to (long) removes the fractional position, so if the delay line is a 32456.332 it will select sample 32456 
// ie No Interpolation
//   output->l = (delayLine[(long)playHead1 % delayLength] + delayLine[(long)playHead3 % delayLength]/2 + (float)input->l*mix) /4;
//   output->r = (delayLine[(long)playHead2 % delayLength] + delayLine[(long)playHead3 % delayLength]/2 + (float)input->l*mix) /4;

 
// Mix output signals 
// leftOut = (headOne + (leftIn * mix))/2;
// rightOut = (headTwo + (leftIn * mix))/2;



leftOut = stmlib::Crossfade(headOne, headTwo, pitchPot3);

// Final clipping before output 
leftOut = stmlib::SoftClip(leftOut);
rightOut = stmlib::SoftClip(rightOut);



// drive output
// converting float from DSP to int for output  
output->l = static_cast<int16_t>(leftOut * 32768.0);
output->r = static_cast<int16_t>(rightOut * 32768.0);


  
  
  
  
  
  // Increment record head by one 
  recordHead++;
  
  // Increment 
  playHead1 = playHead1 + pitchPot1;
  playHead2 = playHead2 + pitchPot2;
  playHead3 = playHead3 + pitchPot3; 


    
    // advance the buffers by one position
    output++;
    input++;
  }
}



void Init() {

  // start the timer that calls SysTick_Handler (see system.h)
  System sys;
  sys.Init(true);
  sys.StartTimers();

  // later versions of Clouds have slight hardware difference; this is
  // to differentiate them
  Version version;
  version.Init();
  bool master = !version.revised();

  // initialize the codec at 96kHz, with a buffer size of 32 samples
  Codec codec;
  codec.Init(master,48000);
  codec.Start(32, &FillBuffer);

  // configure and initialize the internal ADC (for CV and pots) and
  // LED driver.
  leds.Init();
  adc.Init();
}

// this is the function that is called on startup
int main(void) {
  // initialize and configure all the devices
  Init();
  // and then do nothing (the timers will trigger the appropriate functions)
  while (1) {
  }
}
