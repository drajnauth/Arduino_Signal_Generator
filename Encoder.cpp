/*

Routines Written by Dave Rajnauth, VE3OOI to control perform decoding of a rotary encoder and push buttons 

*/

#include "Arduino.h"

#include "VE3OOI_Si5351_Signal_Generator.h"   // Defines for this program
#include "UART.h"                             // VE3OOI Serial Interface Routines (TTY Commands)
#include "VE3OOI_Si5351_v1.4.h"               // VE3OOI Si5351 Routines
#include "RTTY.h"                             // VE3OOI routines for tranmitting RTTY
#include "PSK.h"                              // VE3OOI routines for tranmitting PSK
#include "LCD.h"
#include "Encoder.h"


extern volatile unsigned long flags;


// Encoder Variables
volatile int enc_states[] = {0, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
extern volatile int old_AB;
extern volatile unsigned char encoderVal, encoderState;
extern volatile unsigned int pbstate;
extern volatile unsigned long pbreset;

// Frequency Control variables
extern volatile unsigned long frequency_clk, PSKCarrierFrequency;
extern volatile long frequency_inc;
extern volatile unsigned long frequency_mult;
extern volatile unsigned long frequency_mult_old;


// Push Button Debounce
extern volatile unsigned char pb1ctr, pbenable, pbpushed;
extern volatile unsigned int pb1rctr; 

void CheckPushButtons (void)
{

  // If pushbutton is disabled, then ignore detection and debounce
  if ( !(pbenable & PB1ENABLED) ) {
   
    // Check button state
    if (digitalRead(PBUTTON1)) {
      // Debounce button (if pushed) only if button was not previously pushed
      if ( !(pbpushed & PBUTTON1_PUSHED) ) {
        if (pb1ctr >= PBDEBOUNCE) {           // Check if debounce counter exceed debounce threshold
          pbpushed |= PBUTTON1_PUSHED;        // Set push flag
          flags |= PBUTTON1_PUSHED;           // Set push flag
          pbenable |= PB1ENABLED;             // Disable push button. This is reset after button has been processed
          pb1rctr = PBDEBOUNCE;
        } else {
          pb1ctr++;
        } 
      }
    } else {
      if (pb1rctr) pb1rctr--;                   // Decrement relaxation coutter
      else pbpushed &= ~PBUTTON1_PUSHED;        // if counter is zero, then reset push flag
    }
  } 
}


void EnablePushButtons (void)
{
  pbenable &= ~PB1ENABLED;
  flags &= ~PBUTTON1_PUSHED;
  pb1ctr = 0;
}

void ResetAllButtons (void)
{
  pbpushed = 0;
  pbenable &= ~PB1ENABLED;
  flags &= ~PBUTTON1_PUSHED;
  flags &= ~ROTARY_CW;
  flags &= ~ROTARY_PUSH;
}

unsigned char IsPushed (void) 
{
// Routine to check if a button has been pressed.  If the button is disabled ignore. 
// Buttom must be enabled in order for it to be returned
// pbenable is set by CheckPushButtons ()

  if (pbpushed & PBUTTON1_PUSHED && pbenable & PB1ENABLED) {
    return 1; 
  } 
  return 0;
}

void CheckEncoder (void)
{
// This routine is used to poll the encoder for rotation or rotary button pushed.
// Rotation with cause frequency to increase or decrease by the current increment.
// Rotary pushbutton will cycle through increment values
  unsigned long mult;
  int retvalue;
  
  
  // May overrun so don't process if encoderState has not been cleared (i.e. last update processed)
  if (encoderState) {
    return;
  }

  retvalue = 0;
  
  // Check for rotation or push
  if (flags & OFFLINE_MODE) {
    retvalue = ReadEncoder();        // Returns 0, 1 or -1
    if (retvalue < 0) {
      flags |= ROTARY_CCW;
    } else if (retvalue > 0) {
      flags |= ROTARY_CW;
    } 
    
    retvalue = ReadPBEncoder();
    if (retvalue < 0) {
      flags |= ROTARY_PUSH;
    }
    
  } else if (flags & ONLINE_MODE) {
    mult = frequency_mult;          // Current multiple
    frequency_inc = ReadEncoder();        // Returns 0, 1 or -1
    retvalue = ReadPBEncoder();

    if (retvalue) {
      frequency_mult *= 10;
      // validate bounds for frequency increment
      if (frequency_mult > MAXIMUM_FREQUENCY_MULTIPLIER) {
        frequency_mult = MINIMUM_FREQUENCY_MULTIPLIER;
      }
    }

    // Set encoderState (i.e. send signal) if button push for downstream procesing
    if (frequency_mult_old != frequency_mult) {
      encoderState |= 1;
    }
    frequency_mult_old = frequency_mult;

    // Update frequency based on frequency increment (either +1 or -1). If 0 not turned.
    if (frequency_inc) {
      frequency_clk += (frequency_inc * frequency_mult);

      if (frequency_clk < LOW_FREQUENCY_LIMIT) frequency_clk = LOW_FREQUENCY_LIMIT;
      if (frequency_clk > HIGH_FREQUENCY_LIMIT) frequency_clk = HIGH_FREQUENCY_LIMIT;
    
      encoderVal = 0xFF;        // Reset for next ReadEncoder()
      encoderState |= 2;        // Set encoderState (i.e. send signal) if rotation detected for downstream procesing
    }  
  
  }


 
}


int ReadEncoder(void)
{
// Routine to Read Encoder Rotation - used increment/decrement frequency
// Based on routine on hihiduino.
// see https://hifiduino.wordpress.com/2010/10/20/rotaryencoder-hw-sw-no-debounce/
// Returns 0 for no rotation, 1 for CW rotation and -1 for CCW rotation

  int state;
  old_AB <<= 2;                             //remember previous state
  old_AB |= (( ENC_PORT & 0x0C ) >> 2);     //add current state
  state = enc_states[( old_AB & 0x0f )];
  if (state > 0) encoderVal = CCW;
  else if (state < 0) encoderVal = CW;
  else encoderVal = CW;
  return ( state );
}

//////////////////////////////////
unsigned long ReadPBEncoder(void)
{
// Routine to Read Encoder Push Button - used to select frequency increment multiplier each push increase by x10
// The button reads as 0 when pushed (pin gounded) and 1 when not pushed (weak pullup enabled on this pin)
// The button is software debounced for release.  There is hardware debounce for when its pushed (i.e. when pin grounded)
// Frequency increment increase by 10 by each push of the rotary encoder.

  unsigned char button;

  button = (( ENC_PBPORT & 0x10 ) >> 4);     //Read PIN and shift so its 1 or 0

  // Software debounce button
  // In default state button state is 1, when pushed its grounded so its 0
  // pbstate is used to allow button to be release for a period of time - i.e. allow a relaxation period to expire
  // pbreset is used to reset frequency for a long rotaty button push
  if (!button && !pbstate) {                            // Button pushed and relax condition met
    pbstate = PUSH_BUTTON_RELAXATION;                  // Reset counter for relaxation period
    flags |= ROTARY_PUSH;
    return 1;

  // Relaxation period
  } else if (button) {
    if (pbstate) pbstate--;
    pbreset = 0;                    
  }

  // If button pushed for a long time, reset frequencies back to default
  if (!button) {            // Button continually pushed 
    if (pbreset++ > PUSH_BUTTON_RESET) {
      flags |= MASTER_RESET;
      pbreset = 0;
    }
  }
  
  return ( 0 );    // return button not pushed
}





