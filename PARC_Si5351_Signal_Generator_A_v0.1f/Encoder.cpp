/*
  Routines Written by Dave Rajnauth, VE3OOI to control perform decoding of a rotary encoder and push buttons 

  Software is licensed (Non-Exclusive Licence) for use by the Peel Amateur Radion Club.  

  All other uses licensed under a Creative Commons Attribution 4.0 International License.
*/

#include "Arduino.h"

#include "VE3OOI_Si5351_Signal_Generator.h"   // Defines for this program
#include "UART.h"                             // VE3OOI Serial Interface Routines (TTY Commands)
#include "LCD.h"
#include "Encoder.h"
#include "Timer.h"

extern volatile unsigned long flags;


// Encoder Variables
volatile int enc_states[] = {0, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile int old_AB;
volatile unsigned int pbstate;
volatile unsigned long pbreset;

// Push Button Debounce
volatile int pb1state, pb2state;             // the current reading from the input pin
volatile int oldpb1state, oldpb2state;       // the current reading from the input pin
volatile unsigned long pb1time, pb2time;  // the last time the output pin was toggle

int pb1current;
int pb2current;

unsigned long mscurrent;


void SetupEncoder (void)
{
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup Encoder Pins
  pinMode(ENC_A, INPUT);            // Rotary A: set at input
  pinMode(ENC_B, INPUT);            // Rotary A: set at input
  pinMode(ENC_PB, INPUT);           // Rotary Push button

  // Push Buttons
  pinMode(PBUTTON1, INPUT);         // Push buttons are input
  pinMode(PBUTTON2, INPUT);         // Push buttons are input

  // Push Button Debounce
  pb1state = pb2state = HIGH;             // the current state of the push button
  oldpb1state = oldpb2state = HIGH;       // the last reading from the push button
  pb1time = pb2time = 0;                  // the last time the push button state changed

  flags &= ~PBUTTON1_PUSHED;
  flags &= ~PBUTTON2_PUSHED;
  flags &= ~ROTARY_CW;
  flags &= ~ROTARY_CCW;
  flags &= ~ROTARY_PUSH;

  EnableTimers (1, TIMER500);         // Timer 1 is for Rotary & Pushbutton,. was 1ms

  ResetEncoder ();
  
}

void ResetEncoder (void) 
{
  // Push Button Debounce
  pb1state = pb2state = HIGH;             // the current state of the push button
  oldpb1state = oldpb2state = HIGH;       // the last reading from the push button
  pb1time = pb2time = 0;                  // the last time the push button state changed

  pbreset = 0;
  pbstate = PBDEBOUNCE*2;
  
}



void CheckPushButtons (void)
{
  pb1current = digitalRead(PBUTTON1);
  pb2current = digitalRead(PBUTTON2);

  mscurrent = millis();

  if (pb1current != oldpb1state) {
    pb1time = mscurrent;
  }
  if (pb2current != oldpb2state) {
    pb2time = mscurrent;
   }

  if ((mscurrent - pb1time) > PBDEBOUNCE) {
    if (pb1current != pb1state) {
      pb1state = pb1current;
      if (pb1state == PBUTTON_STATE && !(flags & PBUTTON1_PUSHED) ) {
        digitalWrite(LED_BUILTIN, HIGH);
        flags |= PBUTTON1_PUSHED;
      }
    }
  }
  oldpb1state = pb1current;

  if ((mscurrent - pb2time) > PBDEBOUNCE) {
    if (pb2current != pb2state) {
      pb2state = pb2current;
      if (pb2state == PBUTTON_STATE && !(flags & PBUTTON2_PUSHED) ) {
        digitalWrite(LED_BUILTIN, HIGH);
        flags |= PBUTTON2_PUSHED;
      }
    }
  }
  oldpb2state = pb2current;
}

void CheckEncoder (void)
{
// This routine is used to poll the encoder for rotation or rotary button pushed.
// Rotation with cause frequency to increase or decrease by the current increment.
// Rotary pushbutton will cycle through increment values
  unsigned long mult;
  int retvalue;
  
  // May overrun so don't process if flags has not been cleared (i.e. last update processed)
  if ( (flags & ROTARY_CCW) || (flags & ROTARY_CW)) {
    return;
  }

  retvalue = 0;
  
  // Check for rotation or push
  retvalue = ReadEncoder();        // Returns 0, 1 or -1
  if (retvalue < 0) {
    flags |= ROTARY_CCW;
    digitalWrite(LED_BUILTIN, HIGH);  
    
  } else if (retvalue > 0) {
    flags |= ROTARY_CW;
    digitalWrite(LED_BUILTIN, HIGH);

    
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
  return ( state );
}

//////////////////////////////////
void ReadPBEncoder(void)
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
    pbstate = PBDEBOUNCE;

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
  
}
