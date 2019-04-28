/*

  Program Written by Dave Rajnauth, VE3OOI to control and service timer interupts 

  
  Software is licensed (Non-Exclusive Licence) for use by the Peel Amateur Radion Club.  

  All other uses licensed under a Creative Commons Attribution 4.0 International License.

*/

#include "Arduino.h"

#include <stdint.h>
#include <avr/eeprom.h>        // Needed for storeing calibration to Arduino EEPROM
#include <Wire.h>              // Needed to communitate I2C to Si5351
#include <SPI.h>               // Needed to communitate I2C to Si5351

#include "VE3OOI_Si5351_Signal_Generator.h"   // Defines for this program
#include "UART.h"                             // VE3OOI Serial Interface Routines (TTY Commands)
#include "LCD.h"
#include "Encoder.h"
#include "Timer.h"

extern volatile unsigned long flags;


//////////////////////////////////
// Timer1 ISR - TBD
//////////////////////////////////
ISR(TIMER1_COMPA_vect)
{
  if (!(flags & DISABLE_BUTTONS)) {
    CheckEncoder();  
    CheckPushButtons ();  
    ReadPBEncoder();
  }
}


//////////////////////////////////
// Timer2 ISR - used for encoder and pushbutton polling. It runs at 3ms
//////////////////////////////////
ISR(TIMER2_COMPA_vect)
{

}



//////////////////////////////////
//  Configure and enable a timer.
//////////////////////////////////
void EnableTimers (unsigned char timer, unsigned int count)
{
// Note timers enabled via TCCRnB register, must set a non-zero prescalar to enable
// Arduino uses Timer 0 for serial printing and for delay and other timing functions. Be carefull

  cli();          // disable global interrupts
  switch (timer) {
    case 0:
      break;

    case 1:   // Timer 1 used for input devices (rotary, push buttons, etc)
      TCCR1A = 0;     // reset Timer 5
      TCCR1B = 0;     // TCCRxB turns off timer
      TCNT1 = 0;      // Zero out counter

      // Set match register to 1795  for 113us (or 8850 Hz) with no prescalar for desired interval
      // for Prescalar 1/64, 27 for 113us, 5500 for 22 ms, 3750 for 15ms, 250 for 1ms
      // Set match register to 1000  for 64ms (or 15.6 Hz) with /1024 prescaller
      OCR1A = count;                              // set compare match register for interval
      TCCR1B |= (1 << WGM12);                     // turn on CTC mode
      TCCR1B |= (1 << CS10) | (1 << CS11);        // Set CSx0/CSx1 for /64 prescaler
//      TCCR5B |= (1 << CS10) | (1 << CS12);      // Set CSx0/CSx2 for /1024 prescaler
      TIFR1 |= (1 << ICF1) | (1 << OCF1A) | (1 << OCF1B);    // Clear Interrupt Flags (write 1)
      TIMSK1 |= (1 << OCIE1A);                    // enable timer compare interrupt:
      break;

    case 2:           // Not available on some Arduinos
      TCCR2A = 0;     // reset Timer 2
      TCCR2B = 0;     // TCCRxB turns off timer
      TCNT2 = 0;      // Zero out counter

      // Set match register to 1795  for 113us (or 8850 Hz) with no prescalar for desired interval
      // for Prescalar 1/64, 27 for 113us, 5500 for 22 ms, 3750 for 15ms, 250 for 1ms
      // Set match register to 1000  for 64ms (or 15.6 Hz) with /1024 prescaller
      OCR2A = count;                            // set compare match register for interval
      TCCR2B |= (1 << WGM22);                   // turn on CTC mode
      TCCR2B |= (1 << CS20) | (1 << CS21);      // Set CSx0/CSx1 for /64 prescaler
//      TCCR2B |= (1 << CS20) | (1 << CS22);      // Set CSx0/CSx2 for /1024 prescaler
      TIFR2 |= (1 << OCF2A) | (1 << OCF2B); // Clear Interrupt Flags (write 1)
      TIMSK2 |= (1 << OCIE2A);                  // enable timer compare interrupt:
      break;

  }
  sei();          // enable global interrupts

}

//////////////////////////////////
// Disable a timer.
//////////////////////////////////
void DisableTimers (unsigned char timer)
{
// Note timers enabled via TCCRnB register, must set a non-zero prescalar to enable
// Arduino uses Timer 0 for serial printing and for delay and other timing functions

  cli();          // disable global interrupts
  switch (timer) {
    case 0:
      break;

    case 1:
      TCCR1A = 0;     // reset Timer 1
      TCCR1B = 0;     // TCCRxB turns off timer
      TCNT1 = 0;      // Zero out counter
      TIFR1 |= (1 << ICF1) | (1 << OCF1A) | (1 << OCF1B);    // Clear Interrupt Flags (write 1)
      break;

    case 2:
      TCCR2A = 0;     // reset Timer3
      TCCR2B = 0;     // TCCRxB turns off timer
      TCNT2 = 0;      // Zero out counter
      TIMSK2 = 0;     // Disable timer compare interrupts
      TIFR2 |= (1 << OCF2A) | (1 << OCF2B);    // Reset Interrupt Flags
      break;

  }
  sei();          // enable global interrupts

}
