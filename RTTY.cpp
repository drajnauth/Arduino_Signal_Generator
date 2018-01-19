/*

Program Written by Dave Rajnauth, VE3OOI to control the Si5351. 
It can output different frequencies from 8Khz to 160Mhz on Channel 0 and 2 on Si5351
It also provides functions to calibrate arduino and store the calibration value in EEPROM
*/
#include "Arduino.h"

#include <stdint.h>
#include <avr/eeprom.h>        // Needed for storeing calibration to Arduino EEPROM
#include <Wire.h>              // Needed to communitate I2C to Si5351
#include <SPI.h>               // Needed to communitate I2C to Si5351

#include "VE3OOI_Si5351_Signal_Generator.h"   // Defines for this program
#include "UART.h"                             // VE3OOI Serial Interface Routines (TTY Commands)
#include "VE3OOI_Si5351_v1.4.h"               // VE3OOI Si5351 Routines
#include "RTTY.h"                             // VE3OOI routines for tranmitting RTTY

// Global RTTY Transmitter Variables
extern volatile unsigned long rttyTransmitSpaceFreq;
extern volatile unsigned long rttyTransmitMarkFreq;

extern volatile unsigned char rttyPriorState, rttyFigures;
extern volatile char rttyLTRSSwitch;

extern char message1[27];
extern char message2[25];
extern char message3[22];
extern char message4[22];
extern char message0A[29];
extern char message0N[27];

const char figures[BAUDOT_TABLE_SIZE] = {0x0,'3',0xA,'-',' ',0x7,'8','7',0xD,'$','4',0x27,',','!',':','(','5',
                  '\"',')','2','#','6','0','1','9','?','&',0x0,'.','/',';',0x0,0x0};
                      
const char letters[BAUDOT_TABLE_SIZE] = {0x0,'E',0xA,'A',' ','S','I','U',0xD,'D','R','J','N','F','C','K','T','Z','L',
                  'W','H','Y','P','Q','O','B','G',0x0,'M','X','V',0x0,0x0};



void SendRTTYTestMsg (unsigned long frequency, unsigned char msgid)
{
  unsigned char i;
  
  switch (msgid) {
    case 1:
      sendRTTYMessage ((char *)message0A, frequency);
      sendRTTYMessage ((char *)message0N, frequency);
      sendRTTYMessage ((char *)message0A, frequency);
      sendRTTYMessage ((char *)message0N, frequency);
      break;
          
    case 2:
      sendRTTYMessage ((char *)message1, frequency);
      sendRTTYMessage ((char *)message1, frequency);
      sendRTTYMessage ((char *)message1, frequency);      
      break;
         
    case 0:
    default:
// For LSB    
//      rttyTransmitSpaceFreq = frequency + RTTY_SHIFT_FREQUENCY + TX_FREQUENCY_OFFSET;
//      rttyTransmitMarkFreq = frequency + TX_FREQUENCY_OFFSET;

      rttyTransmitSpaceFreq = frequency - RTTY_SHIFT_FREQUENCY + TX_FREQUENCY_OFFSET;   // USB
      rttyTransmitMarkFreq = frequency + TX_FREQUENCY_OFFSET;
      rttyPriorState = 0xF;
      sendRTTYbit (1);
      delay (5000);
      for (i=0; i<150; i++) {
        sendRTTYbit (0);
        delay (RTTY_BAUD_OVERHEAD_DELAY);
        sendRTTYbit (1);
        delay (RTTY_BAUD_OVERHEAD_DELAY);
      }
  }

}
      
void sendRTTYMessage (char *msg, unsigned long frequency)
{  
  unsigned char i;
  char baudot, uppercase;

// For LSB
// SPACE is 0, MARK is 1
// mark freq = (space freq + 170Hz)
//  rttyTransmitSpaceFreq = frequency + RTTY_SHIFT_FREQUENCY + TX_FREQUENCY_OFFSET;
//  rttyTransmitMarkFreq = frequency + TX_FREQUENCY_OFFSET;

  // Define default Tx frequencies
  // There is a small shift to make the frequency appear as 1000 Hz on receiver's waterfall
  rttyTransmitSpaceFreq = frequency - RTTY_SHIFT_FREQUENCY + TX_FREQUENCY_OFFSET;                 // USB
  rttyTransmitMarkFreq = frequency + TX_FREQUENCY_OFFSET;
  
  
  
  rttyPriorState = 0xF;
  
  i = 0;

//  digitalWrite(RxMute, LOW);            // Mute receiver
  
  while (i<strlen(msg)) {
    if (i == 0) {
      sendRTTYidle(); 
      sendRTTYchar (RTTY_LETTERS);    // Send several LTRS code to sync to receiver
      sendRTTYchar (RTTY_LETTERS); 
      sendRTTYchar (RTTY_LETTERS); 
      sendRTTYchar (RTTY_LETTERS); 
      rttyFigures = RTTY_LETTERS;
    }
    
    if ( (msg[i]>64 && msg[i]<91) || (msg[i]>96 && msg[i]<123) ) {            // Alphabetic characters.  Need to make uppercase
      uppercase = toupper(msg[i]);
      if (rttyFigures != RTTY_LETTERS) {
        sendRTTYchar (RTTY_LETTERS); 
        rttyFigures = RTTY_LETTERS;
        
      }
      baudot = Baudot(uppercase, RTTY_ALPHA);
     
    } else if (msg[i]>47 && msg[i]<58) {                                // Numbers 
      uppercase = msg[i];
      if (rttyFigures != RTTY_FIGURES) {
        sendRTTYchar (RTTY_FIGURES); 
        rttyFigures = RTTY_FIGURES;
      }
      baudot = Baudot(uppercase, RTTY_DIGIT);
      
    } else {                                                    // Other characters
      uppercase = msg[i];
      if (rttyFigures != RTTY_FIGURES) {
        sendRTTYchar (RTTY_FIGURES); 
        rttyFigures = RTTY_FIGURES;
      }
      
      baudot = Baudot(uppercase, RTTY_DIGIT);
      if (!baudot) {                          // if unknown (i.e. not in lookup table), then print a space
        baudot = 0x4;                         // 0x4 is a space
        rttyFigures = RTTY_LETTERS;           // See explanation above
      }          

     
    }
    sendRTTYchar (baudot);

      // This is required because receiver have "un-shift on space"
      // This means that the receive switches to LTRS (Letters) when it receives a space
      // Ensure that this system thinks its in LTRSA mode.
      if (uppercase == ' ') {             
        sendRTTYchar (RTTY_LETTERS); 
        rttyFigures = RTTY_LETTERS;
      }     
    if (i++ >= MAX_MESSAGE_SIZE) break;
  }
  
  DisableSi5351Clocks();

}

char Baudot( char c, unsigned char alpha)
{
// Routine to return the baudot code for an ascii character
// Used to translate ASCII to Baudot.  The "alpha" variable
// is used to identify the table to be use (i.e. 0 use figures, 1 use letters)
 
  unsigned char i;

  // Search baudot table
  for (i=0; i<BAUDOT_TABLE_SIZE; i++) {
    if (alpha) {                        // Search letters table
      if (letters[i] == c) {            // ASCII code found
        return i;                       // Return offset which is baudot code
      }
    } else {                            // Search letters table
      if (figures[i] == c) {            // Same as above
        return i;
      }
    }
  }
  
  return 0;                             // nothing found so return 0 (error)
}

void sendRTTYchar ( char value )
{
  char i;

// send start bit - space
  sendRTTYstart();

  for (i = 0; i < BAUDOT_BITS; i++)  {
    if (value & (1 << i))
      sendRTTYbit(1);
    else
      sendRTTYbit(0);
  }

// Send stop bits - at least 1.5 Mark bits
  sendRTTYstop();

}

void sendRTTYbit (unsigned char b) 
{
  if (b) {
    if (b != rttyPriorState) SetFrequency (SI_CLK2, SI_PLL_B, rttyTransmitMarkFreq, SI_CLK_8MA);
  } else {
    if (b != rttyPriorState) SetFrequency (SI_CLK2, SI_PLL_B, rttyTransmitSpaceFreq, SI_CLK_8MA);
  }

  if (b != rttyPriorState) delay(RTTY_BAUD_OVERHEAD_DELAY); 
  else delay(RTTY_BAUD_DELAY);
  
  rttyPriorState = b;
}




void sendRTTYstart (void) 
{
// Assumption is that transmitter is turned on
  SetFrequency (SI_CLK2, SI_PLL_B, rttyTransmitSpaceFreq, SI_CLK_8MA);
  delay(RTTY_BAUD_DELAY);
  rttyPriorState = 0;
}

void sendRTTYstop (void) 
{
// Assumption is that transmitter is turned on
//  if (!rttyPriorState) SetFrequency (SI_CLK2, SI_PLL_B, rttyTransmitMarkFreq, SI_CLK_8MA);

//  if (!rttyPriorState) delay(RTTY_STOPBIT_OVERHEAD_DELAY); 
//  else delay(RTTY_STOPBIT_DELAY);

  SetFrequency (SI_CLK2, SI_PLL_B, rttyTransmitMarkFreq, SI_CLK_8MA);
  delay(RTTY_STOPBIT_DELAY);

  rttyPriorState = 1;

}


void sendRTTYidle (void) 
{
// Assumption is that transmitter is turned off. But is can be turned on. No harm
  SetFrequency (SI_CLK2, SI_PLL_B, rttyTransmitMarkFreq, SI_CLK_8MA);
  delay(RTTY_IDLE_DELAY); // Send MARK tone for at least 2 character periods (i.e. 1 + 5 + 2 bits or 8 bits x 22ms = 352 ms)
  rttyPriorState = 1;
}


