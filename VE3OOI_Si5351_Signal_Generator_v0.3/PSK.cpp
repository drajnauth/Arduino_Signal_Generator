/*

Program Written by Dave Rajnauth, VE3OOI to control the Si5351. 
It can output different frequencies from 8Khz to 160Mhz on Channel 0 and 2 on Si5351
It also provides functions to calibrate arduino and store the calibration value in EEPROM
*/
//#include "Arduino.h"

#include <stdint.h>            // Needed or Unix style definations
#include <stdio.h>             // Needed for I/O functions
#include <avr/io.h>            // Needed PIN I/O
#include <avr/interrupt.h>     // Needed for timer and adc interrupt
#include <avr/eeprom.h>        // Needed for storing calibration to Arduino EEPROM

#include <Wire.h>              // Needed to communitate I2C to Si5351
#include <SPI.h>               // Needed to communitate I2C to Si5351

#include "VE3OOI_Si5351_Signal_Generator.h"   // Defines for this program
#include "UART.h"                             // VE3OOI Serial Interface Routines (TTY Commands)
#include "VE3OOI_Si5351_v1.4.h"               // VE3OOI Si5351 Routines
#include "RTTY.h"                             // VE3OOI routines for tranmitting RTTY
#include "PSK.h"                              // VE3OOI routines for tranmitting PSK

// Frequency Control variables
extern unsigned long PSKCarrierFrequency;

// Local PSK Variables
extern boolean pskChanged, pskLocked;
extern unsigned char pskResetCtr;
extern boolean decodePhaseChange;


// PSK Transmitter Variables
extern unsigned char pskSwap, pskVcodeLen;
extern unsigned int pskVcode;

/*
PSK31 runs at 31.25 baud or 32 ms per symbol
For 0 bits there is a phase reversal of 180 degrees.  For a 1 bit there is no phase reversal
Varicode is used to represent characters. The code for each character is such that there are no multiple 0 bits in a row. 
If there were multiple 0 bits in a row,then it would represend an idle conditon

Whenever a character is sent, its terminsated with at least two 0 bits (i.e. two phase reversals in a row). 
Therefore the inter-character gap is marked as two phase reversals.
*/

#define VARICODE_TABLE_SIZE 128
// Varicode lookup table below is reversed to accomodate shifting LSB (i.e. LSB and MSB reversed)
const unsigned int varicode[VARICODE_TABLE_SIZE] = {
  0x0355,  // 0 NUL
  0x036d,  // 1 SOH
  0x02dd,  // 2 STX
  0x03bb,  // 3 ETX
  0x035d,  // 4 EOT
  0x03eb,  // 5 ENQ
  0x03dd,  // 6 ACK
  0x02fd,  // 7 BEL
  0x03fd,  // 8 BS
  0x00f7,  // 9 HT
  0x0017,  // 10 LF
  0x03db,  // 11 VT
  0x02ed,  // 12 FF
  0x001f,  // 13 CR
  0x02bb,  // 14 SO
  0x0357,  // 15 SI
  0x03bd,  // 16 DLE
  0x02bd,  // 17 DC1
  0x02d7,  // 18 DC2
  0x03d7,  // 19 DC3
  0x036b,  // 20 DC4
  0x035b,  // 21 NAK
  0x02db,  // 22 SYN
  0x03ab,  // 23 ETB
  0x037b,  // 24 CAN
  0x02fb,  // 25 EM
  0x03b7,  // 26 SUB
  0x02ab,  // 27 ESC
  0x02eb,  // 28 FS
  0x0377,  // 29 GS
  0x037d,  // 30 RS
  0x03fb,  // 31 US
  0x0001,  // 32 SP
  0x01ff,  // 33 !
  0x01f5,  // 34 @
  0x015f,  // 35 #
  0x01b7,  // 36 $
  0x02ad,  // 37 %
  0x0375,  // 38 &
  0x01fd,  // 39 '
  0x00df,  // 40 (
  0x00ef,  // 41 )
  0x01ed,  // 42 *
  0x01f7,  // 43 +
  0x0057,  // 44 ,
  0x002b,  // 45 -
  0x0075,  // 46 .
  0x01eb,  // 47 /
  0x00ed,  // 48 0
  0x00bd,  // 49 1
  0x00b7,  // 50 2
  0x00ff,  // 51 3
  0x01dd,  // 52 4
  0x01b5,  // 53 5
  0x01ad,  // 54 6
  0x016b,  // 55 7
  0x01ab,  // 56 8
  0x01db,  // 57 9
  0x00af,  // 58 :
  0x017b,  // 59 ;
  0x016f,  // 60 <
  0x0055,  // 61 =
  0x01d7,  // 62 >
  0x03d5,  // 63 ?
  0x02f5,  // 64 @
  0x005f,  // 65 A
  0x00d7,  // 66 B
  0x00b5,  // 67 C
  0x00ad,  // 68 D
  0x0077,  // 69 E
  0x00db,  // 70 F
  0x00bf,  // 71 G
  0x0155,  // 72 H
  0x007f,  // 73 I
  0x017f,  // 74 J
  0x017d,  // 75 K
  0x00eb,  // 76 L
  0x00dd,  // 77 M
  0x00bb,  // 78 N
  0x00d5,  // 79 O
  0x00ab,  // 80 P
  0x0177,  // 81 Q
  0x00f5,  // 82 R
  0x007b,  // 83 S
  0x005b,  // 84 T
  0x01d5,  // 85 U
  0x015b,  // 86 V
  0x0175,  // 87 W
  0x015d,  // 88 X
  0x01bd,  // 89 Y
  0x02d5,  // 90 Z
  0x01df,  // 91 [
  0x01ef,  // 92 
  0x01bf,  // 93 ]
  0x03f5,  // 94 ^
  0x016d,  // 95 _
  0x03ed,  // 96 `
  0x000d,  // 97 a
  0x007d,  // 98 b
  0x003d,  // 99 c
  0x002d,  // 100 d
  0x0003,  // 101 e
  0x002f,  // 102 f
  0x006d,  // 103 g
  0x0035,  // 104 h
  0x000b,  // 105 i
  0x01af,  // 106 j
  0x00fd,  // 107 k
  0x001b,  // 108 l
  0x0037,  // 109 m
  0x000f,  // 110 n
  0x0007,  // 111 o
  0x003f,  // 112 p
  0x01fb,  // 113 q
  0x0015,  // 114 r
  0x001d,  // 115 s
  0x0005,  // 116 t
  0x003b,  // 117 u
  0x006f,  // 118 v
  0x006b,  // 119 w
  0x00fb,  // 120 x
  0x005d,  // 121 y
  0x0157,  // 122 z
  0x03b5,  // 123 {
  0x01bb,  // 124 |
  0x02b5,  // 125 }
  0x03ad,  // 126 ~
  0x02b7   // 127 (del)
};

extern char message1[27];
extern char message2[25];
extern char message3[22];
extern char message4[22];
extern char message0A[29];
extern char message0N[27];

unsigned char decodeLastPhase;
int decodePhaseCtr;
unsigned char pskState;



void SendPSKTestMsg (unsigned long frequency, unsigned char msgid)
{
  unsigned char i;
  
  switch (msgid) {
    case 1:
      sendPSKMessage ((char *)message0A, frequency);
      sendPSKMessage ((char *)message0N, frequency);
      sendPSKMessage ((char *)message0A, frequency);
      sendPSKMessage ((char *)message0N, frequency);
      break;
          
    case 2:
      sendPSKMessage ((char *)message1, frequency);
      sendPSKMessage ((char *)message1, frequency);
      sendPSKMessage ((char *)message1, frequency);      
      break;
      
    case 0:
    default:
      PSKCarrierFrequency = frequency + TX_FREQUENCY_OFFSET;
      SetFrequency (SI_CLK2, SI_PLL_B, PSKCarrierFrequency, SI_CLK_8MA);
      delay (5000);
      for (i=0; i<150; i++) {
        if (!pskSwap) pskSwap = 1;
        else pskSwap = 0;
        InvertClk (SI_CLK2, pskSwap);  
        delay (PSK_BAUD_DELAY);
        if (!pskSwap) pskSwap = 1;
        else pskSwap = 0;
        InvertClk (SI_CLK2, pskSwap);
        delay (PSK_BAUD_DELAY);
      }
  }

}
      

void sendPSKMessage (char *msg, unsigned long frequency)
{  
  unsigned char i;
  unsigned int vcode;

// Note: Normally turn on carrier here but SendPSKidle() turns on carrier
  PSKCarrierFrequency = frequency + TX_FREQUENCY_OFFSET;
  
  i = 0;
  while (i<strlen(msg)) {
    if (i == 0) {
      sendPSKidle(); 
    }
    if ( !(msg[i] & 0x80) ) {                     // Ensure the message is below 128 (i.e. varicode applies to ASCII code 0 to 128)
      vcode = LookupVaricode(msg[i]);
      sendPSKchar (vcode);
    }
    i++;
  }
  DisableSi5351Clocks();

}

void sendPSKidle ( void )
{
  int i;
  
  SetFrequency (SI_CLK2, SI_PLL_B, PSKCarrierFrequency, SI_CLK_8MA);
  delay(PSK_BAUD_DELAY);

  for (i=1; i<=PSK_IDLE_COUNT; i++) {
    InvertClk (SI_CLK2, pskSwap);
    if (!pskSwap) pskSwap = 1;
    else pskSwap = 0;
    delay(PSK_BAUD_DELAY);
  }

  //Sync receiver with transmitter
  sendPSKchar (0x0355);   // Send a Null Character
  sendPSKchar (0x0355);   // Send a Null Character
  sendPSKchar (0x0355);   // Send a Null Character

}

void sendPSKchar ( unsigned int vcode )
{
  char i;
  unsigned char nbits;

  nbits = numbits (vcode);
  
  for (i=0; i<nbits; i++) {
    if ( !(vcode & (1 << i)) ) {
      InvertClk (SI_CLK2, pskSwap);
      if (!pskSwap) pskSwap = 1;
      else pskSwap = 0;
    }
    delay(PSK_BAUD_DELAY);
  }
    
  for (i=0; i<PSK_CHAR_GAP_COUNT; i++) {
    InvertClk (SI_CLK2, pskSwap);
    if (!pskSwap) pskSwap = 1;
    else pskSwap = 0;
    delay(PSK_BAUD_DELAY);
  }

}



unsigned int LookupVaricode (char code)
{
  return (varicode[(unsigned char)code]);
}

unsigned int ConvertVaricode (unsigned int code)
{
  unsigned int i;

  for (i=0; i<VARICODE_TABLE_SIZE; i++) {
    if (varicode[i] == code) {
      return i;
    }
  }

  return 0xFF;
}

unsigned char numbits (unsigned int in)
{
  unsigned char i;
  unsigned int temp;
  temp = in;
  
  for (i=0; temp; i++) {
    temp >>= 1;
  }
  
  return i;
  
}



