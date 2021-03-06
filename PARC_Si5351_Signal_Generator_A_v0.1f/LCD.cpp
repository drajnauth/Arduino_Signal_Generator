
/*

  Program Written by Dave Rajnauth, VE3OOI to control the 4x20 LCD.
  
  Software is licensed (Non-Exclusive Licence) for use by the Peel Amateur Radion Club.  

  All other uses licensed under a Creative Commons Attribution 4.0 International License.

 */

#include "Arduino.h"

#include <stdint.h>
#include <Wire.h>  // Comes with Arduino IDE

#include "VE3OOI_Si5351_Signal_Generator.h"   // Defines for this program
#include "UART.h"                             // VE3OOI Serial Interface Routines (TTY Commands)
#include "LCD.h"
#include "Encoder.h"
#include "Timer.h"

//========================================================
// LCD Library
// Can use either LiquidCrystal or HD44780 library.
// define below selects which library to use
//========================================================
#define USE_HD44780     // if this is defined, will use the HD44780 libary
                        // if commented out, will use LiquidCrystal library

// LCD geometry
const int LCD_COLS = 20;
const int LCD_ROWS = 4;

#ifdef USE_HD44780
#include <hd44780.h>                        // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h>  // i2c expander i/o class header
hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip

#else
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
#endif  //USE_HD44780
//========================================================


extern char header1[HEADER1];
extern char header2[HEADER2];

extern char clkentry [CLKENTRYLEN];

extern char clearlcdline[LCD_CLEAR_LINE_LENGTH];
extern char clearlcderrmsg[LCD_ERROR_MSG_LENGTH];

// Frequency Control variables
extern volatile unsigned long frequency_clk, PSKCarrierFrequency;
extern volatile long frequency_inc;
extern volatile long frequency_mult;
extern volatile long offset_inc;
extern volatile int calibration_mult;

// LCD Menu 
extern volatile unsigned char MenuSelection, ClkSelection;
extern char RootMenuOptions[MAXMENU_ITEMS][MAXMENU_LEN];

extern Sig_Gen_Struct sg;

void SetupLCD (void)
{
/*
  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if(status) // non zero status means it was unsuccesful
  {
    status = -status; // convert negative status value to positive number

    // begin() failed so blink error code using the onboard LED if possible
    hd44780::fatalError(status); // does not return
  }
*/
  lcd.begin(LCD_COLS, LCD_ROWS);

  lcd.backlight();
  lcd.setCursor(0,0);                         //Goto at position 0 line 0
  lcd.noCursor(); 
  lcd.noBlink(); 
}

void LCDDisplayHeader (void) 
{
  unsigned char i;
  LCDClearScreen ();
  lcd.print(header1);
  lcd.setCursor(0,1);                         //Goto at position 0 line 1
  lcd.print(header2);
  
  lcd.setCursor(0, 2);
  for (i=0; i<19; i++) {
    lcd.print((char)0xFF);
    delay (50);
  }
  lcd.setCursor(0, 3);
  for (i=0; i<19; i++) {
    lcd.print((char)0xFF);
    delay (50);
  }
  delay (500);
}

void LCDClearScreen (void)
{
  lcd.clear();   
  lcd.noCursor(); 
  lcd.noBlink(); 
}

void LCDErrorMsg (unsigned char pos, char *str) 
{
  if ( (pos+strlen(str)) > 19) {
    lcd.setCursor(pos, 3);                         //Goto at position 0 line 1
    lcd.print(F("TOO LONG")); 
    LCDSelectLine (pos, 3, 1);  
    return;
  }
  LCDSelectLine (pos, 3, 1);  
  lcd.setCursor(pos, 3);                         //Goto at position 0 line 1
  lcd.print(str);
  LCDSelectLine (pos, 3, 1);  
}

void LCDClearErrorMsg (unsigned char pos) 
{
  lcd.setCursor(pos, 3);                         //Goto at position 0 line 1
  lcd.print(clearlcderrmsg);
  LCDSelectLine (pos, 3, 0);  
}


void LCDSelectLine (unsigned char pos, unsigned char line, unsigned char enable)
{
  if (enable) {
    lcd.setCursor(pos,line);                         //Goto at position 8 line 0
    lcd.cursor(); 
    lcd.blink(); 
  } else {
    lcd.setCursor(pos,line);                         //Goto at position 0 line 1
    lcd.noCursor(); 
    lcd.noBlink(); 
  }
}

void LCDDisplayMenuOption (unsigned char line) 
{  
  lcd.setCursor(0, 3);                         //Goto at position 7 line 2
  lcd.print( RootMenuOptions[line] );
}

void LCDDisplayClockMode (unsigned char line)
{
    lcd.setCursor(12,line);
      
    switch (sg.ClkMode[line]) {
      case VFO_CLK_MODE:
        lcd.print (F("VFO"));
        break;
        
      case LO_CLK_MODE:
        lcd.print (F("LO "));
        break;
        
      case IQ_CLK_MODE:
        lcd.print (F("IQ "));
        break;
        

       
    }  
}

void LCDDisplayClockStatus (unsigned char line)
{
    lcd.setCursor(16,line);
    if (sg.ClkStatus[line]) {
        lcd.print (F("ON "));
    } else {
        lcd.print (F("OFF"));
    }
  
}

void LCDClearClockWindow (void)
{
    LCDClearLine (0);  
    LCDClearLine (1);  
    LCDClearLine (2);  
}


void LCDDisplayNumber1D (int num, unsigned char pos, unsigned char row)
{
    lcd.setCursor(pos, row);
    memset (clkentry, 0, sizeof(clkentry));         // Terminate the string  
    sprintf (clkentry, "%d", num);
    lcd.print( clkentry );
}

void LCDDisplayNumber3D (int num, unsigned char pos, unsigned char row)
{
    lcd.setCursor(pos, row);
    memset (clkentry, 0, sizeof(clkentry));         // Terminate the string  
    sprintf (clkentry, "%+04d", num);
    lcd.print( clkentry );
}


void LCDDisplayOffsetFrequency (unsigned char line)
{
    lcd.setCursor(0,line);
    memset (clkentry, 0, sizeof(clkentry));         // Terminate the string  
    sprintf (clkentry, "Offset%u: %+010ld", (unsigned int)line, sg.ClkOffset[line]);
    lcd.print( clkentry );
}

void LCDDisplayClockFrequency (unsigned char line)
{
    lcd.setCursor(0,line);
    memset (clkentry, 0, sizeof(clkentry));         // Terminate the string  
    sprintf (clkentry, "%u:%09lu", (unsigned int)line, sg.ClkFreq[line]);
    lcd.print( clkentry );

}

void LCDDisplayIQClockFrequency (unsigned char line)
{
    lcd.setCursor(0,line);
    memset (clkentry, 0, sizeof(clkentry));         // Terminate the string
    sprintf (clkentry, "%u:%09lu", (unsigned int)line, sg.IQClkFreq[line]);
    lcd.print( clkentry );
}

void LCDDisplayLOClockFrequency (unsigned char line)
{
    long fq;
//    if (sg.ClkMode[line] == VFO_CLK_MODE) {
//      LCDDisplayClockFrequency(line);
//      return;
//    }
    fq = (long)sg.ClkFreq[line] + sg.ClkOffset[line];
    if (fq < 0) fq = 0;
    lcd.setCursor(0,line);
    memset (clkentry, 0, sizeof(clkentry));  
    sprintf (clkentry, "%u:%09lu", (unsigned int)line, fq);
    lcd.print( clkentry );
}

void LCDDisplayClockEntry (unsigned char line) 
{
    LCDClearLine(line);
    LCDDisplayClockFrequency(line);
    LCDDisplayClockMode(line);
    LCDDisplayClockStatus(line);
}

void LCDClearLine (unsigned char line) 
{
//  char clean[21];
//  memset (clean, 0x20, sizeof(clean));
//  clean[20] = 0x0;              // terminate the string

  lcd.setCursor(0,line);        // Goto POSITION
  lcd.print (clearlcdline);      // Clear the line
}
