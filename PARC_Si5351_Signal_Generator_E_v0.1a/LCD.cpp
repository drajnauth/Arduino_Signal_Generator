/*

  Program Written by Dave Rajnauth, VE3OOI to control the 4x20 LCD.
  
  Software is licensed (Non-Exclusive Licence) for use by the Peel Amateur Radion Club.  

  All other uses licensed under a Creative Commons Attribution 4.0 International License.

 */

#include "Arduino.h"

#include <stdint.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header

#include "VE3OOI_Si5351_Signal_Generator.h"   // Defines for this program
#include "UART.h"                             // VE3OOI Serial Interface Routines (TTY Commands)
#include "LCD.h"
#include "Encoder.h"
#include "Timer.h"

// LCD geometry
const int LCD_COLS = 20;
const int LCD_ROWS = 4;

hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip

extern char header1[HEADER1];
extern char header2[HEADER2];

extern char clkentry [CLKENTRYLEN];

// Frequency Control variables
extern volatile unsigned long frequency_clk, PSKCarrierFrequency;
extern volatile long frequency_inc;
extern volatile long frequency_mult;
extern volatile long offset_inc;
extern volatile int calibration_mult;

// LCD Menu 
extern volatile unsigned char MenuSelection;
extern char RootMenuOptions[MAXMENU_ITEMS][MAXMENU_LEN];

extern Sig_Gen_Struct sg;

void SetupLCD (void)
{
  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if(status) // non zero status means it was unsuccesful
  {
    status = -status; // convert negative status value to positive number

    // begin() failed so blink error code using the onboard LED if possible
    hd44780::fatalError(status); // does not return
  }
  
  lcd.backlight();
  lcd.setCursor(0,0);                         //Goto at position 0 line 0
  lcd.noCursor(); 
  lcd.noBlink(); 
}

void LCDDisplayHeader (void) 
{
  LCDClearScreen ();
  lcd.print(header1);
  lcd.setCursor(0,1);                         //Goto at position 0 line 1
  lcd.print(header2);
  delay (2000);
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
    lcd.print("TOO LONG"); 
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
  lcd.print("         ");
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
  lcd.setCursor(0,line);                         //Goto at position 7 line 2
  lcd.print( RootMenuOptions[MenuSelection] );
}

void LCDDisplayClockMode (unsigned char line)
{
    lcd.setCursor(12,line);
      
    switch (sg.ClkMode[line]) {
      case VFO_CLK_MODE:
        lcd.print ("VFO ");
        break;
        
      case LO_CLK_MODE:
        lcd.print ("LO  ");
        break;
        
      case IQ_CLK_MODE:
        lcd.print ("IQ  ");
        break;
        
      case RTTY_CLK_MODE:
        lcd.setCursor(10,line);
        lcd.print ("RTTY");
        break;
        
      case PSK_CLK_MODE:
        lcd.print ("PSK ");
        break;
        
      case CW_CLK_MODE:
        lcd.print ("CW  ");
        break;
       
    }  
}

void LCDDisplayClockStatus (unsigned char line)
{
    lcd.setCursor(17,line);
    
    if (sg.ClkStatus[line]) {
        lcd.print ("ON ");
    } else {
        lcd.print ("OFF");
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
    memset (clkentry, 0, sizeof(clkentry));  
    sprintf (clkentry, "%d", num);
    lcd.print( clkentry );
}

void LCDDisplayNumber3D (int num, unsigned char pos, unsigned char row)
{
    lcd.setCursor(pos, row);
    memset (clkentry, 0, sizeof(clkentry));  
    sprintf (clkentry, "%+04d", num);
    lcd.print( clkentry );
}


void LCDDisplayOffsetFrequency (unsigned char line)
{
    lcd.setCursor(0,line);
    memset (clkentry, 0, sizeof(clkentry));  
    sprintf (clkentry, "Offset%d: %+010ld\0", line, sg.ClkOffset[line]);
    lcd.print( clkentry );
}

void LCDDisplayClockFrequency (unsigned char line)
{
    lcd.setCursor(0,line);
    memset (clkentry, 0, sizeof(clkentry));  
    sprintf (clkentry, "%d:%09ld\0", line, sg.ClkFreq[line]);
    lcd.print( clkentry );
}

void LCDDisplayIQClockFrequency (unsigned char line)
{
    lcd.setCursor(0,line);
    memset (clkentry, 0, sizeof(clkentry));  
    sprintf (clkentry, "%d:%09ld\0", line, sg.IQClkFreq[line]);
    lcd.print( clkentry );
}

void LCDDisplayLOClockFrequency (unsigned char line)
{
    long fq;
    if (sg.ClkMode[line] == VFO_CLK_MODE) {
      LCDDisplayClockFrequency(line);
      return;
    }
    fq = (long)sg.ClkFreq[line] + sg.ClkOffset[line];
    if (fq < 0) fq = 0;
    lcd.setCursor(0,line);
    memset (clkentry, 0, sizeof(clkentry));  
    sprintf (clkentry, "%d:%09ld\0", line, fq);
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
  char clean[21];
  memset (clean, 0x20, sizeof(clean));
  clean[20] = 0;
  lcd.setCursor(0,line);        // Goto POSITION
  lcd.print (clean);            // Clear the line
}

void LCDTest (void) 
{
  LCDErrorMsg (11, "0123456789");
}



