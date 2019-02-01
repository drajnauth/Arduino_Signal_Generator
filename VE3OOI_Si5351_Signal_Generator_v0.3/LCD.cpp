

#include "Arduino.h"

#include <stdint.h>
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>

#include "VE3OOI_Si5351_Signal_Generator.h"   // Defines for this program
#include "UART.h"                             // VE3OOI Serial Interface Routines (TTY Commands)
#include "VE3OOI_Si5351_v1.4.h"               // VE3OOI Si5351 Routines
#include "RTTY.h"                             // VE3OOI routines for tranmitting RTTY
#include "PSK.h"                              // VE3OOI routines for tranmitting PSK
#include "LCD.h"
#include "Encoder.h"
#include "Timer.h"


LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

extern char header1[14];
extern char header2[16];

// Frequency Control variables
extern volatile unsigned long frequency_clk, PSKCarrierFrequency;
extern volatile long frequency_inc;
extern volatile unsigned long frequency_mult;
extern volatile unsigned long frequency_mult_old;

// LCD Menu 
extern volatile unsigned char MenuSelection;
extern char RootMenuOptions[MAXMENU_ITEMS][MAXMENU_LEN];

void SetupLCD (void)
{
  lcd.begin(16,4);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.backlight();
  lcd.setCursor(0,0);                         //Goto at position 0 line 0
  lcd.print(header1);
  lcd.setCursor(0,1);                         //Goto at position 0 line 1
  lcd.print(header2);
}

void LCDClearScreen (void)
{
  lcd.clear();   
}


void LCDSwitchFunction (unsigned char fun)
{
  if (!fun) {
    lcd.setCursor(8,0);                         //Goto at position 8 line 0
    lcd.cursor(); 
    lcd.blink(); 
  } else {
    lcd.setCursor(0,1);                         //Goto at position 0 line 1
    lcd.noCursor(); 
    lcd.noBlink(); 
  }
}

void LCDDisplayMode (void) 
{
  lcd.setCursor(7,1);                         //Goto at position 7 line 2
  lcd.print ("         ");                    // Clear the line
  lcd.setCursor(7,1);                         //Goto at position 7 line 2
  lcd.print( RootMenuOptions[MenuSelection] );
}

void LCDDisplayStatus (unsigned char on) 
{
  lcd.setCursor(8,0);                         //Goto at position 8 line 1
  lcd.print ("        ");                     // Clear the line
  lcd.setCursor(8,0);                         //Goto at position 8 line 1
  if (on) lcd.print("ON!");
  else lcd.print("offline");
}

void LCDDisplayFrequencyIncrement (void) 
{
  lcd.setCursor(0,1);                         //Goto at position 0 line 2
  lcd.print ("       ");                      // Clear the line
  lcd.setCursor(0,1);                         //Goto at position 0 line 2
  if (frequency_mult <= 1000) {
    lcd.print( frequency_mult );
  } else {
    switch (frequency_mult) {
      case 10000:
        lcd.print( "10K" );
        break;
        
      case 100000:
        lcd.print( "100K" );
        break;

      case 1000000:
        lcd.print( "1M" );
        break;

      case 10000000:
        lcd.print( "10M" );
        break;
    
    }
  }
}

void LCDDisplayFrequency (void) 
{
  lcd.setCursor(0,0);                         //Goto at position 0 line 1
  lcd.print ("        ");                     // Clear the line
  lcd.setCursor(0,0);                         //Goto at position 0 line 1
  lcd.print(frequency_clk);
}


