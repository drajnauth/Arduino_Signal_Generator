/*

 Program Written by Dave Rajnauth, VE3OOI to control the Si5351.
 
 Software is licensed (Non-Exclusive Licence) for use by the Peel Amateur Radion Club. 

 All other uses licensed under a Creative Commons Attribution 4.0 International License.

 */
 
#include "Arduino.h"

#include <stdint.h>
#include <avr/eeprom.h>    // Needed for storeing calibration to Arduino EEPROM
#include <Wire.h>       // Needed to communitate I2C to Si5351
#include <SPI.h>        // Needed to communitate I2C to Si5351
#include <EEPROM.h>

#include "VE3OOI_Si5351_Signal_Generator.h"  // Defines for this program
#include "LCD.h"
#include "Encoder.h"
#include "Timer.h"
#include "VE3OOI_Si5351_v2.1.h"


#ifndef REMOVE_CLI
#include "UART.h"               // VE3OOI Serial Interface Routines (TTY Commands)

// These variables are defined in UART.cpp and used for Serial interface
// rbuff is used to store all keystrokes which is parsed by Execute()
// commands[] and numbers[] store all characters or numbers entered provided they
// are separated by spaces.
// ctr is counter used to process entries
// command_entries contains the total number of charaters/numbers entered
char rbuff[RBUFF];
char commands[MAX_COMMAND_ENTRIES];
unsigned char command_entries;
unsigned long numbers[MAX_COMMAND_ENTRIES];
unsigned char ctr;
#endif // REMOVE_CLI


volatile unsigned long mflags;
volatile unsigned long bflags;

// Frequency Control variables
volatile unsigned long frequency_clk;
volatile long offset_frequency;
volatile unsigned long frequency_inc;
volatile long offset_inc;
volatile int calibration_mult;

// Calibration and memory slot variable
volatile int rotaryNumber;
volatile int rotaryInc;

// LCD Menu
volatile unsigned char MenuSelection, ClkSelection;

char RootMenuOptions[MAXMENU_ITEMS][MAXMENU_LEN] = {
 {"VFO ENABLE"},
 {"MIX ENABLE"},
 {"I/Q ENABLE"},
 {"SET OFFSET"},
 {"CALIBRATE "},
 {"SAVE      "},
 {"RECALL    "},
 {"CLI ENABLE"},
 {"RESET     "}
};

char header1[HEADER1] = {'P', 'A', 'R', 'C', ' ', 'S', 'I', 'G', ' ', 'G', 'E', 'N', ' ', 'A', '0', '.', '1', 'F', ' ', 0x0};
char header2[HEADER2] = {' ', '(', 'C', ')', 'V', 'E', '3', 'O', 'O', 'I', 0x0};

char clkentry [CLKENTRYLEN];

char prompt[6] = {0xa, 0xd, ':', '>', ' ', 0x0};
char ovflmsg[9] = {'O', 'v', 'e', 'r', 'f', 'l', 'o', 'w', 0x0};
char errmsg[4] = {'E', 'r', 'r', 0x0};

char clearlcdline[LCD_CLEAR_LINE_LENGTH];
char clearlcderrmsg[LCD_ERROR_MSG_LENGTH];

char okmsg[LCD_ERROR_MSG_LENGTH];

// Specific Sig Gen parameters
Sig_Gen_Struct sg;
Sig_Gen_Struct mem[MAX_MEMORIES];

unsigned int bctr;
unsigned char rotaryChangeFlag;

// the setup function runs once when you press reset or power the board
void setup() {
 // define the baud rate for TTY communications. Note CR and LF must be sent by terminal program
 Serial.begin(9600);

 SetupLCD ();
 
#ifndef REMOVE_CLI
 ResetSerial ();
#endif // REMOVE_CLI

 Reset ();

 if (sg.correction > 1000 || sg.correction < -1000) {
  sg.correction = 0;
 }
 
 setupSi5351(sg.correction);

 SetupEncoder();

 MenuSelection = 0;
 ClkSelection = 0;
 bctr = 0;
}





// the loop function runs over and over again forever
void loop()
{


#ifndef REMOVE_CLI
 // Look for characters entered from the keyboard and process them
 // This function is part of the UART package.
 if (checkFlag(MFLAGS, CLI_MODE)) {
  ProcessSerial ();
 }
#endif // REMOVE_CLI

  while (!checkFlag(BFLAGS, ANY_BUTTON));

  if (checkFlag (BFLAGS, MASTER_RESET) ) {
    Reset();
  }

  if (!checkFlag(MFLAGS, CLI_MODE) ) {
    if (checkFlag(MFLAGS, MENU_MODE)) {
      MenuDisplayMode();

    } else if (checkFlag(MFLAGS, CLOCK_WINDOW_MODE)) {
      MenuClockWindowMode();

    } else if (checkFlag(MFLAGS, CLOCK_FREQUENCY_MODE)) {
      MenuClockFrequencyMode();

    } else if (checkFlag(MFLAGS, IQ_FREQUENCY_MODE)) {
      MenuIQClockFrequencyMode();

    } else if (checkFlag(MFLAGS, OFFSET_FREQUENCY_MODE)) {
      MenuClockFrequencyOffsetMode();
  
    } else if (checkFlag(MFLAGS, CALIBRATION_MODE)) {
      GetRotaryNumber (-500, 500, 100, 11, 3);

    } else if (checkFlag(MFLAGS, MEMORY_SAVE_MODE)) {
      GetRotaryNumber (0, (int)(MAX_MEMORIES-1), 1, 11, 3);

    } else if (checkFlag(MFLAGS, MEMORY_RECALL_MODE)) {
      GetRotaryNumber (0, (int)(MAX_MEMORIES-1), 1, 11, 3);
    }
  
  } else {
    clearFlag(BFLAGS, ROTARY_CW);
    clearFlag(BFLAGS, ROTARY_CCW);
    clearFlag(BFLAGS, PBUTTON1_PUSHED);
    clearFlag(BFLAGS, PBUTTON2_PUSHED);
    digitalWrite(LED_BUILTIN, LOW);
  }


}


void GetRotaryNumber (int lnum, int hnum, int maxinc, unsigned char x, unsigned char y)
{
  unsigned char pos;
  rotaryChangeFlag = 0;
  pos = FrequencyDigitUpdate(rotaryInc) + ROTARY_NUMBER_OFFSET;

  if (checkClearFlag(BFLAGS, ROTARY_CW)) {
    rotaryChangeFlag = 1;
    rotaryNumber += rotaryInc;
    if (rotaryNumber > hnum) rotaryNumber = hnum; 

  } else if (checkClearFlag(BFLAGS, ROTARY_CCW)) {
    rotaryChangeFlag = 1;
    rotaryNumber -= rotaryInc;
    if (rotaryNumber < lnum) rotaryNumber = lnum;   
  }
 

  if ( rotaryChangeFlag ) {
    if (checkFlag(MFLAGS, MEMORY_RECALL_MODE) || checkFlag(MFLAGS, MEMORY_SAVE_MODE)) {
      LCDDisplayNumber1D (rotaryNumber, x, y);
      LCDSelectLine (x, y, 1);
 
    } else if (checkFlag(MFLAGS, CALIBRATION_MODE)) {
      LCDDisplayNumber3D (rotaryNumber, x, y);
      LCDSelectLine (pos, y, 1);
      setupSi5351 (rotaryNumber);
      UpdateFrequency (0);
      UpdateFrequency (1);
      UpdateFrequency (2);
    }
    digitalWrite(LED_BUILTIN, LOW);

  }

  if (checkClearFlag(BFLAGS, ROTARY_PUSH)) {
    rotaryInc *= 10;
    if (rotaryInc > maxinc) rotaryInc = 1;
    if (checkFlag(MFLAGS, CALIBRATION_MODE)) {
      pos = FrequencyDigitUpdate(rotaryInc) + ROTARY_NUMBER_OFFSET;
      LCDSelectLine (pos, y, 1);
    }
  }

  if (checkClearFlag(BFLAGS, PBUTTON1_PUSHED)) {
    if (checkFlag(MFLAGS, MEMORY_SAVE_MODE)) {
      setFlag (BFLAGS, DISABLE_BUTTONS);
      // mem[0] is autoupdated ever few seconds
      memcpy ((char *)&mem[rotaryNumber], (char *)&sg, sizeof(sg));
      SetMemClkStatus (0, rotaryNumber);
      EEPROM.put(0, mem);
      LCDErrorMsg(11, okmsg);
      delay (3000);
      LCDClearErrorMsg(10);
      clearFlag(BFLAGS, DISABLE_BUTTONS); 
      clearFlag(MFLAGS, MEMORY_SAVE_MODE);

    } else if (checkFlag(MFLAGS, MEMORY_RECALL_MODE)) {
      setFlag (BFLAGS, DISABLE_BUTTONS);
      memset ((char *)&mem, 0, sizeof (mem));
      EEPROM.get(0, mem);
      if (mem[rotaryNumber].flags == (MEM_ID | VERSION)) {
        memset ((char *)&sg, 0, sizeof (sg));
        memcpy ((char *)&sg, (char *)&mem[rotaryNumber], sizeof(sg));
        LCDErrorMsg(11, okmsg);
        delay (3000);
        LCDClearErrorMsg(10);
      } else {
        LCDErrorMsg(11, (char *)"MEM ERR");
        delay (3000);
        LCDClearErrorMsg(10);
      }
      RefreshLCD();
      clearFlag(BFLAGS, DISABLE_BUTTONS); 
      clearFlag(MFLAGS, MEMORY_RECALL_MODE);
   
    } else if (checkFlag(MFLAGS, CALIBRATION_MODE)) {
      setFlag (BFLAGS, DISABLE_BUTTONS);
      sg.correction = rotaryNumber;
      memcpy ((char *)&mem[0], (char *)&sg, sizeof(sg));
      SetMemClkStatus (0, 0);
      EEPROM.put(0, mem);
      LCDErrorMsg(11, okmsg);
      delay (3000);
      LCDClearErrorMsg(11);
      ResetSi5351();
      RefreshLCD();
      clearFlag(BFLAGS, DISABLE_BUTTONS); 
      clearFlag(MFLAGS, CALIBRATION_MODE);

    }

    ClearFlags();
    LCDSelectLine(0, 3, 1);
    setFlag (MFLAGS, MENU_MODE);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (checkClearFlag(BFLAGS, PBUTTON2_PUSHED)) {
    ClearFlags();
    ResetSi5351();
    RefreshLCD();
    LCDClearErrorMsg(11);
    LCDSelectLine(0, 3, 1);
    setFlag (MFLAGS, MENU_MODE);
    digitalWrite(LED_BUILTIN, LOW);
  }

}


void MenuClockFrequencyMode (void)
{
  long temp;
  unsigned char pos;
  rotaryChangeFlag = 0;
  pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;

  if (checkClearFlag(BFLAGS, ROTARY_CW)) {
    rotaryChangeFlag = 1;
    sg.ClkFreq[ClkSelection] += frequency_inc;
    if (sg.ClkFreq[ClkSelection] > HighFrequencyLimit(ClkSelection)) {
      sg.ClkFreq[ClkSelection] = HighFrequencyLimit(ClkSelection);
      pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
      LCDSelectLine (pos, ClkSelection, 1);
    }

  } else if (checkClearFlag(BFLAGS, ROTARY_CCW)) {
    rotaryChangeFlag = 1;
    temp = (long)sg.ClkFreq[ClkSelection] - (long)frequency_inc;
    if (temp < (long)LowFrequencyLimit(ClkSelection) || temp < 0) {
      sg.ClkFreq[ClkSelection] = LowFrequencyLimit(ClkSelection);
      pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
      LCDSelectLine (pos, ClkSelection, 1);

    } else {
      sg.ClkFreq[ClkSelection] -= frequency_inc;
    }

    if (checkFlag(MFLAGS, LO_FREQUENCY_MODE)) {
      temp = (long)sg.ClkFreq[ClkSelection] + sg.ClkOffset[ClkSelection];
      if (temp <= 0) {
          sg.ClkFreq[ClkSelection] = (unsigned long)absl(sg.ClkOffset[ClkSelection]) + LowFrequencyLimit(ClkSelection);
      }
     }
  }

  if ( rotaryChangeFlag ) {
    if (checkFlag(MFLAGS, LO_FREQUENCY_MODE )) {
      LCDDisplayLOClockFrequency (ClkSelection); 
    } else {
      LCDDisplayClockFrequency (ClkSelection); 
    }
  
    UpdateFrequency (ClkSelection);
    LCDSelectLine (pos, ClkSelection, 1);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (checkClearFlag(BFLAGS, ROTARY_PUSH)) {
    frequency_inc *= 10;
    if (frequency_inc > MAXIMUM_FREQUENCY_MULTIPLIER) frequency_inc = MINIMUM_FREQUENCY_MULTIPLIER;
    pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
    LCDSelectLine (pos, ClkSelection, 1);
  }

  if (checkClearFlag(BFLAGS, PBUTTON1_PUSHED)) {
    if (sg.ClkStatus[ClkSelection]) {
      sg.ClkStatus[ClkSelection] = 0;
      LCDDisplayClockStatus(ClkSelection);
      DisableFrequency(ClkSelection);

    } else {
      sg.ClkStatus[ClkSelection] = 1;
      LCDDisplayClockStatus(ClkSelection);
      EnableFrequency(ClkSelection);
    }
    pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
    LCDSelectLine(pos, ClkSelection, 1);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (checkClearFlag(BFLAGS, PBUTTON2_PUSHED)) {
    setFlag (MFLAGS, CLOCK_WINDOW_MODE);
    clearFlag(MFLAGS, CLOCK_FREQUENCY_MODE);
    LCDSelectLine(0, ClkSelection, 1);
    digitalWrite(LED_BUILTIN, LOW);
  }

}

void MenuIQClockFrequencyMode (void)
{
  long temp;
  unsigned char pos;
  rotaryChangeFlag = 0;
  pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;

  if (checkClearFlag(BFLAGS, ROTARY_CW)) {
    rotaryChangeFlag = 1;
    sg.IQClkFreq[ClkSelection] += frequency_inc;
    if (sg.IQClkFreq[ClkSelection] > HighFrequencyLimit(ClkSelection)) {
      sg.IQClkFreq[ClkSelection] = HighFrequencyLimit(ClkSelection);
      pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
      LCDSelectLine (pos, ClkSelection, 1);
    }

  } else if (checkClearFlag(BFLAGS, ROTARY_CCW)) {
    rotaryChangeFlag = 1;
    temp = (long)sg.IQClkFreq[ClkSelection] - (long)frequency_inc;
    if (temp < (long)LowFrequencyLimit(ClkSelection) || temp < 0) {
      sg.IQClkFreq[ClkSelection] = LowFrequencyLimit(ClkSelection);
      pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
      LCDSelectLine (pos, ClkSelection, 1);

    } else {
      sg.IQClkFreq[ClkSelection] -= frequency_inc;
    }

  }

  if ( rotaryChangeFlag ) {
    sg.IQClkFreq[0] = sg.IQClkFreq[ClkSelection]; 
    sg.IQClkFreq[1] = 0; 
    sg.IQClkFreq[2] = sg.IQClkFreq[ClkSelection]; 
    LCDDisplayIQClockFrequency (0);
    LCDDisplayIQClockFrequency (1);
    LCDDisplayIQClockFrequency (2);
    UpdateIQFrequency (ClkSelection);
    LCDSelectLine (pos, ClkSelection, 1);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (checkClearFlag(BFLAGS, ROTARY_PUSH)) {
    frequency_inc *= 10;
    if (frequency_inc > MAXIMUM_FREQUENCY_MULTIPLIER) frequency_inc = MINIMUM_FREQUENCY_MULTIPLIER;
    pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
    LCDSelectLine (pos, ClkSelection, 1);
  }

  if (checkClearFlag(BFLAGS, PBUTTON1_PUSHED)) {
    if (!ClkSelection) ClkSelection = 2;
    else ClkSelection = 0;
    LCDSelectLine(pos, ClkSelection, 1);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (checkClearFlag(BFLAGS, PBUTTON2_PUSHED)) {
    ClearFlags();
    setFlag (MFLAGS, MENU_MODE);
    LCDSelectLine(0, 3, 1);
    digitalWrite(LED_BUILTIN, LOW);
  }

}


void MenuClockFrequencyOffsetMode (void)
{
  long temp;
  unsigned char pos;
  rotaryChangeFlag = 0;

  pos = FrequencyDigitUpdate(offset_inc);
  pos += OFFSET_DISPLAY_SHIFT;

  if (checkClearFlag(BFLAGS, ROTARY_CW)) {
    rotaryChangeFlag = 1;
    sg.ClkOffset[ClkSelection] += offset_inc;
    if (sg.ClkOffset[ClkSelection] > MAXIMUM_OFFSET_FREQUENCY) {
      sg.ClkOffset[ClkSelection] = MAXIMUM_OFFSET_FREQUENCY;
    }

  } else if (checkClearFlag(BFLAGS, ROTARY_CCW)) {
    rotaryChangeFlag = 1;
    temp = sg.ClkOffset[ClkSelection] - offset_inc;
    if (temp < (-MAXIMUM_OFFSET_FREQUENCY) ) {
      sg.ClkOffset[ClkSelection] = (-MAXIMUM_OFFSET_FREQUENCY);

    // This may be redundant but keep it for future
    } else if (sg.ClkOffset[ClkSelection] > 1000000 && temp < 1000000) {
      sg.ClkOffset[ClkSelection] -= offset_inc;

    } else {
      sg.ClkOffset[ClkSelection] -= offset_inc;
    }
  }

  if ( rotaryChangeFlag ) {
    LCDDisplayOffsetFrequency (ClkSelection);
    pos = FrequencyDigitUpdate(offset_inc);
    pos += OFFSET_DISPLAY_SHIFT;
    LCDSelectLine (pos, ClkSelection, 1);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (checkClearFlag(BFLAGS, ROTARY_PUSH)) {
    offset_inc *= 10;
    if (offset_inc > MAXIMUM_OFFSET_MULTIPLIER) offset_inc = MINIMUM_OFFSET_MULTIPLIER;
    pos = FrequencyDigitUpdate(offset_inc);
    pos += OFFSET_DISPLAY_SHIFT;
    LCDSelectLine (pos, ClkSelection, 1);
  }

  if (checkClearFlag(BFLAGS, PBUTTON1_PUSHED)) {
    ClkSelection++;
    if (ClkSelection >= 3) ClkSelection = 0;
    pos = FrequencyDigitUpdate(offset_inc);
    pos += OFFSET_DISPLAY_SHIFT;
    LCDSelectLine (pos, ClkSelection, 1);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (checkClearFlag(BFLAGS, PBUTTON2_PUSHED)) {
    ClearFlags();
    setFlag (MFLAGS, MENU_MODE);
    LCDSelectLine(0, 3, 1);
    digitalWrite(LED_BUILTIN, LOW);
  }

}

void MenuClockWindowMode ()
{
  unsigned char pos;

  rotaryChangeFlag = 0;
  if (checkClearFlag(BFLAGS, ROTARY_CW)) {
    rotaryChangeFlag = 1;
    if (++ClkSelection >= MAXCLK) ClkSelection = 0;

  } else if (checkClearFlag(BFLAGS, ROTARY_CCW)) {
    rotaryChangeFlag = 1;
    if (!ClkSelection) ClkSelection = MAXCLK - 1;
    else ClkSelection--;
  }

 if ( rotaryChangeFlag ) {
  LCDSelectLine(0, ClkSelection, 1);
  digitalWrite(LED_BUILTIN, LOW);
 }

 if (checkClearFlag(BFLAGS, ROTARY_PUSH)) {
  
#ifdef ENABLE_SWAP_VFO
  // Swap CLK2 and CLK2
  if ( (ClkSelection == 0 || ClkSelection == 2) && sg.ClkStatus[0] && sg.ClkStatus[2]) {
   // Swap frequency
   frequency_clk = sg.ClkFreq[0]; 
   sg.ClkFreq[0] = sg.ClkFreq[2]; 
   sg.ClkFreq[2] = frequency_clk;
   // Swap offset
   offset_frequency = sg.ClkOffset[0]; 
   sg.ClkOffset[0] = sg.ClkOffset[2]; 
   sg.ClkOffset[2] = offset_frequency;
   if (checkFlag(MFLAGS,  LO_FREQUENCY_MODE)) {
    LCDDisplayLOClockFrequency (0);
    LCDDisplayLOClockFrequency (2);    
   } else {
    LCDDisplayClockEntry(0);
    LCDDisplayClockEntry(2);
   }
   UpdateFrequency (0);
   UpdateFrequency (2);   
   LCDSelectLine(0, ClkSelection, 1);
  }
#endif // ENABLE_SWAP_VFO

  digitalWrite(LED_BUILTIN, LOW);
 }

 if (checkClearFlag(BFLAGS, PBUTTON1_PUSHED)) {
  if (sg.ClkStatus[ClkSelection]) {
   sg.ClkStatus[ClkSelection] = 0;
   LCDDisplayClockStatus(ClkSelection);
   LCDSelectLine(0, ClkSelection, 1);
   DisableFrequency(ClkSelection);

  } else {
   sg.ClkStatus[ClkSelection] = 1;
   LCDDisplayClockStatus(ClkSelection);
   pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
   LCDSelectLine (pos, ClkSelection, 1);
   EnableFrequency(ClkSelection);
   clearFlag(MFLAGS, CLOCK_WINDOW_MODE);
   setFlag(MFLAGS, CLOCK_FREQUENCY_MODE);
  }
  digitalWrite(LED_BUILTIN, LOW);
 }

 if (checkClearFlag(BFLAGS, PBUTTON2_PUSHED)) {
  LCDSelectLine(0, ClkSelection, 0);
  ClearFlags();
  setFlag (MFLAGS, MENU_MODE);
  LCDSelectLine(0, 3, 1);
  digitalWrite(LED_BUILTIN, LOW);
 }

}


void MenuDisplayMode ()
{
 rotaryChangeFlag = 0;
 if (checkClearFlag(BFLAGS, ROTARY_CW)) {
  rotaryChangeFlag = 1;
  MenuSelection++;
  if (MenuSelection >= MAXMENU_ITEMS) {
   MenuSelection = 0;
  }

 } else if (checkClearFlag(BFLAGS, ROTARY_CCW)) {
  rotaryChangeFlag = 1;
  if (MenuSelection) {
   MenuSelection--;
  } else {
   MenuSelection = MAXMENU_ITEMS - 1;
  }
 }

 if ( rotaryChangeFlag ) {
  LCDDisplayMenuOption (MenuSelection);
  LCDSelectLine(0, 3, 1);
  digitalWrite(LED_BUILTIN, LOW);
 }

 if (checkClearFlag(BFLAGS, ROTARY_PUSH)) {
  DoMenu();
 }

 if (checkClearFlag(BFLAGS, PBUTTON1_PUSHED)) {
  DoMenu();
  digitalWrite(LED_BUILTIN, LOW);
 }

 if (checkClearFlag(BFLAGS, PBUTTON2_PUSHED)) {
  DoMenu();
  digitalWrite(LED_BUILTIN, LOW);
 }

}


void DoMenu(void)
{
 unsigned char pos;

 switch (MenuSelection) {
  case VFO_ENABLE:
   ResetSi5351();
   sg.ClkMode[0] = VFO_CLK_MODE;
   sg.ClkMode[1] = VFO_CLK_MODE;
   sg.ClkMode[2] = VFO_CLK_MODE;
   sg.ClkStatus[0] = 0;      // Off
   sg.ClkStatus[1] = 0;      // Off
   sg.ClkStatus[2] = 0;      // Off
   
   ClearFlags();
   setFlag (MFLAGS, CLOCK_WINDOW_MODE);
  
   LCDClearClockWindow();
   LCDDisplayClockEntry(0);
   LCDDisplayClockEntry(1);
   LCDDisplayClockEntry(2);

   ClkSelection = 0;
   LCDSelectLine(0, ClkSelection, 1);
   break;

  case LO_ENABLE:
   ResetSi5351();
   sg.ClkMode[0] = VFO_CLK_MODE;
   sg.ClkMode[1] = VFO_CLK_MODE;
   sg.ClkMode[2] = VFO_CLK_MODE;
   if (sg.ClkOffset[0]) sg.ClkMode[0] = LO_CLK_MODE;
   if (sg.ClkOffset[1]) sg.ClkMode[1] = LO_CLK_MODE;
   if (sg.ClkOffset[2]) sg.ClkMode[2] = LO_CLK_MODE;
   sg.ClkStatus[0] = 0;      // Off
   sg.ClkStatus[1] = 0;      // Off
   sg.ClkStatus[2] = 0;      // Off
   
   ClearFlags();
   setFlag (MFLAGS, CLOCK_WINDOW_MODE);
   setFlag (MFLAGS, LO_FREQUENCY_MODE);

   if ((sg.ClkOffset[0] + (long)sg.ClkFreq[0]) <= 0) sg.ClkFreq[0] = absl(sg.ClkOffset[0]) + DEFAULT_LOW_FREQUENCY_LIMIT;
   if ((sg.ClkOffset[1] + (long)sg.ClkFreq[1]) <= 0) sg.ClkFreq[1] = absl(sg.ClkOffset[1]) + DEFAULT_LOW_FREQUENCY_LIMIT;
   if ((sg.ClkOffset[2] + (long)sg.ClkFreq[2]) <= 0) sg.ClkFreq[2] = absl(sg.ClkOffset[2]) + DEFAULT_LOW_FREQUENCY_LIMIT;
   LCDClearClockWindow();
   LCDDisplayLOClockFrequency(0);
   LCDDisplayLOClockFrequency(1);
   LCDDisplayLOClockFrequency(2);
   LCDDisplayClockMode (0);
   LCDDisplayClockMode (1);
   LCDDisplayClockMode (2);
   LCDDisplayClockStatus(0);
   LCDDisplayClockStatus(1);
   LCDDisplayClockStatus(2);
   LCDSelectLine(0, 3, 0);
   ClkSelection = 0;
   LCDSelectLine(0, ClkSelection, 1);
   break;

  case IQ_ENABLE:
   ResetSi5351();
   sg.ClkMode[0] = IQ_CLK_MODE;
   sg.ClkMode[1] = IQ_CLK_MODE;
   sg.ClkMode[2] = IQ_CLK_MODE;
   sg.ClkStatus[0] = 1;
   sg.ClkStatus[1] = 0;
   sg.ClkStatus[2] = 1;
   
   ClearFlags();
   setFlag (MFLAGS, IQ_FREQUENCY_MODE);
   
   LCDClearClockWindow();
   LCDDisplayIQClockFrequency (0);
   LCDDisplayIQClockFrequency (1);
   LCDDisplayIQClockFrequency (2);
   LCDDisplayClockMode (0);
   LCDDisplayClockMode (1);
   LCDDisplayClockMode (2);
   LCDDisplayClockStatus(0);
   LCDDisplayClockStatus(1);
   LCDDisplayClockStatus(2);
   
   ClkSelection = 0;
   pos = FrequencyDigitUpdate(frequency_inc);
   pos += FREQUENCY_DISPLAY_SHIFT;
   LCDSelectLine (pos, ClkSelection, 1);
   
   UpdateIQFrequency (ClkSelection);
   break;

  case SET_OFFSET:
   ResetSi5351();
   SetMemClkStatus (0, 0);
   
   ClearFlags();
   setFlag (MFLAGS, OFFSET_FREQUENCY_MODE);

   LCDClearClockWindow();
   LCDDisplayOffsetFrequency (0);
   LCDDisplayOffsetFrequency (1);
   LCDDisplayOffsetFrequency (2);
   
   ClkSelection = 0;
   pos = FrequencyDigitUpdate(offset_inc);
   pos += OFFSET_DISPLAY_SHIFT;
   LCDSelectLine (pos, ClkSelection, 1);
   break;

  case CALIBRATE:
   ResetSi5351();
   SetMemClkStatus (1, 0);
   sg.ClkMode[0] = VFO_CLK_MODE;
   sg.ClkMode[1] = VFO_CLK_MODE;
   sg.ClkMode[2] = VFO_CLK_MODE;
   sg.ClkStatus[0] = 1;
   sg.ClkStatus[1] = 1;
   sg.ClkStatus[2] = 1;
   
   ClearFlags();
   setFlag (MFLAGS, CALIBRATION_MODE);
   
   LCDClearClockWindow();
   LCDDisplayClockEntry(0);
   LCDDisplayClockEntry(1);
   LCDDisplayClockEntry(2);

   UpdateFrequency (0);
   UpdateFrequency (1);
   UpdateFrequency (2);   

   rotaryNumber = sg.correction;
   rotaryInc = 10;
   LCDDisplayNumber3D (rotaryNumber, 11, 3);
   pos = FrequencyDigitUpdate(rotaryInc) + ROTARY_NUMBER_OFFSET;
   LCDSelectLine (pos, 3, 1);
   
   break;

  case SAVE:
   ResetSi5351();
   SetMemClkStatus (0, 0);
   RefreshLCD();
   
   ClearFlags();
   setFlag (MFLAGS, MEMORY_SAVE_MODE);
   
   rotaryNumber = 1;
   rotaryInc = 1;
   LCDDisplayNumber1D (rotaryNumber, 11, 3);
   LCDSelectLine (11, 3, 1);
   break;

  case RECALL:
   ResetSi5351();
   SetMemClkStatus (0, 0);
   RefreshLCD();
   
   ClearFlags();
   setFlag (MFLAGS, MEMORY_RECALL_MODE);
   
   rotaryNumber = 1;
   rotaryInc = 1;
   LCDDisplayNumber1D (rotaryNumber, 11, 3);
   LCDSelectLine (11, 3, 1);
   break;

  case CLI_ENABLE:

#ifndef REMOVE_CLI
   ResetSi5351();
   LCDDisplayHeader();
   FlushSerialInput();
   Serial.print (header1);
   Serial.println (header2);
   Serial.write (prompt);
   Serial.flush();
   ClearFlags();
   setFlag (MFLAGS, CLI_MODE);
   setFlag (MFLAGS, DISABLE_BUTTONS);
#else 
   LCDErrorMsg(11, (char *)"ERROR");
   delay (3000);
   LCDClearErrorMsg(11);  
#endif //REMOVE_CLI
   break;

  case RESET:
   Reset();
   return;
   break;
 }


}

void ClearFlags ()
{
 clearFlag(MFLAGS, MENU_MODE);
 clearFlag(MFLAGS, CLOCK_WINDOW_MODE);
 clearFlag(MFLAGS, CLOCK_FREQUENCY_MODE);
 clearFlag(MFLAGS, LO_FREQUENCY_MODE);
 clearFlag(MFLAGS, IQ_FREQUENCY_MODE);
 clearFlag(MFLAGS, OFFSET_FREQUENCY_MODE);
 clearFlag(MFLAGS, CALIBRATION_MODE);
 clearFlag(MFLAGS, MEMORY_SAVE_MODE);
 clearFlag(MFLAGS, MEMORY_RECALL_MODE);
 clearFlag(MFLAGS, CLI_MODE);
// clearFlag(MFLAGS, MASTER_RESET);    // This should never be cleared.
 
}




unsigned char FrequencyDigitUpdate (long inc)
{
 switch (inc) {
  case 1:
   return 8;
   break;

  case 10:
   return 7;
   break;

  case 100:
   return 6;
   break;

  case 1000:
   return 5;
   break;

  case 10000:
   return 4;
   break;

  case 100000:
   return 3;
   break;

  case 1000000:
   return 2;
   break;

  case 10000000:
   return 1;
   break;

  default:
   return 1;
   break;

 }


}

unsigned long HighFrequencyLimit (unsigned char line)
{
 
 if (checkFlag(MFLAGS, IQ_FREQUENCY_MODE)) return SI_MAX_IQ_OUT_FREQ;

 switch (line) {  
  case 0:
  case 2:
   return DEFAULT_HIGH_FREQUENCY_LIMIT; 
   break;
   
  case 1:
   return SI_MAX_OUT_FREQ; 
   break;
 }

 return DEFAULT_HIGH_FREQUENCY_LIMIT;

}

unsigned long LowFrequencyLimit (unsigned char line)
{
 if (checkFlag(MFLAGS, IQ_FREQUENCY_MODE)) return SI_MIN_IQ_OUT_FREQ;
 
 switch (line) {
  case 0:
  case 2:
   return DEFAULT_LOW_FREQUENCY_LIMIT;
   break; 
   
  case 1:
   return SI_MIN_OUT_FREQ;
   break;
 }

 return DEFAULT_LOW_FREQUENCY_LIMIT;

}

void UpdateIQFrequency (unsigned char line)
{

 if (sg.IQClkFreq[line] < SI_MIN_IQ_OUT_FREQ) sg.IQClkFreq[line] = SI_MIN_IQ_OUT_FREQ;
 if (sg.IQClkFreq[line] > SI_MAX_IQ_OUT_FREQ) sg.IQClkFreq[line] = SI_MAX_IQ_OUT_FREQ;
   
 SetIQFrequency (SI_CLK0, SI_CLK2, SI_PLL_A, sg.IQClkFreq[line]);
 
}

void SetMemClkStatus (unsigned char stat, unsigned char index) 
{
  mem[index].ClkStatus[0] = mem[index].ClkStatus[1] = mem[index].ClkStatus[2] = stat;
}


void UpdateFrequency (unsigned char line)
{
 unsigned long freq;

 freq = sg.ClkFreq[line];

 if (freq < LowFrequencyLimit(line)) freq = LowFrequencyLimit(line);
 if (freq > HighFrequencyLimit(line)) freq = HighFrequencyLimit(line);
 
 switch (line) {
  case 0:
   SetFrequency (SI_CLK0, SI_PLL_A, freq);
   break;

  case 1:
   SetFrequency (SI_CLK1, SI_PLL_B, freq);
   break;

  case 2:
   SetFrequency (SI_CLK2, SI_PLL_A, freq);
   break;
   
 }


}

void EnableFrequency (unsigned char line)
{
 UpdateFrequency(line);
}

void DisableFrequency (unsigned char line)
{
 switch (line) {
  case 0:
   PowerDownSi5351Clock (SI_CLK0);
   break;
   
  case 1:
   PowerDownSi5351Clock (SI_CLK1);
   break;
   
  case 2:
   PowerDownSi5351Clock (SI_CLK2);
   break;
 }
}

void RefreshLCD (void)
{
 LCDClearScreen();
 sg.ClkMode[0] = sg.ClkMode[1] = sg.ClkMode[2] = VFO_CLK_MODE;
 sg.ClkStatus[0] = sg.ClkStatus[1] = sg.ClkStatus[2] = 0;
 LCDDisplayClockEntry(0);
 LCDDisplayClockEntry(2);
 LCDDisplayClockEntry(1);
 LCDDisplayMenuOption(MenuSelection);
 LCDSelectLine(0, 3, 1); 
}

void Reset (void)
{
 
#ifndef REMOVE_CLI
 ResetSerial();
 Serial.print (header1);
 Serial.println (header2);
// Serial.write (prompt);
 Serial.flush();
#endif // REMOVE_CLI

 ResetSi5351();

 frequency_clk = DEFAULT_FREQUENCY;
 frequency_inc = DEFAULT_FREQUENCY_INCREMENT;
 offset_inc = DEFAULT_FREQUENCY_INCREMENT;
 calibration_mult = DEFAULT_CALIBRATION_INCREMENT;

 memset (clearlcdline, 0x20, sizeof(clearlcdline));
 clearlcdline[LCD_CLEAR_LINE_LENGTH-1] = 0;
 memset (clearlcderrmsg, 0x20, sizeof (clearlcderrmsg));
 clearlcderrmsg[LCD_ERROR_MSG_LENGTH-1] = 0;
 memset (okmsg, 0x20, sizeof (okmsg));
 okmsg[LCD_ERROR_MSG_LENGTH-1] = 0;
 okmsg[0] = 'O';
 okmsg[1] = 'K';

 // Read sg from EEPROM
 memset ((char *)&sg, 0, sizeof (sg));
 memset ((char *)&mem, 0, sizeof (mem));
 EEPROM.get(0, mem);
 memcpy ((char *)&sg, (char *)&mem[0], sizeof(sg));

 if (sg.flags != (MEM_ID | VERSION)) {
  sg.flags = (MEM_ID | VERSION);
  sg.ClkFreq[0] = 1000000;
  sg.ClkFreq[1] = 1000000;
  sg.ClkFreq[2] = 1000000;
  sg.IQClkFreq[0] = 3000000;
  sg.IQClkFreq[1] = 0;
  sg.IQClkFreq[2] = 3000000;
  sg.ClkOffset[0] = 0;
  sg.ClkOffset[1] = 0;
  sg.ClkOffset[2] = 0;
  sg.ClkMode[0] = sg.ClkMode[1] = sg.ClkMode[2] = VFO_CLK_MODE;
  sg.ClkStatus[0] = sg.ClkStatus[1] = sg.ClkStatus[2] = 0;
  sg.correction = 0;
  memcpy ((char *)&mem[0], (char *)&sg, sizeof(sg));
  EEPROM.put(0, mem);
 }

 rotaryNumber = 0;
 rotaryInc = 1;

 ResetEncoder();

 // LCD Menu
 LCDDisplayHeader();

 MenuSelection = 0;
 ClkSelection = 0;

 RefreshLCD();

 mflags = MENU_MODE;
 bflags = 0;
}

long absl (long v)
{
 if (v < 0) return (-v);
 else return v;
}



unsigned char checkFlag (unsigned char type, unsigned long bitmask)
{
 unsigned char ret;

 if (type) {
  ret = 0;
  cli();
  if (bflags & bitmask) ret = 1;
  sei();
  return ret;
  
 } else {
  if ( mflags & bitmask) return 1;
  else return 0;
  
 }
}

unsigned char checkClearFlag (unsigned char type, unsigned long bitmask)
{
 unsigned char ret;

 if (type) {
  ret = 0;
  cli();
  if (bflags & bitmask) {
    ret = 1;
    bflags &= ~bitmask;
  }
  sei();
  return ret;
  
 } else {
  if ( mflags & bitmask) {
    mflags &= ~bitmask;
    return 1;
  }
  else return 0;
  
 }
}

void setFlag (unsigned char type, unsigned long bitmask)
{
 if (type) {
  cli();
  bflags |= bitmask;
  sei();
  
 } else {
  mflags |= bitmask;
  
 }
}

void clearFlag (unsigned char type, unsigned long bitmask)
{
 if (type) {
  cli();
  bflags &= ~bitmask;
  sei();
  
 } else {
  mflags &= ~bitmask;
 }
}





#ifndef REMOVE_CLI

// Place program specific content here
void ExecuteSerial (char *str)
{
 // num defined the actual number of entries process from the serial buffer
 // i is a generic counter
 unsigned char num;
 unsigned char clk, phase;

 // This function called when serial input in present in the serial buffer
 // The serial buffer is parsed and characters and numbers are scraped and entered
 // in the commands[] and numbers[] variables.
 num = ParseSerial (str);

 // Process the commands
 // Note: Whenever a parameter is stated as [CLK] the square brackets are not entered. The square brackets means
 // that this is a command line parameter entered after the command.
 // E.g. F [CLK] [FREQ] would be mean "F 0 7000000" is entered (no square brackets entered)
 switch (commands[0]) {

  // Calibrate the Si5351.
  // Syntax: C [CAL] [FREQ], where CAL is the new Calibration value and FREQ is the frequency to output
  // Syntax: C , If no parameters specified, it will display current calibration value
  // Bascially you can set the initial CAL to 100 and check fequency accurate. Adjust up/down as needed
  // numbers[0] will contain the correction, numbers[1] will be the frequency in Hz
  case 'C':       // Calibrate
   // First, Check inputs to validate
   if (numbers[0] == 0UL && numbers[1] == 0UL) {
    Serial.print (F("Correction: "));
    Serial.println (sg.correction);
    break;
    
   } else if (numbers[0] > 500) {
    Serial.println (F("Bad Cal"));
    break;
    
   } else if (numbers[1] < SI_MIN_OUT_FREQ || numbers[1] > SI_MAX_OUT_FREQ) {
    Serial.println (F("Bad Freq"));
    break;
   }

   // New value defined so read the old values and display what will be done
   Serial.print (F("Old Correction: "));
   Serial.println (sg.correction);
   Serial.print ("New Correction: ");
   
   // Store the new value entered  
   if (commands[1] == '-') sg.correction = - (int)numbers[0];
   
   else sg.correction = (int)numbers[0];
   Serial.println (sg.correction);

   // Reset the Si5351 and then display frequency based on new setting   
   setupSi5351(sg.correction);
   memcpy ((char *)&mem[0], (char *)&sg, sizeof(sg));
   EEPROM.put(0, mem);
   SetFrequency (SI_CLK0, SI_PLL_A, numbers[1]);
   SetFrequency (SI_CLK1, SI_PLL_A, numbers[1]);
   SetFrequency (SI_CLK2, SI_PLL_A, numbers[1]);
   break;

  case 'F':       // Set Frequency
   // Validate inputs
   if (numbers [0] > 2UL) {
    Serial.println (F("Bad Clk"));
    break;
   }

   if (numbers[1] < SI_MIN_OUT_FREQ || numbers[1] > SI_MAX_OUT_FREQ) {
    Serial.println (F("Bad Freq"));
    break;
   }

   // set frequency
   if (numbers[0] == 0UL) {
    if (commands[1] == 'A') SetFrequency (SI_CLK0, SI_PLL_A, numbers[1]);
    else SetFrequency (SI_CLK0, SI_PLL_B, numbers[1]); 

   } else if (numbers[0] == 1UL) {
    if (commands[1] == 'A') SetFrequency (SI_CLK1, SI_PLL_A, numbers[1]);
    else SetFrequency (SI_CLK1, SI_PLL_B, numbers[1]);

   } else if (numbers[0] == 2UL) {
    if (commands[1] == 'A') SetFrequency (SI_CLK2, SI_PLL_A, numbers[1]);
    else SetFrequency (SI_CLK2, SI_PLL_B, numbers[1]);

   }
   break;

  case 'I':       // Memory setting
   memset ((char *)&mem, 0xFF, sizeof (mem));
   EEPROM.put(0, mem);
   break;


  case 'M':       // Memory setting
   printMem(0);
   printMem(1);
   printMem(2);
   printMem(3);
   break;

  // Phase controls
  case 'P':
   if (numbers[0] > 2UL){
    Serial.println (F("Bad CLK"));
    return;
   }
   clk = (unsigned char)numbers[0];
   
   if (numbers[1] > 128UL || numbers[1] < 4UL ){
    Serial.println (F("Bad Phase"));
    return;
   }
   phase = (unsigned char)numbers[1];

   Serial.print (F("Clk: "));
   Serial.print (clk);
   Serial.print (F(" Phase: "));
   Serial.println (phase);

   UpdatePhaseRegister (clk, phase);   
   break;

  case 'Q':       // Reset
   if (numbers[0] < SI_MIN_IQ_OUT_FREQ || numbers[1] > SI_MAX_IQ_OUT_FREQ) {
    Serial.println (F("Bad Freq"));
    break;
   }
       
   SetIQFrequency (SI_CLK0, SI_CLK2, SI_PLL_A, numbers[0]);
   
   break;

  // This command reset the Si5351. A reset zeros all parameters including the correction/calibration value
  // Therefore the calibration must be re-read from eeprom
  case 'R':       // Reset
   Reset();
   break;

  // If an undefined command is entered, display an error message
  default:
   ErrorOut ();
 }

}


void printMem (unsigned char i) 
{
 EEPROM.get(0, mem);
 memset(rbuff,0,sizeof(rbuff));
 if (mem[i].flags == (MEM_ID | VERSION)) {
  sprintf (rbuff, "Mem: %d Corr: %d", i, mem[i].correction);
  Serial.println (rbuff);
  sprintf (rbuff, "\tVFO1: %ld VFO2: %ld VFO3: %ld", mem[i].ClkFreq[0], mem[i].ClkFreq[1], mem[i].ClkFreq[2]);
  Serial.println (rbuff);
  sprintf (rbuff, "\tOFF1: %ld OFF2: %ld OFF3: %ld", mem[i].ClkOffset[0], mem[i].ClkOffset[1], mem[i].ClkOffset[2]);
  Serial.println (rbuff);
  sprintf (rbuff, "\tIQ1: %ld IQ2: %ld IQ3: %ld", mem[i].IQClkFreq[0], mem[i].IQClkFreq[1], mem[i].IQClkFreq[2]);
  Serial.println (rbuff);
  memset(rbuff,0,sizeof(rbuff));
 } else {
  Serial.println ((char *)"MEM ERR");
 }

}

#endif // REMOVE_CLI
