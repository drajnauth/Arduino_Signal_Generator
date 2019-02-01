/*

  Program Written by Dave Rajnauth, VE3OOI to control the Si5351.
  
  Software is licensed (Non-Exclusive Licence) for use by the Peel Amateur Radion Club.  

  All other uses licensed under a Creative Commons Attribution 4.0 International License.

 */
 
#include "Arduino.h"

#include <stdint.h>
#include <avr/eeprom.h>        // Needed for storeing calibration to Arduino EEPROM
#include <Wire.h>              // Needed to communitate I2C to Si5351
#include <SPI.h>               // Needed to communitate I2C to Si5351
#include <EEPROM.h>

#include "VE3OOI_Si5351_Signal_Generator.h"   // Defines for this program
#include "LCD.h"
#include "Encoder.h"
#include "Timer.h"
#include "si5351.h"

#ifndef REMOVE_CLI

#include "UART.h"                             // VE3OOI Serial Interface Routines (TTY Commands)

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


volatile unsigned long flags;

// Encoder Variables
volatile int old_AB = 0;
volatile unsigned char encoderVal, encoderState;
volatile unsigned int pbstate;
volatile unsigned long pbreset;

// Push Button Debounce
volatile int pb1state, pb2state;             // the current reading from the input pin
volatile int oldpb1state, oldpb2state;       // the current reading from the input pin
volatile unsigned long pb1time, pb2time;  // the last time the output pin was toggle

// Frequency Control variables
volatile unsigned long frequency_clk;
volatile unsigned long frequency_inc;
volatile long offset_inc;
volatile int calibration_mult;
volatile unsigned long long freqll, pllfreqll;

// Calibration and memory slot variable
int rotaryNumber;
int rotaryInc;

// LCD Menu
volatile unsigned char MenuSelection, ClkSelection;

char RootMenuOptions[MAXMENU_ITEMS][MAXMENU_LEN] = {
  {"VFO ENABLE\0"},
  {"LO ENABLE \0"},
  {"I/Q ENABLE\0"},
  {"SET OFFSET\0"},
  {"CALIBRATE \0"},
  {"SAVE      \0"},
  {"RECALL    \0"},
  {"TBD       \0"},
  {"TBD       \0"},
  {"TBD       \0"},
  {"TBD       \0"},
  {"RESET     \0"}
};

char header1[HEADER1] = {'P', 'A', 'R', 'C', ' ', 'S', 'I', 'G', ' ', 'G', 'E', 'N', ' ', 'V', '0', '.', '1', 'a', 'E', 0x0};
char header2[HEADER2] = {' ', '(', 'C', ')', 'V', 'E', '3', 'O', 'O', 'I', 0x0};

char clkentry [CLKENTRYLEN];

char prompt[6] = {0xa, 0xd, ':', '>', ' ', 0x0};
char ovflmsg[9] = {'O', 'v', 'e', 'r', 'f', 'l', 'o', 'w', 0x0};
char errmsg[4] = {'E', 'r', 'r', 0x0};

// Specific Sig Gen parameters
Sig_Gen_Struct sg;
Sig_Gen_Struct mem[4];
unsigned long savectr;

Si5351 si5351;



// the setup function runs once when you press reset or power the board
void setup() {
  // define the baud rate for TTY communications. Note CR and LF must be sent by terminal program
  Serial.begin(115200);

  SetupLCD ();
  
#ifndef REMOVE_CLI
  ResetSerial ();
#endif // REMOVE_CLI

  Reset ();

  if (sg.correction > 1000 || sg.correction < -1000) {
    sg.correction = 0;
  }
  
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, SI5351_XTAL_FREQ, (long)sg.correction);

  SetupEncoder();

  savectr = millis();

  MenuSelection = 0;
  ClkSelection = 0;
}





// the loop function runs over and over again forever
void loop()
{

#ifndef REMOVE_CLI
  // Look for characters entered from the keyboard and process them
  // This function is part of the UART package.
  ProcessSerial ();
#endif // REMOVE_CLI



  if (flags & MASTER_RESET) {
    Reset();
  }

  if (flags & MENU_MODE) {
    MenuDisplayMode();

  } else if (flags & CLOCK_WINDOW_MODE) {
    MenuClockWindowMode();

  } else if (flags & CLOCK_FREQUENCY_MODE) {
    MenuClockFrequencyMode();

  } else if (flags & IQ_FREQUENCY_MODE) {
    MenuIQClockFrequencyMode();

  } else if (flags & OFFSET_FREQUENCY_MODE) {
    MenuClockFrequencyOffsetMode();
    
  } else if (flags & CALIBRATION_MODE) {
    GetRotaryNumber (-500, 500, 100, 11, 3);

  } else if (flags & MEMORY_SAVE_MODE) {
    GetRotaryNumber (1, 3, 1, 11, 3);

  } else if (flags & MEMORY_RECALL_MODE) {
    GetRotaryNumber (1, 3, 1, 11, 3);
  }

  // save running config in EEPROM array
  if (millis() -  savectr > 1000) {
    memcpy ((char *)&mem[0], (char *)&sg, sizeof(sg));
    SetMemClkStatus (0, 0);
    savectr = millis();
  }

}



int GetRotaryNumber (int lnum, int hnum, int maxinc, unsigned char x, unsigned char y)
{
  unsigned char pos;
  pos = FrequencyDigitUpdate(rotaryInc) + ROTARY_NUMBER_OFFSET;

  if (flags & ROTARY_CW) {
    rotaryNumber += rotaryInc;
    if (rotaryNumber > hnum) rotaryNumber = hnum;

  } else if (flags & ROTARY_CCW) {
    rotaryNumber -= rotaryInc;
    if (rotaryNumber < lnum) rotaryNumber = lnum;
  }

  if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
    flags &= ~ROTARY_CW;
    flags &= ~ROTARY_CCW;
    if (encoderState & 0x2) encoderState &= ~0x2;
    if (encoderState & 0x1) encoderState &= ~0x1;

    if (flags & MEMORY_RECALL_MODE || flags & MEMORY_SAVE_MODE) {
      LCDDisplayNumber1D (rotaryNumber, x, y);
      LCDSelectLine (x, y, 1);
      
    } else if (flags & CALIBRATION_MODE) {
      LCDDisplayNumber3D (rotaryNumber, x, y);
      LCDSelectLine (pos, y, 1);
      si5351.set_correction((long)rotaryNumber, SI5351_PLL_INPUT_XO);
      flags &= ~PLLA_RUNNING;
      flags &= ~PLLB_RUNNING;
      UpdateFrequency (0);
      UpdateFrequency (1);
      UpdateFrequency (2);
    }

  }

  if (flags & ROTARY_PUSH) {
    rotaryInc *= 10;
    if (rotaryInc > maxinc) rotaryInc = 1;
    if (flags & CALIBRATION_MODE) {
      pos = FrequencyDigitUpdate(rotaryInc) + ROTARY_NUMBER_OFFSET;
      LCDSelectLine (pos, y, 1);
    }
    flags &= ~ROTARY_PUSH;
  }

  if (flags & PBUTTON1_PUSHED) {
    if (flags & MEMORY_SAVE_MODE) {
      flags |= DISABLE_BUTTONS;
      memcpy ((char *)&mem[rotaryNumber], (char *)&sg, sizeof(sg));
      SetMemClkStatus (0, rotaryNumber);
      EEPROM.put(0, mem);
      flags &= ~DISABLE_BUTTONS; 
      LCDErrorMsg(11, (char *)"OK     ");
      delay (3000);
      LCDClearErrorMsg(10);
      flags &= ~MEMORY_SAVE_MODE;

    } else if (flags & MEMORY_RECALL_MODE) {
      flags |= DISABLE_BUTTONS;
      memset ((char *)&mem, 0, sizeof (mem));
      EEPROM.get(0, mem);
      if (mem[rotaryNumber].flags == 0xFEEDFACE) {
        memset ((char *)&sg, 0, sizeof (sg));
        memcpy ((char *)&sg, (char *)&mem[rotaryNumber], sizeof(sg));
        LCDErrorMsg(11, (char *)"OK     ");
        delay (3000);
        LCDClearErrorMsg(10);
      } else {
        LCDErrorMsg(11, (char *)"MEM ERR");
        delay (3000);
        LCDClearErrorMsg(10);
      }
      flags &= ~DISABLE_BUTTONS;
      flags &= ~MEMORY_RECALL_MODE;
      
    } else if (flags & CALIBRATION_MODE) {
      sg.correction = rotaryNumber;
      memcpy ((char *)&mem[0], (char *)&sg, sizeof(sg));
      SetMemClkStatus (0, 0);
      EEPROM.put(0, mem);
      LCDErrorMsg(11, (char *)"OK     ");
      delay (3000);
      LCDClearErrorMsg(10);
    }

    flags |= MENU_MODE;
    flags &= ~PBUTTON1_PUSHED;
  }

  if (flags & PBUTTON2_PUSHED) {
    ClearFlags();
    si5351.reset();
    LCDErrorMsg(11, (char *)"       ");
    LCDSelectLine(0, 3, 1);

    flags |= MENU_MODE;
    flags &= ~PBUTTON2_PUSHED;
  }

}

void MenuClockFrequencyMode (void)
{
  long temp;
  unsigned char pos;
  pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;

  if (flags & ROTARY_CW) {
    sg.ClkFreq[ClkSelection] += frequency_inc;
    if (sg.ClkFreq[ClkSelection] > HighFrequencyLimit(ClkSelection)) {
      sg.ClkFreq[ClkSelection] = HighFrequencyLimit(ClkSelection);
      pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
      LCDSelectLine (pos, ClkSelection, 1);
    }

  } else if (flags & ROTARY_CCW) {
    temp = sg.ClkFreq[ClkSelection] - frequency_inc;
    if (temp < (long)LowFrequencyLimit(ClkSelection) || temp < 0) {
      sg.ClkFreq[ClkSelection] = LowFrequencyLimit(ClkSelection);
      pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
      LCDSelectLine (pos, ClkSelection, 1);

    } else {
      sg.ClkFreq[ClkSelection] -= frequency_inc;
    }

    if (flags & LO_FREQUENCY_MODE) {
      temp = (long)sg.ClkFreq[ClkSelection] + sg.ClkOffset[ClkSelection];
      if (temp <= 0) {
        sg.ClkFreq[ClkSelection] = (unsigned long)absl(sg.ClkOffset[ClkSelection]) + LowFrequencyLimit(ClkSelection);
      }
    }
  }

  if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
    flags &= ~ROTARY_CW;
    flags &= ~ROTARY_CCW;
    if (encoderState & 0x2) encoderState &= ~0x2;
    if (encoderState & 0x1) encoderState &= ~0x1;
    LCDDisplayLOClockFrequency (ClkSelection);
    UpdateFrequency (ClkSelection);
    LCDSelectLine (pos, ClkSelection, 1);
  }

  if (flags & ROTARY_PUSH) {
    frequency_inc *= 10;
    if (frequency_inc > MAXIMUM_FREQUENCY_MULTIPLIER) frequency_inc = MINIMUM_FREQUENCY_MULTIPLIER;
    flags &= ~ROTARY_PUSH;
    pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
    LCDSelectLine (pos, ClkSelection, 1);
  }

  if (flags & PBUTTON1_PUSHED) {
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
    flags &= ~PBUTTON1_PUSHED;
  }

  if (flags & PBUTTON2_PUSHED) {
    flags |= CLOCK_WINDOW_MODE;
    flags &= ~CLOCK_FREQUENCY_MODE;
    LCDSelectLine(0, ClkSelection, 1);
    flags &= ~PBUTTON2_PUSHED;
  }

}

void MenuIQClockFrequencyMode (void)
{
  long temp;
  unsigned char pos;
  pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;

  if (flags & ROTARY_CW) {
    sg.IQClkFreq[ClkSelection] += frequency_inc;
    if (sg.IQClkFreq[ClkSelection] > HighFrequencyLimit(ClkSelection)) {
      sg.IQClkFreq[ClkSelection] = HighFrequencyLimit(ClkSelection);
      pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
      LCDSelectLine (pos, ClkSelection, 1);
    }

  } else if (flags & ROTARY_CCW) {
    temp = sg.IQClkFreq[ClkSelection] - frequency_inc;
    if (temp < (long)LowFrequencyLimit(ClkSelection) || temp < 0) {
      sg.IQClkFreq[ClkSelection] = LowFrequencyLimit(ClkSelection);
      pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
      LCDSelectLine (pos, ClkSelection, 1);

    } else {
      sg.IQClkFreq[ClkSelection] -= frequency_inc;
    }

  }

  if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
    flags &= ~ROTARY_CW;
    flags &= ~ROTARY_CCW;
    if (encoderState & 0x2) encoderState &= ~0x2;
    if (encoderState & 0x1) encoderState &= ~0x1;
    sg.IQClkFreq[0] = sg.IQClkFreq[ClkSelection]; 
    sg.IQClkFreq[1] = sg.IQClkFreq[ClkSelection]; 
    sg.IQClkFreq[2] = sg.IQClkFreq[ClkSelection]; 
    LCDDisplayIQClockFrequency (0);
    LCDDisplayIQClockFrequency (1);
    LCDDisplayIQClockFrequency (2);
    UpdateIQFrequency (ClkSelection);
    LCDSelectLine (pos, ClkSelection, 1);
  }

  if (flags & ROTARY_PUSH) {
    frequency_inc *= 10;
    if (frequency_inc > MAXIMUM_FREQUENCY_MULTIPLIER) frequency_inc = MINIMUM_FREQUENCY_MULTIPLIER;
    flags &= ~ROTARY_PUSH;
    pos = FrequencyDigitUpdate(frequency_inc) + FREQUENCY_DISPLAY_SHIFT;
    LCDSelectLine (pos, ClkSelection, 1);
  }

  if (flags & PBUTTON1_PUSHED) {
    if (++ClkSelection >= MAXCLK) ClkSelection = 0;
    LCDSelectLine(pos, ClkSelection, 1);
    flags &= ~PBUTTON1_PUSHED;
  }

  if (flags & PBUTTON2_PUSHED) {
    ClearFlags();
    flags |= MENU_MODE;
    LCDSelectLine(0, 3, 1);
    flags &= ~PBUTTON2_PUSHED;
  }

}


void MenuClockFrequencyOffsetMode (void)
{
  long temp;
  unsigned char pos;

  pos = FrequencyDigitUpdate(offset_inc);
  pos += OFFSET_DISPLAY_SHIFT;

  if (flags & ROTARY_CW) {
    sg.ClkOffset[ClkSelection] += offset_inc;
    if (sg.ClkOffset[ClkSelection] > MAXIMUM_OFFSET_FREQUENCY) {
      sg.ClkOffset[ClkSelection] = MAXIMUM_OFFSET_FREQUENCY;
    }

  } else if (flags & ROTARY_CCW) {
    temp = sg.ClkOffset[ClkSelection] - offset_inc;
    if (temp <  (-MAXIMUM_OFFSET_FREQUENCY) ) {
      sg.ClkOffset[ClkSelection] = (-MAXIMUM_OFFSET_FREQUENCY);

      // This may be redundant but keep it for future
    } else if (sg.ClkOffset[ClkSelection] > 1000000 && temp < 1000000) {
      sg.ClkOffset[ClkSelection] -= offset_inc;

    } else {
      sg.ClkOffset[ClkSelection] -= offset_inc;
    }
  }

  if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
    flags &= ~ROTARY_CW;
    flags &= ~ROTARY_CCW;
    if (encoderState & 0x2) encoderState &= ~0x2;
    if (encoderState & 0x1) encoderState &= ~0x1;

    LCDDisplayOffsetFrequency (ClkSelection);
    pos = FrequencyDigitUpdate(offset_inc);
    pos += OFFSET_DISPLAY_SHIFT;
    LCDSelectLine (pos, ClkSelection, 1);
  }

  if (flags & ROTARY_PUSH) {
    offset_inc *= 10;
    if (offset_inc > MAXIMUM_OFFSET_MULTIPLIER) offset_inc = MINIMUM_OFFSET_MULTIPLIER;
    flags &= ~ROTARY_PUSH;
    pos = FrequencyDigitUpdate(offset_inc);
    pos += OFFSET_DISPLAY_SHIFT;
    LCDSelectLine (pos, ClkSelection, 1);
  }

  if (flags & PBUTTON1_PUSHED) {
    ClkSelection++;
    if (ClkSelection >= 3) ClkSelection = 0;
    pos = FrequencyDigitUpdate(offset_inc);
    pos += OFFSET_DISPLAY_SHIFT;
    LCDSelectLine (pos, ClkSelection, 1);
    flags &= ~PBUTTON1_PUSHED;
  }

  if (flags & PBUTTON2_PUSHED) {
    ClearFlags();
    flags |= MENU_MODE;
    LCDSelectLine(0, 3, 1);
    flags &= ~PBUTTON2_PUSHED;
  }

}


void MenuDisplayMode ()
{
  if (flags & ROTARY_CW) {
    if (++MenuSelection >= MAXMENU_ITEMS) MenuSelection = 0;

  } else if (flags & ROTARY_CCW) {
    if (!MenuSelection) MenuSelection = MAXMENU_ITEMS - 1;
    else MenuSelection--;
  }

  if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
    flags &= ~ROTARY_CW;
    flags &= ~ROTARY_CCW;
    if (encoderState & 0x2) encoderState &= ~0x2;
    if (encoderState & 0x1) encoderState &= ~0x1;

    LCDDisplayMenuOption (3);
    LCDSelectLine(0, 3, 1);
  }

  if (flags & ROTARY_PUSH) {
    DoMenu();
    flags &= ~ROTARY_PUSH;
  }

  if (flags & PBUTTON1_PUSHED) {
    DoMenu();
    flags &= ~PBUTTON1_PUSHED;
  }

  if (flags & PBUTTON2_PUSHED) {
    DoMenu();
    flags &= ~PBUTTON2_PUSHED;
  }

}

void MenuClockWindowMode ()
{
  unsigned char pos;


  if (flags & ROTARY_CW) {
    if (++ClkSelection >= MAXCLK) ClkSelection = 0;

  } else if (flags & ROTARY_CCW) {
    if (!ClkSelection) ClkSelection = MAXCLK - 1;
    else ClkSelection--;
  }

  if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
    flags &= ~ROTARY_CW;
    flags &= ~ROTARY_CCW;
    if (encoderState & 0x2) encoderState &= ~0x2;
    if (encoderState & 0x1) encoderState &= ~0x1;

    LCDSelectLine(0, ClkSelection, 1);
  }

  if (flags & ROTARY_PUSH) {
    flags &= ~ROTARY_PUSH;
  }

  if (flags & PBUTTON1_PUSHED) {
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
      flags &= ~CLOCK_WINDOW_MODE;
      flags |= CLOCK_FREQUENCY_MODE;
    }
    flags &= ~PBUTTON1_PUSHED;
  }

  if (flags & PBUTTON2_PUSHED) {
    LCDSelectLine(0, ClkSelection, 0);
    ClearFlags();
    flags |= MENU_MODE;
    LCDSelectLine(0, 3, 1);
    flags &= ~PBUTTON2_PUSHED;
  }

}


void DoMenu(void)
{
  unsigned char pos;

  switch (MenuSelection) {
    case VFO_ENABLE:
      sg.ClkMode[0] = VFO_CLK_MODE;
      sg.ClkMode[1] = VFO_CLK_MODE;
      sg.ClkMode[2] = VFO_CLK_MODE;
      
      ClearFlags();
      flags |= CLOCK_WINDOW_MODE;
      
      LCDClearClockWindow();
      LCDDisplayClockFrequency(0);
      LCDDisplayClockFrequency(1);
      LCDDisplayClockFrequency(2);
      LCDDisplayClockMode (0);
      LCDDisplayClockMode (1);
      LCDDisplayClockMode (2);
      LCDDisplayClockStatus(0);
      LCDDisplayClockStatus(1);
      LCDDisplayClockStatus(2);
      ClkSelection = 0;
      LCDSelectLine(0, ClkSelection, 1);
      SetupClocks();
      break;

    case LO_ENABLE:
      sg.ClkMode[0] = VFO_CLK_MODE;
      sg.ClkMode[1] = VFO_CLK_MODE;
      sg.ClkMode[2] = VFO_CLK_MODE;
      if (sg.ClkOffset[0]) sg.ClkMode[0] = LO_CLK_MODE;
      if (sg.ClkOffset[1]) sg.ClkMode[1] = LO_CLK_MODE;
      if (sg.ClkOffset[2]) sg.ClkMode[2] = LO_CLK_MODE;
      
      ClearFlags();
      flags |= CLOCK_WINDOW_MODE;
      flags |= LO_FREQUENCY_MODE;

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
      SetupClocks();
      break;

    case IQ_ENABLE:
      sg.ClkMode[0] = IQ_CLK_MODE;
      sg.ClkMode[1] = IQ_CLK_MODE;
      sg.ClkMode[2] = IQ_CLK_MODE;
      sg.ClkStatus[0] = 1;
      sg.ClkStatus[1] = 0;
      sg.ClkStatus[2] = 1;
      
      ClearFlags();
      flags |= IQ_FREQUENCY_MODE;
      
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
      
      SetupClocks();
      UpdateIQFrequency (ClkSelection);
      break;

    case SET_OFFSET:
      si5351.reset();
      SetMemClkStatus (0, 0);
      
      ClearFlags();
      flags |= OFFSET_FREQUENCY_MODE;

      LCDClearClockWindow();
      LCDDisplayOffsetFrequency (0);
      LCDDisplayOffsetFrequency (1);
      LCDDisplayOffsetFrequency (2);
      
      ClkSelection = 0;
      pos = FrequencyDigitUpdate(offset_inc);
      pos += OFFSET_DISPLAY_SHIFT;
      LCDSelectLine (pos, ClkSelection, 1);
      
      SetupClocks();
      break;

    case CALIBRATE:
      SetMemClkStatus (1, 0);
      sg.ClkMode[0] = VFO_CLK_MODE;
      sg.ClkMode[1] = VFO_CLK_MODE;
      sg.ClkMode[2] = VFO_CLK_MODE;
      
      ClearFlags();
      flags |= CALIBRATION_MODE;
      
      LCDClearClockWindow();
      LCDDisplayClockFrequency(0);
      LCDDisplayClockFrequency(1);
      LCDDisplayClockFrequency(2);
      LCDDisplayClockMode (0);
      LCDDisplayClockMode (1);
      LCDDisplayClockMode (2);
      LCDDisplayClockStatus(0);
      LCDDisplayClockStatus(1);
      LCDDisplayClockStatus(2);
      
      SetupClocks();
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
      si5351.reset();
      SetMemClkStatus (0, 0);
      
      ClearFlags();
      flags |= MEMORY_SAVE_MODE;
      
      rotaryNumber = 1;
      rotaryInc = 1;
      LCDDisplayNumber1D (rotaryNumber, 11, 3);
      LCDSelectLine (11, 3, 1);
      break;

    case RECALL:
      si5351.reset();
      SetMemClkStatus (0, 0);
      
      ClearFlags();
      flags |= MEMORY_RECALL_MODE;
      
      rotaryNumber = 1;
      rotaryInc = 1;
      LCDDisplayNumber1D (rotaryNumber, 11, 3);
      LCDSelectLine (11, 3, 1);
      break;

    case RESET:
      Reset();
      return;
      break;
  }


}

void ClearFlags ()
{
  flags &= ~MEMORY_RECALL_MODE;
  flags &= ~MEMORY_SAVE_MODE;
  flags &= ~CLOCK_WINDOW_MODE;
  flags &= ~MENU_MODE;
  flags &= ~OFFSET_FREQUENCY_MODE;
  flags &= ~CLOCK_FREQUENCY_MODE;
  flags &= ~LO_FREQUENCY_MODE;
  flags &= ~IQ_FREQUENCY_MODE;
  
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

  switch (line) {
    case 0:
    case 1:
    case 2:
      if (flags & IQ_FREQUENCY_MODE) return IQ_HIGH_FREQUENCY_LIMIT;
      else return DEFAULT_HIGH_FREQUENCY_LIMIT; 
      break;
  }

  return DEFAULT_HIGH_FREQUENCY_LIMIT;

}

unsigned long LowFrequencyLimit (unsigned char line)
{
  switch (line) {
    case 0:
    case 1:
    case 2:
      if (flags & IQ_FREQUENCY_MODE) return IQ_LOW_FREQUENCY_LIMIT;
      else return DEFAULT_LOW_FREQUENCY_LIMIT; 
      break;
  }

  return DEFAULT_LOW_FREQUENCY_LIMIT;

}

void UpdateIQFrequency (unsigned char line)
{
  int mult;

  if (sg.IQClkFreq[line] < IQ_LOW_FREQUENCY_LIMIT) sg.IQClkFreq[line] = IQ_LOW_FREQUENCY_LIMIT;
  if (sg.IQClkFreq[line] > IQ_HIGH_FREQUENCY_LIMIT) sg.IQClkFreq[line] = IQ_HIGH_FREQUENCY_LIMIT;
  
  mult = GetPLLFreq (sg.IQClkFreq[line]);
  if (mult) {
    pllfreqll = (unsigned long long)sg.IQClkFreq[line] * (unsigned long long)mult * 100ULL;
    freqll = (unsigned long long)sg.IQClkFreq[line] * 100ULL;      
  } 
      
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);   // redundant defaults to PLLA
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.set_ms_source(SI5351_CLK2, SI5351_PLLA);   // redundant defaults to PLLA
  
  si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK0);
  si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK2);

  si5351.set_phase(SI5351_CLK0, 0);
  si5351.set_phase(SI5351_CLK2, (unsigned char)mult);

  // We need to reset the PLL before they will be in phase alignment
  si5351.pll_reset(SI5351_PLLA);
  
}

void SetMemClkStatus (unsigned char stat, unsigned char index) 
{
    mem[index].ClkStatus[0] = mem[index].ClkStatus[1] = mem[index].ClkStatus[2] = stat;
}

void SetupClocks (void)
{
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);   
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);   
  si5351.set_ms_source(SI5351_CLK2, SI5351_PLLA);

  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);
}

void UpdateFrequency (unsigned char line)
{
  unsigned long freq;

  freq = sg.ClkFreq[line];

  if (freq < LowFrequencyLimit(line)) freq = LowFrequencyLimit(line);
  if (freq > HighFrequencyLimit(line)) freq = HighFrequencyLimit(line);
  
  pllfreqll = (unsigned long long)SI5351_PLL_VCO_MAX  * 100ULL;
  freqll = (unsigned long long)freq * 100ULL;

  switch (line) {
    case 0:
      si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK0);
      if (!(flags & PLLA_RUNNING)) {
        flags |= PLLA_RUNNING;
      }
      break;

    case 1:
      si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK1);
//      if (frequency_clk >= 150000000 && freq < 150000000) {
//        si5351.output_enable(SI5351_CLK1, 0);
//        si5351.pll_reset(SI5351_PLLB);
//      }
      flags |= PLLB_RUNNING;
      frequency_clk = freq;
      break;

    case 2:
      si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK2);
      if (!(flags & PLLA_RUNNING)) {
        flags |= PLLA_RUNNING;
      }
      break;
      
    case 128:
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
      si5351.output_enable(SI5351_CLK0, 0);
      break;
      
    case 1:
      si5351.output_enable(SI5351_CLK1, 0);
      break;
      
    case 2:
      si5351.output_enable(SI5351_CLK2, 0);
      break;
      
  }
}


void Reset (void)
{
  
#ifndef REMOVE_CLI
  Serial.print (header1);
  Serial.println (header2);
  Serial.write (prompt);
  Serial.flush();
#endif // REMOVE_CLI

  si5351.reset();

  frequency_clk = DEFAULT_FREQUENCY;
  frequency_inc = DEFAULT_FREQUENCY_INCREMENT;
  offset_inc = DEFAULT_FREQUENCY_INCREMENT;
  calibration_mult = DEFAULT_CALIBRATION_INCREMENT;

  // Read sg from EEPROM
  memset ((char *)&sg, 0, sizeof (sg));
  memset ((char *)&mem, 0, sizeof (mem));
  EEPROM.get(0, mem);
  memcpy ((char *)&sg, (char *)&mem[0], sizeof(sg));

  if (sg.flags != 0xFEEDFACE) {
    sg.flags = 0xFEEDFACE;
    sg.ClkFreq[0] = 1000000;
    sg.ClkFreq[1] = 100000000;
    sg.ClkFreq[2] = 8000;
    sg.IQClkFreq[0] = 5000000;
    sg.IQClkFreq[1] = 5000000;
    sg.IQClkFreq[2] = 5000000;
    sg.ClkOffset[0] = 1000000;
    sg.ClkOffset[1] = 100000000;
    sg.ClkOffset[2] = 8000;
    sg.ClkMode[0] = sg.ClkMode[1] = sg.ClkMode[2] = VFO_CLK_MODE;
    sg.ClkStatus[0] = sg.ClkStatus[1] = sg.ClkStatus[2] = 0;
    sg.correction = 0;
    memcpy ((char *)&mem[0], (char *)&sg, sizeof(sg));
    EEPROM.put(0, mem);
  }

  encoderVal = 0xFF;
  encoderState = 0;
  pbstate = 0;
  pbreset = 0;
  old_AB = 0;

  rotaryNumber = 0;
  rotaryInc = 1;


  // Push Button Debounce
  pb1state = pb2state = HIGH;             // the current state of the push button
  oldpb1state = oldpb2state = HIGH;       // the last reading from the push button
  pb1time = pb2time = 0;                  // the last time the push button state changed

  // LCD Menu
  LCDDisplayHeader();

  MenuSelection = 0;
  ClkSelection = 0;

  LCDDisplayClockEntry(0);
  LCDDisplayClockEntry(1);
  LCDDisplayClockEntry(2);
  LCDDisplayMenuOption(3);
  LCDSelectLine(0, 3, 1);


  flags = MENU_MODE;
}

long absl (long v)
{
  if (v < 0) return (-v);
  else return v;
}






unsigned int GetPLLFreq(unsigned long freq) 
{ 
    unsigned long long pfreq;
    unsigned int i;

    for (i = 10; i <= 200; i = i + 2) {
        pfreq = (unsigned long long) freq *  (unsigned long long)i;
        if (pfreq >= SI5351_PLL_VCO_MIN && pfreq <= SI5351_PLL_VCO_MAX) {
          return i;
        }
    } 
    return 0;
}



#ifndef REMOVE_CLI

// Place program specific content here
void ExecuteSerial (char *str)
{
  // num defined the actual number of entries process from the serial buffer
  // i is a generic counter
  unsigned char num;
  unsigned long i;

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
    case 'C':             // Calibrate
      break;


    // This command is used to quickly set the frequency. The function will decide the
    // best PLL frequecy to use. It also assumes that CLK0 and CLK1 will share PLLA and
    // CLK2 will use PLLB. Phase is set to 0 and 8mA output is used
    // Syntax: F [CLK] [FREQ], where CLK is output clock, FREQ is output frequency between 8Khz to 160Mhz
    // Note: If you select a frequency greater than 150 Mhz, you cannot select a fequency below this without resetting the Si5351
    case 'F':             // Set Frequency
      // Validate inputs
      if (numbers [0] > 2) {
        Serial.println ("Bad Clk");
        break;
      }

      if (numbers[1] < SI5351_CLKOUT_MIN_FREQ || numbers[1] > SI5351_CLKOUT_MAX_FREQ) {
        Serial.println ("Ban Freq");
        break;
      } else {
        freqll = (unsigned long long)numbers[1] * 100ULL;
      }
      
      if (numbers[0] != 2) Serial.println ("CLK1 & CLK2 Share PLLB");
      else Serial.println ("CLK2 Uses PLLB");

      pllfreqll = (unsigned long long)SI5351_PLL_VCO_MAX * 100ULL;
      SetupClocks();

      // set frequency
      if (numbers[0] == 0) {
        if (commands[1] == 'B') {
          si5351.set_ms_source(SI5351_CLK0, SI5351_PLLB);
          si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK0);
        } else {
          si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
          si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK0);
        }
        Serial.println (si5351.si5351_read(SI5351_CLK0_CTRL), HEX);

      } else if (numbers[0] == 1) {
        if (commands[1] == 'B') {
          si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
          si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK1);
        } else {
          si5351.set_ms_source(SI5351_CLK1, SI5351_PLLA);
          si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK1);
        }

      } else if (numbers[0] == 2) {
        if (commands[1] == 'B') {
          si5351.set_ms_source(SI5351_CLK2, SI5351_PLLB);
          si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK2);
        } else {
          si5351.set_ms_source(SI5351_CLK2, SI5351_PLLA);
          si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK2);
        }

      }
      break;

    // Help Screen. This consumes a ton of memory but necessary for those
    // without much computer or programming experience.
    case 'H':             // Help
      Serial.println (errmsg);
      break;

    // Phase controls
    case 'P': 
      unsigned int mult;
      if (numbers[0] < SI5351_CLKOUT_MIN_FREQ || numbers[0] > SI5351_CLKOUT_MAX_FREQ) {
        Serial.println ("Ban Freq");
        break;
      } 
      
      mult = GetPLLFreq (numbers[0]);
      if (mult) {
        pllfreqll = (unsigned long long)numbers[0] * (unsigned long long)mult * 100ULL;
        freqll = (unsigned long long)numbers[0] * 100ULL;      
      } else {
        Serial.println ("Freq too high");
        break;
      }
      SetupClocks();
  
      si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK0);
      si5351.set_freq_manual(freqll, pllfreqll, SI5351_CLK2);

      si5351.set_phase(SI5351_CLK0, 0);
      si5351.set_phase(SI5351_CLK2, (unsigned char)mult);

      // We need to reset the PLL before they will be in phase alignment
      si5351.pll_reset(SI5351_PLLA);
      break;


    // This command reset the Si5351.  A reset zeros all parameters including the correction/calibration value
    // Therefore the calibration must be re-read from eeprom
    case 'R':             // Reset
      Reset();
      break;

    // This command initialized the eeprom
    case 'I':             // Reset
      flags |= DISABLE_BUTTONS;
      memset ((char *)&mem, 0, sizeof (mem));
      EEPROM.put(0, mem);
      flags &= ~DISABLE_BUTTONS;
      break;

    case 'T':
      break;

    case 'X':             // Reset
      printMem(0);
      printMem(1);
      printMem(2);
      printMem(3);
      break;


    // If an undefined command is entered, display an error message
    default:
      ErrorOut ();
  }

}

void printMem (unsigned char i) 
{
  
  EEPROM.get(0, mem);
  Serial.print ("\r\nmem: ");
  Serial.println (i);
  Serial.println (mem[i].flags);
  Serial.println (mem[i].ClkFreq[0]);
  Serial.println (mem[i].ClkFreq[1]);
  Serial.println (mem[i].ClkFreq[2]);
  Serial.println (mem[i].IQClkFreq[0]);
  Serial.println (mem[i].IQClkFreq[1]);
  Serial.println (mem[i].IQClkFreq[2]);
  Serial.println (mem[i].ClkOffset[0]);
  Serial.println (mem[i].ClkOffset[1]);
  Serial.println (mem[i].ClkOffset[2]);
  Serial.println (mem[i].ClkMode[0]);
  Serial.println (mem[i].ClkMode[1]);
  Serial.println (mem[i].ClkMode[2]);
  Serial.println (mem[i].ClkStatus[0]);
  Serial.println (mem[i].ClkStatus[1]);
  Serial.println (mem[i].ClkStatus[2]);
  Serial.println (mem[i].correction);
  
}

#endif // REMOVE_CLI




