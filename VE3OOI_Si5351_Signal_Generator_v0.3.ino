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
#include "PSK.h"                              // VE3OOI routines for tranmitting PSK
#include "LCD.h"
#include "Encoder.h"
#include "Timer.h"


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


volatile unsigned long flags;


// Encoder Variables
volatile int old_AB = 0;
volatile unsigned char encoderVal, encoderState;
volatile unsigned int pbstate;
volatile unsigned char pbfunction;
volatile unsigned long pbreset;

// Push Button Debounce
volatile unsigned char pb1ctr, pbenable, pbpushed;
volatile unsigned int pb1rctr;

// Frequency Control variables
volatile unsigned long frequency_clk, PSKCarrierFrequency;
volatile long frequency_inc;
volatile unsigned long frequency_mult;
volatile unsigned long frequency_mult_old;

// Local PSK Variables
boolean pskChanged, pskLocked;
unsigned char pskResetCtr;
boolean decodePhaseChange;

// PSK Transmitter Variables
unsigned char pskSwap, pskVcodeLen;
unsigned int pskVcode;

// RTTY Transmitter Variables
volatile unsigned long rttyTransmitSpaceFreq;
volatile unsigned long rttyTransmitMarkFreq;

volatile unsigned char rttyPriorState, rttyFigures;
volatile char rttyLTRSSwitch;

// LCD Menu
volatile unsigned char MenuSelection;

char RootMenuOptions[MAXMENU_ITEMS][MAXMENU_LEN] = {
  {"Freq CLK0\0"},
  {"Freq CLK1\0"},
  {"Freq CLK2\0"},
  {"RTY 1 CK2\0"},
  {"RTY 2 CK2\0"},
  {"PSK 1 CK2\0"},
  {"PSK 2 CK2\0"},
  {"SW 80 CK2\0"},
  {"SW 40 CK2\0"},
  {"SW 20 CK2\0"},
  {"RESET\0"}
};


// This defines the various parameter used to program Si5351 (See Silicon Labs AN619 Note)
// multisynch defines specific parameters used to determine Si5351 registers
// clk0ctl, clk1ctl, clk2ctl defined specific parameters used to control each clock
extern Si5351_def multisynth;
extern Si5351_CLK_def clk0ctl;
extern Si5351_CLK_def clk1ctl;
extern Si5351_CLK_def clk2ctl;


char message1[22] = {'K', 'C', '2', 'T', 'X', 'O', ' ', 'D', 'E', ' ', 'V', 'E', '3', 'O', 'O', 'I', ' ', 's', 'k', 0xA, 0xD, 0x0};
char message0A[29] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                      'Y', 'Z', 0xA, 0xD, 0x0
                     };
char message0N[27] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ' ', '$', ',', '!', ':', '(', '\"', ')', '#', '?', '&', '.', '/', ';', 0xA, 0xD, 0x0};

char header1[14] = {'V', 'E', '3', 'O', 'O', 'I', ' ', 'S', 'i', 'g', 'n', 'a', 'l', 0x0};
char header2[14] = {'G', 'e', 'n', 'e', 'r', 'a', 't', 'o', 'r', ' ', 'v', '.', '2', 0x0};


char prompt[6] = {0xa, 0xd, ':', '>', ' ', 0x0};
char ovflmsg[9] = {'O', 'v', 'e', 'r', 'f', 'l', 'o', 'w', 0x0};
char errmsg[4] = {'E', 'r', 'r', 0x0};


// the setup function runs once when you press reset or power the board
void setup() {

  // Setup Encoder Pins
  // Setup encoder pins as inputs with pullups
  pinMode(ENC_A, INPUT);            // Rotary A: set at input
  digitalWrite(ENC_A, HIGH);        // Enable weak pullups (i.e. grounded when engaged)
  pinMode(ENC_B, INPUT);            // Rotary A: set at input
  digitalWrite(ENC_B, HIGH);        // Enable weak pullups (i.e. grounded when engaged)
  pinMode(ENC_PB, INPUT);           // Rotary Push button
  digitalWrite(ENC_PB, HIGH);       // Enable weak pullups (i.e. grounded when engaged)

  // Push Buttons
  pinMode(PBUTTON1, INPUT);         // Push buttons are input

  // define the baud rate for TTY communications. Note CR and LF must be sent by  terminal program
  Serial.begin(115200);
  Serial.print (header1);
  Serial.println (header2);
  Serial.write (prompt);
  Serial.flush();
  ResetSerial ();


  //  initialize the Si5351
  ResetSi5351 (SI_CRY_LOAD_8PF);

  // Read XTAl correction value from Arduino eeprom memory
  EEPROMReadCorrection();

  EnableTimers (1, TIMER3MS);         // Timer 1 is for Rotary & Pushbutton

  // This is used for testing.  Can toggle this pins to measure timing using scope
  // Pin 13 is PB5, which uses data direction register DDB5
  DDRB = (1 << DDB5);

  Reset ();

}



// the loop function runs over and over again forever
void loop() 
{


  // Look for characters entered from the keyboard and process them
  // This function is part of the UART package.
  ProcessSerial ();

  if (flags & MASTER_RESET) {
    Reset();
  }

  if (flags & OFFLINE_MODE) {
    if (flags & ROTARY_CW) {
      if (++MenuSelection >= MAXMENU_ITEMS) MenuSelection = MAXMENU_ITEMS - 1;
      
    } else if (flags & ROTARY_CCW) {
      if (MenuSelection) MenuSelection--;
    }

    if ( (flags & ROTARY_CW) || (flags & ROTARY_CCW) ) {
      LCDDisplayMode ();
      flags &= ~ROTARY_CW;
      flags &= ~ROTARY_CCW;
    }
    
    if (flags & ROTARY_PUSH) {
      flags |= SWITCH_MODE;
      flags &= ~ROTARY_PUSH;
    }

    if (IsPushed ()) {
      flags |= SWITCH_MODE;
      EnablePushButtons();
    }
    
    if (encoderState & 0x2) encoderState &= ~0x2;
    if (encoderState & 0x1) encoderState &= ~0x1;
    
  } else if (flags & ONLINE_MODE) {
    if (IsPushed ()) {
      flags |= SWITCH_MODE;
      EnablePushButtons();
    }
    if (encoderState) {
      UpdateFrequencyData ();
      if (MenuSelection == Freq_CLK0) {
        SetFrequency (SI_CLK0, SI_PLL_A, frequency_clk, SI_CLK_8MA);
      } else if (MenuSelection == Freq_CLK1) {
        SetFrequency (SI_CLK1, SI_PLL_A, frequency_clk, SI_CLK_8MA);
      } else if (MenuSelection == Freq_CLK2) {
        SetFrequency (SI_CLK2, SI_PLL_B, frequency_clk, SI_CLK_8MA);
      } 
    }
  }

  if (flags & SWITCH_MODE) {
    if (flags & OFFLINE_MODE) {
      flags |= ONLINE_MODE;
      flags &= ~OFFLINE_MODE;
      DoMenu(1);
    } else if (flags & ONLINE_MODE) {
      flags |= OFFLINE_MODE;
      flags &= ~ONLINE_MODE;
      DoMenu(0);
    }
    flags &= ~SWITCH_MODE;
  }

}

void DoMenu(unsigned char option) 
{
  
  if (!option) {                // Disable and go into offline mode
    DisableSi5351Clocks();
    LCDDisplayStatus (0);
    LCDSwitchFunction (0);
    return;
  }

  LCDDisplayStatus (1);
  LCDSwitchFunction (1);

  switch (MenuSelection) {
    case Freq_CLK0:
      SetFrequency (SI_CLK0, SI_PLL_A, frequency_clk, SI_CLK_8MA);
      break;

    case Freq_CLK1:
      SetFrequency (SI_CLK1, SI_PLL_A, frequency_clk, SI_CLK_8MA);
      break;
      
    case Freq_CLK2:
      SetFrequency (SI_CLK2, SI_PLL_B, frequency_clk, SI_CLK_8MA);
      break;

    case RTY_1_CLK2:
      SendRTTYTestMsg (frequency_clk, 1);
      break;
      
    case RTY_2_CLK2:
      SendRTTYTestMsg (frequency_clk, 2);
      break;
      
    case PSK_1_CLK2:
      SendPSKTestMsg (frequency_clk, 1);
      break;

    case PSK_2_CLK2:
      SendPSKTestMsg (frequency_clk, 2);
      break;

    case SW_80_CLK2:
      if (frequency_mult < 1000) frequency_mult = 1000;
      frequency_clk = 3500000;
      LCDDisplayFrequency ();
      LCDDisplayFrequencyIncrement();
      ScanFrequencies (frequency_clk, 4000000, frequency_mult, 0);
      break;

    case SW_40_CLK2:
      if (frequency_mult < 1000) frequency_mult = 1000;
      frequency_clk = 7000000;
      LCDDisplayFrequency ();
      LCDDisplayFrequencyIncrement();
      ScanFrequencies (frequency_clk, 7300000, frequency_mult, 0);
      break;

    case SW_20_CLK2:
      if (frequency_mult < 1000) frequency_mult = 1000;
      frequency_clk = 14000000;
      LCDDisplayFrequency ();
      LCDDisplayFrequencyIncrement();
      ScanFrequencies (frequency_clk, 14350000, frequency_mult, 0);
      break;

    case Do_Reset:
      break;  
  }
  if (MenuSelection != Freq_CLK0 && MenuSelection != Freq_CLK1 && MenuSelection != Freq_CLK2) {
    flags |= OFFLINE_MODE;
    flags &= ~ONLINE_MODE;
    LCDDisplayStatus (0);
    LCDSwitchFunction (0);
  }
}

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

      // First, Check inputs to validate
      if (numbers[0] == 0 && numbers[1] == 0) {
        EEPROMReadCorrection();
        Serial.print ("C: ");
        Serial.println (multisynth.correction);
        break;
      } else if (numbers[0] == 0 || numbers[1] < SI_MIN_OUT_FREQ || numbers[1] > SI_MAX_OUT_FREQ) {
        Serial.println (errmsg);
        break;
      }

      // New value defined so read the old values and display what will be done
      EEPROMReadCorrection();
      Serial.println (multisynth.correction);
      Serial.print ("=>");
      Serial.println (numbers[0]);

      // Store the new value entered, reset the Si5351 and then display frequency based on new setting
      multisynth.correction = numbers[0];
      EEPROMWriteCorrection();

      ResetSi5351 (SI_CRY_LOAD_8PF);
      EEPROMReadCorrection();
      SetFrequency (SI_CLK0, SI_PLL_B, numbers[1], SI_CLK_8MA);
      SetFrequency (SI_CLK1, SI_PLL_B, numbers[1], SI_CLK_8MA);
      SetFrequency (SI_CLK2, SI_PLL_B, numbers[1], SI_CLK_8MA);
      break;


    // This command is used to quickly set the frequency. The function will decide the
    // best PLL frequecy to use. It also assumes that CLK0 and CLK1 will share PLLA and
    // CLK2 will use PLLB. Phase is set to 0 and 8mA output is used
    // Syntax: F [CLK] [FREQ], where CLK is output clock, FREQ is output frequency between 8Khz to 160Mhz
    // Note: If you select a frequency greater than 150 Mhz, you cannot select a fequency below this without resetting the Si5351
    case 'F':             // Set Frequency
      // Validate inputs
      if (numbers [0] > 2 || numbers[1] < SI_MIN_OUT_FREQ || numbers[1] > SI_MAX_OUT_FREQ) {
        Serial.println (errmsg);
        break;
      }
//      numbers[1] += TX_FREQUENCY_OFFSET;

      // Make assumtions and set frequency
      if (numbers[0] == 0) {
        SetFrequency (SI_CLK0, SI_PLL_A, numbers[1], SI_CLK_8MA);
        MenuSelection = Freq_CLK0;
      } else if (numbers[0] == 1) {
        SetFrequency (SI_CLK1, SI_PLL_A, numbers[1], SI_CLK_8MA);
        MenuSelection = Freq_CLK1;
      } else if (numbers[0] == 2) {
        SetFrequency (SI_CLK2, SI_PLL_B, numbers[1], SI_CLK_8MA);
        MenuSelection = Freq_CLK2;
      }

      frequency_clk = numbers[1];
      LCDDisplayFrequency ();
      LCDDisplayMode();
      LCDDisplayStatus (1);
      LCDSwitchFunction (1);
      break;

    // Help Screen. This consumes a ton of memory but necessary for those
    // without much computer or programming experience.
    case 'H':             // Help
      Serial.println (errmsg);
      break;


    // This command reset the Si5351.  A reset zeros all parameters including the correction/calibration value
    // Therefore the calibration must be re-read from eeprom
    case 'R':             // Reset
      ResetSi5351 (SI_CRY_LOAD_8PF);
      EEPROMReadCorrection();
      Reset();
      break;

    // This command is used to sweep between two frequencies.
    // Syntax: S [START] [STOP] {INC] [DELAY] where START is the starting frequency, stop is the ending frequency
    // INC is the amount to increment the frequency by and [DELAY] is the pause in ms before changing frequeny
    // E.g. S 1000000 10000000 1000000 2000 , Sweep from 1 Mhz to 10 Mhz and increment by 1 Mhz.  Pause for 2 seconds before changing frequncy
    // The output is fixed to PLLA and all clocks display the frequencies
    case 'S':             // Scan Frequencies
      // Validate the inputs
      if (numbers[0] < SI_MIN_OUT_FREQ || numbers[0] > SI_MAX_OUT_FREQ || numbers[1] < SI_MIN_OUT_FREQ ||
          numbers[1] > SI_MAX_OUT_FREQ || numbers[2] == 0 || numbers[2] > numbers[1]) {
        Serial.println (errmsg);
        break;
      }

      if (numbers[3] == 0 || numbers[3] > 3000) {
        numbers[3] = 500;
      }
      Serial.print (numbers[0]);
      Serial.print ("=>");
      Serial.print (numbers[1]);
      Serial.print ("@");
      Serial.println (numbers[2]);
      Serial.flush();

      MenuSelection = Freq_CLK0;
      LCDDisplayMode();
      
      frequency_mult = numbers[2];
      LCDDisplayFrequencyIncrement(); 
           
      LCDDisplayStatus (1);
      LCDSwitchFunction (1);

      for (i = numbers[0]; i <= numbers[1]; i += numbers[2]) {
        SetFrequency (SI_CLK0, SI_PLL_A, i, 8);
        SetFrequency (SI_CLK1, SI_PLL_A, i, 8);
        SetFrequency (SI_CLK2, SI_PLL_A, i, 8);
        Serial.println (i);
        frequency_clk = i;
        LCDDisplayFrequency ();
        delay (numbers[3]);
      }
      
      LCDDisplayStatus (0);
      LCDSwitchFunction (0);
      break;

    // This command sends psk signal as a test.
    case 'T':             // PSK Test
      // Validate frequency
      if (numbers[0] < SI_MIN_OUT_FREQ || numbers[0] > SI_MAX_OUT_FREQ || numbers[1] > MAX_MESSAGES) {
        Serial.println (errmsg);
        break;
      }

      LCDDisplayStatus (1);
      LCDSwitchFunction (1);
      
      if (commands[1] == 'R') {
        MenuSelection = RTY_1_CLK2;
        LCDDisplayMode();
        frequency_clk = numbers[0];
        LCDDisplayFrequency ();
        SendRTTYTestMsg (numbers[0], (unsigned char)numbers[1]);
        
      } else if (commands[1] == 'P') {
        MenuSelection = PSK_1_CLK2;
        LCDDisplayMode();
        frequency_clk = numbers[0];
        LCDDisplayFrequency ();
        SendPSKTestMsg (numbers[0], (unsigned char)numbers[1]);
      } else {
        Serial.println (errmsg);
        break;
      }

      DisableSi5351Clocks();
      LCDDisplayStatus (0);
      LCDSwitchFunction (0);
      break;

    case 'X':             // Timing
      break;



    // If an undefined command is entered, display an error message
    default:
      ErrorOut ();
  }

}

void ScanFrequencies (unsigned long fstart, unsigned long fend, unsigned long finc, unsigned char cons)
{
  
  for (frequency_clk = fstart; frequency_clk <= fend; frequency_clk += finc) {
    SetFrequency (SI_CLK2, SI_PLL_B, frequency_clk, 8);
    if (cons) Serial.println (frequency_clk);
    else LCDDisplayFrequency ();
    delay (SWEEP_DELAY);
  }
  DisableSi5351Clocks();
 
}

void UpdateFrequencyData (void)
{

  // encoderState is used to signal if the encoder pushbutton was engaged to change the frequency
  // increment. That is the increment/decrement to apply to the frequency when to encoder is rotated
  // Because the LCD is slow, this caused PSK/RTTY character to be dropped or misintrepreted.
  // Arduino just does not have the horsepower
  if (encoderState & 0x1) {
    LCDDisplayFrequencyIncrement ();        // Update LCD with increment
    encoderState &= ~0x1;                   // Clear the signal
  }

  if (encoderState & 0x2) {
    // Encoder was rotated to update the various frequencies used to Rx and Tx
    rttyTransmitSpaceFreq = frequency_clk + RTTY_SHIFT_FREQUENCY + TX_FREQUENCY_OFFSET;
    rttyTransmitMarkFreq = frequency_clk + TX_FREQUENCY_OFFSET;
    PSKCarrierFrequency = frequency_clk + TX_FREQUENCY_OFFSET;

    // If the receiver or the waterfall is running then change frequency
    // Updating the frequency takes some time (calculating Si5351 dividers and I2C communications) and
    // will cause RTTY/PSK decode errors. Arduino horsepower thing....

    // Finally update the frequency on the LCD and clear the signal
    LCDDisplayFrequency ();
    encoderState &= ~0x2;
  }

}

void Reset (void)
{

  DisableSi5351Clocks();
  
  frequency_clk = DEFAULT_FREQUENCY;
  PSKCarrierFrequency = DEFAULT_FREQUENCY + TX_FREQUENCY_OFFSET;
  frequency_inc = DEFAULT_FREQUENCY_INCREMENT;
  frequency_mult = DEFAULT_FREQUENCY_MULTIPLIER;
  frequency_mult_old = frequency_mult;

  flags = OFFLINE_MODE;

  encoderVal = 0xFF;
  encoderState = 0;
  pbstate = 0;
  pbreset = 0;
  old_AB = 0;
  pbfunction = 0;

  // Push Button Debounce
  pb1ctr = pb1rctr = pbenable = pbpushed = 0;

  // LCD Menu
  MenuSelection = 0;

  SetupLCD ();
  delay (3000);
  LCDDisplayMode();
  LCDDisplayFrequencyIncrement();
  LCDDisplayFrequency();
  LCDDisplayStatus (0);
  LCDSwitchFunction (0);  
}

// This routines are NOT part of the Si5351 and should not be included as part of the Si5351 routines.
// Note that some arduino do not have eeprom and would generate an error during compile.
// If you plan to use a Arduino without eeprom then you need to hard code a calibration value.
void EEPROMWriteCorrection(void)
// write the calibration value to Arduino eeprom
{
  eeprom_write_dword((uint32_t*)0, multisynth.correction);
}

void EEPROMReadCorrection(void)
// read the calibratio value from Arduino eeprom
{
  multisynth.correction = eeprom_read_dword((const uint32_t*)0);
}


