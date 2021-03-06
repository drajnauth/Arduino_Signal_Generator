#ifndef _MAIN_H_
#define _MAIN_H_

#define REMOVE_CLI
#define ENABLE_SWAP_VFO

#define MEM_ID 0xFEEFFACE
#define VERSION 0xA1E

typedef struct {
  unsigned long flags;
  unsigned long ClkFreq[3];
  unsigned long IQClkFreq[3];
  long ClkOffset[3];
  unsigned char ClkMode[3];
  unsigned char ClkStatus[3];
  int correction;   // can be + or -
} Sig_Gen_Struct;

// Mode Specific Flags
#define VFO_CLK_MODE 0x1
#define LO_CLK_MODE 0x2
#define IQ_CLK_MODE 0x4

// General Flags
#define BFLAGS 1
#define MFLAGS 0

#define MENU_MODE             0x01
#define CLOCK_WINDOW_MODE     0x02
#define CLOCK_FREQUENCY_MODE  0x04
#define LO_FREQUENCY_MODE     0x08
#define IQ_FREQUENCY_MODE     0x10
#define OFFSET_FREQUENCY_MODE 0x20
#define CALIBRATION_MODE      0x40
#define MEMORY_SAVE_MODE      0x80
#define MEMORY_RECALL_MODE    0x100
#define CLI_MODE              0x200

#define ROTARY_CW             0x01
#define ROTARY_CCW            0x02
#define ROTARY_PUSH           0x04
#define PBUTTON1_PUSHED       0x08
#define PBUTTON2_PUSHED       0x10
#define DISABLE_BUTTONS       0x20
#define MASTER_RESET          0x80
#define ANY_BUTTON            0xFF

#define MAX_MESSAGES 2
#define MAX_MEMORIES 4
#define AUTOSAVE_MEMORY_MS 2000

#define MAXCLK 3

// Mesages
#define NESSAGE1 20
#define HEADER1 20
#define HEADER2 11
#define CLKENTRYLEN 20

#define LCD_CLEAR_LINE_LENGTH 21
#define LCD_ERROR_MSG_LENGTH 9

// Display offset
#define OFFSET_DISPLAY_SHIFT 10
#define FREQUENCY_DISPLAY_SHIFT 2
#define ROTARY_NUMBER_OFFSET 6

// Frequencies defines
#define DEFAULT_FREQUENCY           7100000

#define DEFAULT_FREQUENCY_INCREMENT 1000000
#define DEFAULT_FREQUENCY_MULTIPLIER 100
#define MAXIMUM_FREQUENCY_MULTIPLIER 10000000
#define MINIMUM_FREQUENCY_MULTIPLIER 1

#define DEFAULT_LOW_FREQUENCY_LIMIT         8000UL
#define DEFAULT_HIGH_FREQUENCY_LIMIT        110000000UL     // was 114000000UL

#define SI_MAX_IQ_OUT_FREQ     80000000UL
#define SI_MIN_IQ_OUT_FREQ     3000000UL

#define DEFAULT_OFFSET_INCREMENT 1000000
#define DEFAULT_CALIBRATION_INCREMENT 10
#define MAXIMUM_OFFSET_MULTIPLIER 10000000
#define MINIMUM_OFFSET_MULTIPLIER 1
#define MAXIMUM_CALIBRATION_MULTIPLIER 100
#define MINIMUM_CALIBRATION_MULTIPLIER 1

#define MAXIMUM_OFFSET_FREQUENCY 50000000
#define MINIMUM_OFFSET_FREQUENCY 100000

// Menu Options
#define MAXMENU_ITEMS 9
#define MAXMENU_LEN 12

#define VFO_ENABLE 0
#define LO_ENABLE 1
#define IQ_ENABLE 2
#define SET_OFFSET 3
#define CALIBRATE 4
#define SAVE 5
#define RECALL 6
#define CLI_ENABLE 7
#define RESET 8

void ExecuteSerial (char *str);
void Reset (void);
void EEPROMWriteCorrection(void);
void EEPROMReadCorrection(void);
void DoMenu(unsigned char option);

void MenuDisplayMode(void);
void MenuClockWindowMode(void);
void MenuClockFrequencyMode(void);
void RefreshLCD (void);
void GetRotaryNumber (int lnum, int hnum, int maxinc, unsigned char row, unsigned char pos);

unsigned long LowFrequencyLimit (unsigned char line);
unsigned long HighFrequencyLimit (unsigned char line);

void EnableFrequency (unsigned char line);
void UpdateFrequency (unsigned char line);
void UpdateIQFrequency (unsigned char line);

void DisableFrequency (unsigned char line);
unsigned int GetPLLFreq(unsigned long freq);

unsigned char FrequencyDigitUpdate (long inc);
void MenuClockFrequencyOffsetMode (void);

void SetMemClkStatus (unsigned char stat, unsigned char index);

long absl (long v);

void printMem (unsigned char i);

unsigned char checkFlag (unsigned char type, unsigned long bitmask);
void setFlag (unsigned char type, unsigned long bitmask);
void clearFlag (unsigned char type, unsigned long bitmask);

#endif // _MAIN_H_
