#ifndef _MAIN_H_
#define _MAIN_H_


#define REMOVE_CLI

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
#define RTTY_CLK_MODE 0x8
#define PSK_CLK_MODE 0x10
#define CW_CLK_MODE 0x20

// General Flags
#define MENU_MODE             0x1
#define CLOCK_WINDOW_MODE     0x2
#define CLOCK_FREQUENCY_MODE  0x4
#define LO_FREQUENCY_MODE     0x8
#define IQ_FREQUENCY_MODE     0x10
#define OFFSET_FREQUENCY_MODE 0x20
#define CALIBRATION_MODE      0x40
#define MEMORY_SAVE_MODE      0x80
#define MEMORY_RECALL_MODE    0x100

#define ROTARY_CW             0x1000
#define ROTARY_CCW            0x2000
#define ROTARY_PUSH           0x4000
#define PBUTTON1_PUSHED       0x8000
#define PBUTTON2_PUSHED       0x10000
#define DISABLE_BUTTONS       0x20000

#define PLLA_RUNNING          0x100000
#define PLLB_RUNNING          0x200000
#define MASTER_RESET          0x10000000

#define MAX_MESSAGES 2

#define MAXMENU_ITEMS 12
#define MAXMENU_LEN 12
#define MAXCLK 3

// Mesages
#define NESSAGE1 20
#define HEADER1 20
#define HEADER2 11
#define CLKENTRYLEN 20

// Display offset
#define OFFSET_DISPLAY_SHIFT 10
#define FREQUENCY_DISPLAY_SHIFT 2
#define ROTARY_NUMBER_OFFSET 6

// Frequencies defines
#define DEFAULT_FREQUENCY           7100000
#define DEFAULT_OFFSET_FREQUENCY    3910000




#define DEFAULT_FREQUENCY_INCREMENT 1000000
#define DEFAULT_FREQUENCY_MULTIPLIER 100
#define MAXIMUM_FREQUENCY_MULTIPLIER 10000000
#define MINIMUM_FREQUENCY_MULTIPLIER 1

#define LOW_FREQUENCY_LIMIT         2500
#define HIGH_FREQUENCY_LIMIT        200000000

#define DEFAULT_LOW_FREQUENCY_LIMIT         4000
#define DEFAULT_HIGH_FREQUENCY_LIMIT        114000000

#define IQ_LOW_FREQUENCY_LIMIT         5000000
#define IQ_HIGH_FREQUENCY_LIMIT        60000000

#define DEFAULT_OFFSET_INCREMENT 1000000
#define DEFAULT_CALIBRATION_INCREMENT 10
#define MAXIMUM_OFFSET_MULTIPLIER 10000000
#define MINIMUM_OFFSET_MULTIPLIER 1
#define MAXIMUM_CALIBRATION_MULTIPLIER 100
#define MINIMUM_CALIBRATION_MULTIPLIER 1

#define MAXIMUM_OFFSET_FREQUENCY 50000000
#define MINIMUM_OFFSET_FREQUENCY 100000


#define TX_FREQUENCY_OFFSET         975   // Was was -995 for LSB i.e positive for USB, negative LSB

// Menu Options
#define VFO_ENABLE 0
#define LO_ENABLE 1
#define IQ_ENABLE 2
#define SET_OFFSET 3
#define CALIBRATE 4
#define SAVE 5
#define RECALL 6
#define TBD7 7
#define TBD8 8
#define TBD9 9
#define TBD10 10
#define RESET 11

void ExecuteSerial (char *str);
void Reset (void);
void EEPROMWriteCorrection(void);
void EEPROMReadCorrection(void);
void DoMenu(unsigned char option);

void MenuDisplayMode(void);
void MenuClockWindowMode(void);
void MenuClockFrequencyMode(void);
int GetRotaryNumber (int lnum, int hnum, int maxinc, unsigned char row, unsigned char pos);

unsigned long LowFrequencyLimit (unsigned char line);
unsigned long HighFrequencyLimit (unsigned char line);
void EnableFrequency (unsigned char line);
void UpdateFrequency (unsigned char line);
void UpdateIQFrequency (unsigned char line);
void SetupClocks (void);
void DisableFrequency (unsigned char line);
unsigned int GetPLLFreq(unsigned long freq);

unsigned char FrequencyDigitUpdate (long inc);
void MenuClockFrequencyOffsetMode (void);

void SetMemClkStatus (unsigned char stat, unsigned char index);

long absl (long v);

void printMem (unsigned char i);

#endif // _MAIN_H_