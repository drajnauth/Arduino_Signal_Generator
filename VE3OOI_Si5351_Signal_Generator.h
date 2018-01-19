#ifndef _MAIN_H_
#define _MAIN_H_



// Flags
#define OFFLINE_MODE 0x1
#define ONLINE_MODE 0x2
#define SWITCH_MODE 0x4
#define ROTARY_CW  0x8
#define ROTARY_CCW  0x10
#define ROTARY_PUSH 0x20
#define PBUTTON1_PUSHED 0x40
#define MASTER_RESET 0x80

#define SWEEP_DELAY 200   //200ms delay between frequency increments. Note it takes about 5 ms to change frequency

#define MAX_MESSAGES 2

#define MAXMENU_ITEMS 11
#define MAXMENU_LEN 11

// Frequencies defines
#define DEFAULT_FREQUENCY           7100000

#define DEFAULT_FREQUENCY_INCREMENT 100
#define DEFAULT_FREQUENCY_MULTIPLIER 100
#define MAXIMUM_FREQUENCY_MULTIPLIER 10000000
#define MINIMUM_FREQUENCY_MULTIPLIER 10
#define LOW_FREQUENCY_LIMIT         8000
#define HIGH_FREQUENCY_LIMIT        150000000


#define TX_FREQUENCY_OFFSET         975   // Was was -995 for LSB i.e positive for USB, negative LSB

// Menu Options
#define Freq_CLK0 0
#define Freq_CLK1 1
#define Freq_CLK2 2
#define RTY_1_CLK2 3
#define RTY_2_CLK2 4
#define PSK_1_CLK2 5
#define PSK_2_CLK2 6
#define SW_80_CLK2 7
#define SW_40_CLK2 8
#define SW_20_CLK2 9
#define Do_Reset 10

void ExecuteSerial (char *str);
void Reset (void);
void EEPROMWriteCorrection(void);
void EEPROMReadCorrection(void);
void UpdateFrequencyData (void);
void DoMenu(unsigned char option);
void ScanFrequencies (unsigned long fstart, unsigned long fend, unsigned long finc, unsigned char cons);

#endif // _MAIN_H_
