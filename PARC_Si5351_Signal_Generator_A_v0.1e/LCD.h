#ifndef _MyLCD_H_
#define _MyLCD_H_

void SetupLCD (void);
void LCDClearScreen (void);

void LCDDisplayHeader (void);
void LCDErrorMsg (unsigned char pos, char *str);
void LCDClearErrorMsg (unsigned char pos);
void LCDClearClockWindow (void);
void LCDClearLine (unsigned char line);

void LCDSelectLine (unsigned char pos, unsigned char line, unsigned char enable);

void LCDDisplayFrequency (void);
void LCDDisplayNumber3D (int num, unsigned char row, unsigned char pos);
void LCDDisplayNumber1D (int num, unsigned char row, unsigned char pos);
void LCDDisplayClockFrequency (unsigned char line);
void LCDDisplayOffsetFrequency (unsigned char line);
void LCDDisplayLOClockFrequency (unsigned char line);
void LCDDisplayIQClockFrequency (unsigned char line);

void LCDDisplayMenuOption (unsigned char line);
void LCDDisplayClockEntry (unsigned char line); 
void LCDDisplayClockMode (unsigned char line);
void LCDDisplayClockStatus (unsigned char line);

#endif // _MyLCD_H_
