#ifndef _PSK_H_
#define _PSK_H_

#define PSK_BAUD_DELAY 31                     // 32 ms per bit. i.e. Baud is 31.25 and bit time is 1/31.25=32 ms 
#define PSK_IDLE_COUNT 10                     // number of baud timeperiods for continious phase reversals
#define PSK_CHAR_GAP_COUNT 3                  // number of continious phase reversals between characters

unsigned int ConvertVaricode (unsigned int code);
unsigned int LookupVaricode (char code);
unsigned char numbits (unsigned int in);
unsigned int swapbits (unsigned int in, unsigned char bits);

void SendPSKTestMsg (unsigned long frequency, unsigned char msgid);
void sendPSKMessage (char *msg, unsigned long frequency);
void sendPSKidle ( void );
void sendPSKchar ( unsigned int vcode );


#endif // _PSK_H_
