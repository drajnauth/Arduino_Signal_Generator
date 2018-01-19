#ifndef _RTTY_H_
#define _RTTY_H_


void Pause (int dly);

void sendRTTYchar ( char value );
void sendRTTYbit (unsigned char b) ;
void sendRTTYMessage (char *msg, unsigned long frequency);
char Baudot( char c, unsigned char alpha);

char setupRTTYChar (char asciichar);
void TxRTTYbit (unsigned char b); 

void sendRTTYstart (void);
void sendRTTYstop (void);
void sendRTTYidle (void);

void SendRTTYTestMsg (unsigned long frequency, unsigned char msgid);

#define RTTY_FIGURES 27       //11011 bin
#define RTTY_LETTERS 31       //11111 bin

#define RTTY_TX_FIGURES 0xF6     //11110110 bin, 11 + 11011 + 0 (2 stop bits, FiguresCode, 1 Start bit)
#define RTTY_TX_LETTERS 0xFE     //11111110 bin, 11 + 11111 + 0 (2 stop bits, LettersCode, 1 Start bit)

#define BAUDOT_BITS 5
#define BAUDOT_TABLE_SIZE 33

#define RTTY_MODE 0
#define PSK_MODE 1

#define MAX_MESSAGE_SIZE 40

#define RTTY_ALPHA 1
#define RTTY_DIGIT 0

#define RTTY_SPACE_FREQUENCY 830    // Mark Frequency is 1000 Hz, Space Frequency is 830 Hz, Mid Frequency is 915Hz
#define RTTY_MARK_FREQUENCY 1000    // Mark Frequency is 1000 Hz, Space Frequency is 830 Hz, Mid Frequency is 915Hz
#define RTTY_SHIFT_FREQUENCY 170    // Shift of 170 Hz
// Frequencies defines

#define RTTY_BAUD_DELAY 22                    // 22 ms per bit. i.e. Baud is 45.45 and bit time is 1/45.45=22 ms 
//#define RTTY_BAUD_OVERHEAD_DELAY 16           // 22 ms per bit but 6 ms overhead for frequency change so is 16ms
//#define RTTY_STOPBIT_DELAY 33                 // 33 ms for stop bit. i.e 1.5 stop bits which is 22ms + 11 ms = 33ms. With 6 ms overhead, its 28ms 

#define RTTY_BAUD_OVERHEAD_DELAY 17           // 22 ms per bit but 4.7 ms overhead for frequency change so is 22-5=17ms
#define RTTY_STOPBIT_DELAY 44                 // 44 ms for 2 stop bits i.e. 22 x 2 = 44 ms.

#define RTTY_STOPBIT_OVERHEAD_DELAY 28        // 33 ms for stop bit. With 6 ms overhead, its 28ms 
#define RTTY_IDLE_DELAY 352                   // assume 500ms for rtty receiver to lock onto signal


#endif // _RTTY_H_
