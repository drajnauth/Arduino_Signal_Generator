#ifndef _TIMER_H_
#define _TIMER_H_

#define TIMER10MS  1250         // Counter for 10 ms, default 2500
#define TIMER5MS   1250         // Counter for 5 ms, default 1250
#define TIMER3MS   750          // Counter for 3 ms, default 750
#define TIMER1MS   250          // Counter for 1 ms, default 250
#define TIMER500   125          // Counter for 0.5 ms, default 125
#define TIMER250   63           // Counter for .25 ms, default 63

// Timer Control Routines
void EnableTimers (unsigned char timer, unsigned int count);
void DisableTimers (unsigned char timer);
void Pause (int dly);
void DisableTimer0 (void);
void SaveTimerRegisters (void);
void RestoreTimerRegisters (void);


#endif // _TIMER_H_
