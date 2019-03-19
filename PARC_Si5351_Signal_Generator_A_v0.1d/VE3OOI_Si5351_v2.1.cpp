/*

  Program Written by Dave Rajnauth, VE3OOI to control the Si5351.
  It can output different frequencies from 1.8Khz to 200Mhz on any channel on Si5351

  Software is licensed (Non-Exclusive Licence) for use by the Peel Amateur Radion Club.  

  All other uses licensed under a Creative Commons Attribution 4.0 International License.

*/

#include <stdint.h>
#include <avr/eeprom.h>
#include "Arduino.h"

#include "VE3OOI_Si5351_v2.1.h"         // VE3OOI Si5351 Routines
#include "i2c.h"


//#define DEBUG_PRINT               

// These are variables used by the various routines.  Its globally defined to conserve ram memory
volatile unsigned long temp;
volatile unsigned char base, base2;
volatile unsigned char clkreg, clkreg0, clkreg1, clkreg2;
volatile unsigned char clkenable;
volatile unsigned int oldmult;
volatile unsigned long MS_P1, MS_P2, MS_P3;
volatile unsigned long MS_a, MS_b, MS_c;
volatile unsigned long Fxtal, Fxtalcorr;

volatile unsigned int mult;
volatile unsigned long long accum;

unsigned int R_DIV;
unsigned char MS_DIVBY4;
unsigned char Si5351RegBuffer[10];
double fraction; 


void setupSi5351 (int correction)
{

  i2cInit();

  // Set Crystal Internal Load Capacitance. For Adafruit module its 8 pf
  Si5351WriteRegister (SIREG_183_CRY_LOAD_CAP, SI_CRY_LOAD_8PF);

  // Set Crystal Internal Load Capacitance. For Adafruit module its 8 pf
  Si5351WriteRegister (SIREG_183_CRY_LOAD_CAP, SI_CRY_LOAD_8PF);

  // Set XTAL for source of PLLA and PLLB
  clkreg = Si5351ReadRegister (SIREG_15_PLL_INPUT_SRC);
  clkreg &= B00110011;
  Si5351WriteRegister (SIREG_15_PLL_INPUT_SRC, clkreg);

  // Define XTAL frequency. For Aadfruit it 25 Mhz.
  Fxtal =  SI_CRY_FREQ_25MHZ;
  Fxtalcorr = Fxtal + (long) ((double)(correction / 10000000.0) * (double) Fxtal);

  ResetSi5351();  

  while (CheckSi5351Status() & SI_NOT_INITIALIZED);

}

void ResetSi5351 (void) 
{
  // Disable clock outputs
  Si5351WriteRegister (SIREG_3_OUTPUT_ENABLE_CTL, 0xFF);  // Each bit corresponds to a clock outpout.  1 to disable, 0 to enable

  // Power off CLK0, CLK1, CLK2
  Si5351WriteRegister (SIREG_16_CLK0_CTL, 0x80);          // PLLA, CLK Powered off, MS Fractional Mode, Clk not inverted, Multisynthx, 8mA Drive
  Si5351WriteRegister (SIREG_17_CLK1_CTL, 0x80);          // PLLB, CLK Powered off, MS Fractional Mode, Clk not inverted, Multisynthx, 8mA Drive
  Si5351WriteRegister (SIREG_18_CLK2_CTL, 0x80);          // PLLA, CLK Powered off, MS Fractional Mode, Clk not inverted, Multisynthx, 8mA Drive

  // Reset phase registers
  UpdatePhaseRegister(SI_CLK0, 0);
  UpdatePhaseRegister(SI_CLK1, 0);
  UpdatePhaseRegister(SI_CLK2, 0);

  ResetSi5351PLL (SI_PLL_AB);

  clkenable = clkreg = base = base2 = MS_DIVBY4 = 0;
  clkreg0 = clkreg1 = clkreg2 = 0;
  MS_P1 = MS_P2 = MS_P3 = 0;
  MS_a =  MS_b = MS_c = 0;
  R_DIV = 0;
  oldmult = 0;

}

void ResetSi5351PLL (unsigned char pll) 
{
  unsigned char reg;  
  reg = Si5351ReadRegister (SIREG_177_PLL_RESET);
  if (pll == SI_PLL_A) {
    // Reset PLLA (bit 5 set) & PLLB (bit 7 set)
    reg |= SI_PLLA_RESET;
    Si5351WriteRegister (SIREG_177_PLL_RESET, reg);
    while ( Si5351ReadRegister(SIREG_177_PLL_RESET) & SI_PLLA_RESET );
    
  } else if (pll == SI_PLL_B) {
    // Reset PLLA (bit 5 set) & PLLB (bit 7 set)
    reg |= SI_PLLB_RESET;
    Si5351WriteRegister (SIREG_177_PLL_RESET, reg );
    while ( Si5351ReadRegister(SIREG_177_PLL_RESET) & SI_PLLB_RESET );
   
  } else {
    // Reset PLLA (bit 5 set) & PLLB (bit 7 set)
    reg |= SI_PLLA_RESET;
    reg |= SI_PLLB_RESET;
    Si5351WriteRegister (SIREG_177_PLL_RESET, reg );
    while ( Si5351ReadRegister(SIREG_177_PLL_RESET) & SI_PLLA_RESET );
    while ( Si5351ReadRegister(SIREG_177_PLL_RESET) & SI_PLLB_RESET );
  }
   
}


unsigned char CheckSi5351Status (void)
{
  return (Si5351ReadRegister (SIREG_0_DEVICE_STAT));
   
}

void PowerUpSi5351Clock (unsigned char clk)
// This routine turns off CLKs by setting the corresponding bit in the CLK control register
{  
  unsigned char reg;
  switch (clk) {
    case SI_CLK0:
      reg = Si5351ReadRegister (SIREG_16_CLK0_CTL);
      reg &= ~SI_CLK_OFF;
      Si5351WriteRegister (SIREG_16_CLK0_CTL, reg);      // Bit 8 must be set to power down clock, clear to enable. 1 to disable, 0 to enable
      break;
      
    case SI_CLK1:
      reg = Si5351ReadRegister (SIREG_17_CLK1_CTL);
      reg &= ~SI_CLK_OFF;
      Si5351WriteRegister (SIREG_17_CLK1_CTL, reg);
      break;
      
    case SI_CLK2:
      reg = Si5351ReadRegister (SIREG_18_CLK2_CTL);
      reg &= ~SI_CLK_OFF;
      Si5351WriteRegister (SIREG_18_CLK2_CTL, reg);
      break;
      
  }

}

void PowerDownSi5351Clock (unsigned char clk)
// This routine turns off CLKs by setting the corresponding bit in the CLK control register
{
  unsigned char reg;
  switch (clk) {
    case SI_CLK0:
      reg = Si5351ReadRegister (SIREG_16_CLK0_CTL);
      reg |= SI_CLK_OFF;
      Si5351WriteRegister (SIREG_16_CLK0_CTL, reg);      // Bit 8 must be set to power down clock, clear to enable. 1 to disable, 0 to enable
      break;
      
    case SI_CLK1:
      reg = Si5351ReadRegister (SIREG_17_CLK1_CTL);
      reg |= SI_CLK_OFF;
      Si5351WriteRegister (SIREG_17_CLK1_CTL, reg);
      break;
      
    case SI_CLK2:
      reg = Si5351ReadRegister (SIREG_18_CLK2_CTL);
      reg |= SI_CLK_OFF;
      Si5351WriteRegister (SIREG_18_CLK2_CTL, reg);
      break;
      
  }

}


void DisableSi5351Clock (unsigned char clk)
{
  unsigned char reg;
  reg = Si5351ReadRegister (SIREG_3_OUTPUT_ENABLE_CTL);
  
  switch (clk) {
    case SI_CLK0:
      reg |= SI_ENABLE_CLK0;
      break;
      
    case SI_CLK1:
      reg |= SI_ENABLE_CLK1;
      break;
      
    case SI_CLK2:
      reg |= SI_ENABLE_CLK2;
      break;
      
  }
  Si5351WriteRegister (SIREG_3_OUTPUT_ENABLE_CTL, reg);

}



void EnableUpSi5351Clock (unsigned char clk)
{
  unsigned char reg;
  reg = Si5351ReadRegister (SIREG_3_OUTPUT_ENABLE_CTL);

  switch (clk) {
    case SI_CLK0:
      reg &= ~SI_ENABLE_CLK0;
      break;
      
    case SI_CLK1:
      reg &= ~SI_ENABLE_CLK1;
      break;
      
    case SI_CLK2:
      reg &= ~SI_ENABLE_CLK2;
      break;
      
  }
  Si5351WriteRegister (SIREG_3_OUTPUT_ENABLE_CTL, reg);


}


void ProgramSi5351MSN (unsigned char clk, unsigned char pll, unsigned long pllfreq, unsigned long freq)
{
  unsigned long freq_temp;

  // The ValidateFrequency() call checks if frequency is below 1 Mhz or above 100 Mhz or above 150 Mhz.  See note above for frequencies below 1 Mhz or above 150 Mhz.  Frequencies
  // between 100 Mhz and 150 Mhz can be easily done using an integer multipler (i.e. use a fixed multipler of 6 - 6x100 Mhx is 600 Mhz which is inside PLL frequency requirement
  freq_temp = validateLowFrequency (freq);

  accum = (unsigned long long)pllfreq/(unsigned long long)freq_temp;
  MS_a = (unsigned long)accum;
 
  if (MS_a < SI5351_MULTISYNTH_A_MIN || MS_a > SI5351_MULTISYNTH_A_MAX) {
    Serial.print ("MS DIV ERR: ");
    Serial.println (MS_a);
    return;
  }

  accum = (unsigned long long)pllfreq % (unsigned long long)freq_temp;
  accum *= (unsigned long long)SI_MAX_DIVIDER;
  accum /= (unsigned long long)freq_temp;
  MS_b = (unsigned long)accum;
  MS_c = SI_MAX_DIVIDER;  
 

#ifdef DEBUG_PRINT
  unsigned long Fout;
  accum = (unsigned long long)freq_temp*(unsigned long long)MS_a + ((unsigned long long)freq_temp*(unsigned long long)MS_b) / (unsigned long long)MS_c;
  Fout = (unsigned long) accum;
  Serial.println ("\r\nProgramSi5351MSN ====================================");
  Serial.print ("Freq: ");
  Serial.print (freq_temp);
  Serial.print (" Pll: ");
  Serial.print ((char)pll);
  Serial.print (" Pll Freq: ");
  Serial.println (pllfreq);
  Serial.print ("Actual Divider: ");
  fraction = (double)pllfreq / (double)freq_temp;
  Serial.print ( fraction, 15 );
  Serial.print (" Calculated Divider: ");
  Serial.println ( ((double)MS_a+(double)MS_b/(double)MS_c), 15);
  Serial.print (" 64bit Recalc PLL Freq: ");
  Serial.println ( Fout );

  Fout = freq_temp * MS_a + (freq_temp * MS_b) / MS_c;
  Serial.print (" 32bit Recalc PLL Freq: ");
  Serial.println ( Fout );

  Serial.print ("MS_a: ");
  Serial.print (MS_a);
  Serial.print (" MS_b: ");
  Serial.print (MS_b);
  Serial.print (" MS_c: ");
  Serial.println (MS_c);

  Serial.print ("\r\nPhase: ");
  Serial.println (MS_a);

#endif   

  if (freq < SI_MAX_MS_FREQ) {
    // Fractional mode
    // encode A, B and C for multisynth divider into P1, P2 and P3
    temp = (128 * MS_b) / MS_c;
    MS_P1 = 128 * MS_a + temp - 512;
    MS_P2 = 128 * MS_b - MS_c * temp;
    MS_P3 = MS_c;

  } else {
    // Integer mode used only when fequency is over 150 Mhz.
    MS_P1 = 0;
    MS_P2 = 0;
    MS_P3 = 1;
  }

  
  // Zero all Buffer used for repeated write of all registers
  memset ((char *)&Si5351RegBuffer, 0, sizeof(Si5351RegBuffer));

  switch (clk) {
    case SI_CLK0:
      base = SIREG_42_MSYN0_1;                        // Base register address for PLL
      break;

    case SI_CLK1:
      base = SIREG_50_MSYN1_1;                        // Base register address for PLL
      break;

    case SI_CLK2:
      base = SIREG_58_MSYN2_1;                        // Base register address for PLL
      break;
  }

  Si5351RegBuffer[0] = (MS_P3 & 0x0000FF00) >> 8;
  Si5351RegBuffer[1] = (MS_P3 & 0x000000FF);
  Si5351RegBuffer[2] = ( ((MS_P1 & 0x00030000) >> 16) |
                         ((R_DIV & 0x7) << 4) |
                         ((MS_DIVBY4 & 0x3) << 2)) ;
  Si5351RegBuffer[3] = (MS_P1 & 0x0000FF00) >> 8;
  Si5351RegBuffer[4] = (MS_P1 & 0x000000FF);
  Si5351RegBuffer[5] = ((MS_P3 & 0x000F0000) >> 12) |
                       ((MS_P2 & 0x000F0000) >> 16);
  Si5351RegBuffer[6] = (MS_P2 & 0x0000FF00) >> 8;
  Si5351RegBuffer[7] = (MS_P2 & 0x000000FF);
  
  // Write the values to the corresponding register
  Si5351RepeatedWriteRegister(base, 8, Si5351RegBuffer);

  ResetSi5351PLL (SI_PLL_AB);  

  // clkreg is the actual data that will be written to the clock control register and we need to build it up based on parameters 
  clkreg = 0;    
  clkenable = Si5351ReadRegister(SIREG_3_OUTPUT_ENABLE_CTL);
        
  // Define the source for the clock. It can be PLLA, PLLB or XTAL pass through. XTAL passthrough simply take the clock frequency and passes it through
  switch (pll) {
    case SI_PLL_B:
      clkreg |= SI_CLK_SRC_PLLB;      // Set to use PLLB
     break;
      
    case SI_PLL_A:
      clkreg &= ~SI_CLK_SRC_PLLB;     // Set to use PLLA i.e. clear using PLLB define
      break;
      
    case SI_XTAL:
      clkreg &= ~SI_CLK_SRC_MS1;       // Set to use XTAL - i.e. XTAL passthrough.
     break;                          // PLL setting ignored
  }


  // if frequency is above 150 Mhz then must use integer mode. See note above for details
  if (freq >= SI_MIN_MSRATIO4_FREQ) {
    clkreg |= SI_CLK_MS_INT;                        // Set MSx_INT bit for interger mode
    clkreg |= SI_CLK_SRC_MS1;                       // Set CLK to use MultiSyncth1 as source
  } else {
    clkreg &= ~SI_CLK_MS_INT;                       // Clear MSx_INT bit for interger mode
    clkreg |= SI_CLK_SRC_MS1;                       // Set CLK to use MultiSyncth1 as source
  }

  // The bit values that are written to the register is different from the
  // interger ma numbers.  For example 2ma, a value of 0 is written into bits 0 & 1
  // For 6mA, a value of 4 is written into bits 0 & 1 of clock control register
  // mAdrive is the interger value for drive current (i.e. 2, 4, 6 8 mA)
  // "SI_CLK_2MA" is the actual value that is used to set appropriate bits in the clock control register
  // clkreg is the variable that has the actual data that will be written to the clock control register
  clkreg |= SI_CLK_8MA;
 
  // Update clk control based on above settings
  if (clk == SI_CLK0) {
    UpdateClkControlRegister (SI_CLK0);
    clkenable &= ~SI_ENABLE_CLK0;
    clkreg0 = clkreg;
    
  } else if (clk == SI_CLK1) {
    UpdateClkControlRegister (SI_CLK1);
    clkenable &= ~SI_ENABLE_CLK1;
    clkreg1 = clkreg;
    
  } else if (clk == SI_CLK2) {
    UpdateClkControlRegister (SI_CLK2);
    clkenable &= ~SI_ENABLE_CLK2;
    clkreg2 = clkreg;
  }

  Si5351WriteRegister (SIREG_3_OUTPUT_ENABLE_CTL, clkenable);
 
#ifdef DEBUG_PRINT
  clkreg = ReadClkControlRegister (clk);    
  clkenable = Si5351ReadRegister (SIREG_3_OUTPUT_ENABLE_CTL);    
  Serial.print ("Clk: ");
  Serial.print ((unsigned char)clk);
  Serial.print (" Read back clkreg: ");
  Serial.print (clkreg, BIN);
  Serial.print (" clkenable: ");
  Serial.print ( clkenable, BIN );
  Serial.println (" ");
#endif   


}


void ProgramSi5351PLL (unsigned char pll, unsigned long pllfreq)
{
  if (!Fxtalcorr) {
    return;
  }

  accum = (unsigned long long)pllfreq/(unsigned long long)Fxtalcorr;
  MS_a = (unsigned long)accum;

  if (MS_a < SI5351_PLL_MULTISYNTH_A_MIN || MS_a > SI5351_PLL_MULTISYNTH_A_MAX) {
    Serial.print ("PLL DIV ERR: ");
    Serial.println (MS_a);
    return;
  }
  
  accum = (unsigned long long)pllfreq % (unsigned long long)Fxtalcorr;
  accum *= (unsigned long long)SI_MAX_DIVIDER;
  accum /= (unsigned long long)Fxtalcorr;
  MS_b = (unsigned long)accum;
  MS_c = SI_MAX_DIVIDER;  
  
#ifdef DEBUG_PRINT
  unsigned long Fvco;
  accum = (unsigned long long)Fxtalcorr*(unsigned long long)MS_a+((unsigned long long)Fxtalcorr*(unsigned long long)MS_b)/(unsigned long long)MS_c;
  Fvco = (unsigned long) accum;
  Serial.println ("\r\nProgramSi5351PLL ====================================");
  Serial.print (" Pll: ");
  Serial.print ((char)pll);
  Serial.print (" PLL Freq: ");
  Serial.print (pllfreq);
  Serial.print (" Fxtalcorr: ");
  Serial.println (Fxtalcorr);
  Serial.print ("Actual Divider: ");
  fraction = (double)pllfreq / (double)Fxtalcorr;
  Serial.print ( fraction, 15 );
  Serial.print (" Calculated Divider: ");
  Serial.println ( ((double)MS_a+(double)MS_b/(double)MS_c), 15);
  Serial.print (" 64bit Recalc PLL Freq: ");
  Serial.println ( Fvco );
  Serial.print ("MS_a: ");
  Serial.print (MS_a);
  Serial.print (" MS_b: ");
  Serial.print (MS_b);
  Serial.print (" MS_c: ");
  Serial.println (MS_c);
#endif   

      
  // Encode Fractional PLL Feedback Multisynth Divider into P1, P2 and P3
  temp = (128 * MS_b) / MS_c;
  MS_P1 = 128 * MS_a + temp - 512;
  MS_P2 = 128 * MS_b - MS_c * temp;
  MS_P3 = MS_c;
  
  // Zero all Buffer used for repeated write of all registers
  memset ((char *)&Si5351RegBuffer, 0, sizeof(Si5351RegBuffer));

  switch (pll) {
    case SI_PLL_A:
      base = SIREG_26_MSNA_1;                        // Base register address for PLL A
      break;
    
    case SI_PLL_B:
      base = SIREG_34_MSNB_1;                        // Base register address for PLL B
      break;
  }

  //Load the buffer with MSN register data
  Si5351RegBuffer[0] = (MS_P3 & 0x0000FF00) >> 8;
  Si5351RegBuffer[1] = (MS_P3 & 0x000000FF);
  Si5351RegBuffer[2] = (MS_P1 & 0x00030000) >> 16;
  Si5351RegBuffer[3] = (MS_P1 & 0x0000FF00) >> 8;
  Si5351RegBuffer[4] = (MS_P1 & 0x000000FF);
  Si5351RegBuffer[5] = ((MS_P3 & 0x000F0000) >> 12) |
                       ((MS_P2 & 0x000F0000) >> 16);
  Si5351RegBuffer[6] = (MS_P2 & 0x0000FF00) >> 8;
  Si5351RegBuffer[7] = (MS_P2 & 0x000000FF);
  
  // Write the data to the Si5351
  Si5351RepeatedWriteRegister(base, 8, Si5351RegBuffer);

  if (CheckSi5351Status() & SI_PLLA_LOCK_LOSS) {
    Serial.println ("PLL LOCK ERROR");
  }


}



void SetIQFrequency (unsigned char clk, unsigned char clk2, unsigned char pll, unsigned long freq)
{
  unsigned long pllfreq;
  
  mult = GetPLLFreq(freq);
  if (!mult) {
    Serial.print ("Phase Err: ");
    Serial.println (mult);
    return;
  }
  pllfreq = freq * mult;

  R_DIV = 0;
  MS_DIVBY4 = 0;

#ifdef DEBUG_PRINT
  Serial.print ("Clk1: ");
  Serial.print (clk);
  Serial.print (" Clk2: ");
  Serial.print (clk);
  Serial.print (" PLL Freq: ");
  Serial.print (pllfreq);
  Serial.print (" Mult: ");
  Serial.println (mult);
#endif
  
  ProgramSi5351PLL(pll, pllfreq);
  
  ProgramSi5351MSN (clk, pll, pllfreq, freq);  
  ProgramSi5351MSN (clk2, pll, pllfreq, freq);  
  
  UpdatePhaseRegister (SI_CLK2, mult);
  
}


void SetManualFrequency (unsigned char clk, unsigned char pll, unsigned long pllfreq, unsigned long freq)
{

  R_DIV = 0;
  MS_DIVBY4 = 0;

#ifdef DEBUG_PRINT
  Serial.print ("Clk: ");
  Serial.println (clk);
  Serial.print (" PLL: ");
  Serial.println ((char)pll);
  Serial.print (" PLL Freq: ");
  Serial.print (pllfreq);
  Serial.print (" Mult: ");
  Serial.println (mult);
#endif
  
  ProgramSi5351PLL(pll, pllfreq);
  
  ProgramSi5351MSN (clk, pll, pllfreq, freq);

}

void SetFrequency (unsigned char clk, unsigned char pll, unsigned long freq)
{
  unsigned long pllfreq;
  
  if (freq > SI_MAX_OUT_FREQ) {
    freq = SI_MAX_OUT_FREQ;

  } else if (freq < SI_MIN_OUT_FREQ) {
    freq = SI_MIN_OUT_FREQ;
  }

  /* Low frequency - for Frequencies below 500 but above 8 Khz.
  need to use the R_DIV to reduce the frequency
  the trick here is to set the Output freq such that
  when when div by R_DIV you get frequency you want
  */
  R_DIV = 0;

  /* High frequency - for Frequencies above 150 Mhz to 160 Mhz.
  Need to set PLL to 4x Freq then used MS_DIVBY4 (i.e 11b or 0x3).  All P1,P2 dividers are 0, P3 is 1
  Need to also set MSx_INT bit in clock control register (bit 0x40)
  */
  MS_DIVBY4 = 0;
  

//
// This changed for 4x20 LCD Sig Gen where CLK0 and CLK2 share the same PLL
//
  // 75Mhz to 110Mhz
//  if (freq >= SI_MIN_MSRATIO8_FREQ && freq < SI_MAX_MSRATIO8_FREQ)  {
//    pllfreq = freq*8;
    
  // 110Mhz to 150Mhz
//  } else if (freq >= SI_MIN_MSRATIO6_FREQ && freq < SI_MAX_MSRATIO6_FREQ)  {

  // 110Mhz to 150Mhz
  if (freq > SI_MIN_MSRATIO6_FREQ && freq < SI_MAX_MSRATIO6_FREQ)  {
    pllfreq = freq*6;
    
  // 150 Mhz to 200Mhz
  } else if (freq >= SI_MIN_MSRATIO4_FREQ && freq <= SI_MAX_MSRATIO4_FREQ) {
    pllfreq = freq*4;
    MS_DIVBY4 = 0x3;  // Divide by 4

  // 2.8K to 8Khz
  } else if (freq < 8000 && freq > 2800) {
    pllfreq = SI_MIN_PLL_FREQ;

  } else if (freq <= 2800) {
    pllfreq = SI_VERY_MIN_PLL_FREQ;

  // 8 Khz to 75 Mhz
  } else {
    pllfreq = SI_MAX_PLL_FREQ;
  }

#ifdef DEBUG_PRINT
  Serial.print ("Clk: ");
  Serial.print ((unsigned char)clk);
  Serial.print (" Freq: ");
  Serial.print (freq);
  Serial.print (" PLL: ");
  Serial.print ((char)pll);
  Serial.print (" PLL Freq: ");
  Serial.println (pllfreq);
#endif
  
  ProgramSi5351PLL(pll, pllfreq);
  
  ProgramSi5351MSN (clk, pll, pllfreq, freq);

}


unsigned long validateLowFrequency (unsigned long freq)
// This routines determine if the frequency need any special configuration
// For example frequencies below 500 Khz and above 150 Mhz need special processing to make them work
// For convenience, the same approach used to calculate frequencies below 500 Khz is used to calculate 
// frequencies below is 1 Mhz
{
  unsigned long freq_temp;    // this will contain the new frequency based on divider used.

  // The idea here is that multiply frequency to be over 1 Mhz then we can generate multisynth dividers easily
  // When frequency is below 500 Khz, multisynch dividers are too big to generate the frequency
  // We then use the R_DIV divider to divide the output 
  
  freq_temp = freq;
  
  if (freq < 1000000 && freq >= 200000) {
    // Here we multiple frequency by 4 but then set R_DIV to divide output frequency by 4
    R_DIV = 0x2;      // 0x2 divide by 4
    freq_temp = freq * 4;

  } else if (freq < 200000 && freq >= 50000) {
    // Here we multiple frequency by 16 but then set R_DIV to divide output frequency by 16
    R_DIV = 0x4;      // 0x4 divide by 16
    freq_temp = freq * 16;

  } else if (freq < 50000 && freq >= SI_MIN_OUT_FREQ) {
    // Etc...
    R_DIV = 0x7;      // 0x7 divide by 128
    freq_temp = freq * 128;

  } 


#ifdef DEBUG_PRINT
  Serial.print ("Val Freq: ");
  Serial.print (freq);
  Serial.print (" xFreq: ");
  Serial.print (freq_temp);
  Serial.print (" R_DIV: ");
  Serial.print (R_DIV, HEX);
  Serial.print (" MS_DIVBY: ");
  Serial.println (MS_DIVBY4, HEX);
#endif   

  // return the updated frequency to be used to calculate multisynth dividers
  return freq_temp;
}

unsigned int GetPLLFreq(unsigned long freq) {
 
    unsigned long pfreq;
    unsigned int i;

    // Min divider for MS is 4 and max phase value is 128
    for (i = 4; i <= 128; i+=2) {
        pfreq = freq * (unsigned long)i;
        if (pfreq >= SI_VERY_MIN_PLL_FREQ && pfreq <= SI_MAX_PLL_FREQ) {
          return i;
        }
    } 
    return 0;
}



void InvertClk (unsigned char clk, unsigned char invert)
// This routine inverts the CLK0_INV bit in the clk control register.  
// When a sqaure wave is inverted, its the same as a 180 deg phase shift.
// This routine does not enable the clock.  Its assumed that its been enabled elsewhere
{

  if (invert) clkreg |= SI_CLK_INVERT;           // Set the invert bit in register to be written 
  else clkreg &= ~SI_CLK_INVERT;                 // clear the invert bit
      
  // Update clk control
  UpdateClkControlRegister (clk);
}


void UpdateClkControlRegister (unsigned char clk)
{
// This routine write the clock control register variable stored in the clock control strucutre to the
// Si5351 clock control register.  The register must be defined elsewhere.
// This routine does not enable the clock.  Its assumed that its been enabled elsewhere
  switch (clk) {
    case 0:
      Si5351WriteRegister (SIREG_16_CLK0_CTL, clkreg);
      break;

    case 1:
      Si5351WriteRegister (SIREG_17_CLK1_CTL, clkreg);
      break;

    case 2:
      Si5351WriteRegister (SIREG_18_CLK2_CTL, clkreg);
      break;
  }
}

unsigned char ReadClkControlRegister (unsigned char clk)
{

  switch (clk) {
    case 0:
      return Si5351ReadRegister (SIREG_16_CLK0_CTL);
      break;

    case 1:
      return Si5351ReadRegister (SIREG_17_CLK1_CTL);
      break;

    case 2:
      return Si5351ReadRegister (SIREG_18_CLK2_CTL);
      break;
  }
}

void UpdatePhaseRegister (unsigned char clk, unsigned char phase)
{
  unsigned char reg = 0;
  switch (clk) {
    case 0:
      reg = SIREG_165_CLK0_PHASE_OFFSET;
      break;

    case 1:
      reg = SIREG_166_CLK1_PHASE_OFFSET;
      break;

    case 2:
      reg = SIREG_167_CLK2_PHASE_OFFSET;
      break;
  } 

  Si5351WriteRegister (reg, phase);
  ResetSi5351PLL (SI_PLL_AB);              // Need to reset both PLLs for Phase to work!!
  
}



void Si5351RepeatedWriteRegister(unsigned char  addr, unsigned char  bytes, unsigned char *data)
{
  i2cSendRepeatedRegister(addr, bytes, data);
}


void Si5351WriteRegister (unsigned char reg, unsigned char value)
// Routine uses the I2C protcol to write data to the Si5351 register.
{
  unsigned char err;
 
  err = i2cSendRegister(reg, value);
  if (err) {
    Serial.print ("I2C W Err ");
    Serial.println (err);
  }
}

unsigned char Si5351ReadRegister (unsigned char reg)
// This function uses I2C protocol to read data from Si5351 register. The result read is returned
{
  unsigned char value, err;;

  err=i2cReadRegister(reg, &value);  
  if (err) {
    Serial.print ("I2C R Err ");
    Serial.println (err);
    value = 0;
  }

  return value;
}
