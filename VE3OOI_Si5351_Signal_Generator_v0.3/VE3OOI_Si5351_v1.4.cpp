

#include <stdint.h>
#include <avr/eeprom.h>
#include "Arduino.h"
#include "Wire.h"


#include "VE3OOI_Si5351_v1.4.h"               // VE3OOI Si5351 Routines
#include "i2c.h"

// This defines the various parameter used to program Si5351 (See Silicon Labs AN619 Note)
// multisynch defines specific parameters used to determine Si5351 registers
// clk0ctl, clk1ctl, clk2ctl defined specific parameters used to control each clock
Si5351_def multisynth;
Si5351_CLK_def clk0ctl;
Si5351_CLK_def clk1ctl;
Si5351_CLK_def clk2ctl;

// These are variables used by the various routines.  Its globally defined to conserve ram memory
unsigned long temp;
unsigned char base;
unsigned char clkreg;

/*
The way the Si5351 works (in a nutshell) is the a PLL frequency is generated based on the Crystal Frequency (XTAL).  A multisyncth multiplier (called Feedback Multisynth Divider
but I refer to is at the PLL multisynth multiplier) is used to generate the PLL frequency. The PLL frequency MUST be between 600 Mhz and 900 Mhz!!. So for 25 Mhz clock the multipler must 
be between 24 and 36

The output clock (clk0, clk1, clk2) take a PLL frequency and applied another multisyncth divider (called Output Multisynth Divider) to get the desired frequency.

So,
PLL Frequency = XTAL_Frequency * (PLL_MS_A + PLL_MS_B / PLL_MS_C), where A, B and C is the PLL Multisynch multiplier
Clock Output Frequency = Desired Frequency = PLL_Frequency / (MS_A + MS_B / MS_C), where A, B, C are a seperate other multisynth divider (Output Multisynth Divider)

Therefore its critical to get the B/C ratio to represent the fractional component of the divider/multiplier.  
For example, to generate a PLL frequency of 612500000 Hz (612.5 Mhz), we need to multiply the 25Mhz crystal frequency by 24.5 or (24 + 1/2) so B/C = 1/2

The FareyFraction () routine take a decimal number (e.g. 0.5) and generates approprite B & C values to represent the decimal number.

The A, B and C values are encoded and then written to various Si5351 registers.

A special note about drive current (2, 4, 6, or 8 mA).  To get 50R output impedence you must use 8 mA.  Using lower drives causes the output impedence to increase above 50R.

Si5351_def is structure that defines the following "multisynth" structure
=========================================================================
The multsynth structure is used to define parameters used to configure PLL and Multisynth dividers.  It does not 
represent a running state.  

multisynth.Fxtal is the reference crystal clock
multisynth.Fxtalcorr is the adjusted/corrected reference crystal clock based on correction/calibration value
multisynth.correction correction for xtal in parts per 10 million
multisynth.PLL is the PLL to use either 'A' or 'B'
multisynth.MS_Fout is the output frequency
multisynth.PLL_Fvco is the PLL Clock Frequency (Max 900 MHz)

multisynth.ClkEnable is a parameter used to enable or disable the clock being configured

multisynth.MSN_P1 is PLL Feedback Multisynth Divider
multisynth.MSN_P2 is PLL Feedback Multisynth Divider
multisynth.MSN_P3 is PLL Feedback Multisynth Divider
multisynth.PLL_a is PLL Feedback Multisynth Divider
multisynth.PLL_b is PLL Feedback Multisynth Divider
multisynth.PLL_c is PLL Feedback Multisynth Divider

multisynth.PLL_Fvco = multisynth.Fxtal * (multisynth.PLL_a + multisynth.PLL_b/multisynth.PLL_c)
where a, b, c are fractional dividers for PLL frequency
a is multiplier, b is numerator and c is demonator (a+b/c)

multisynth.MS_a is Output Multisynth Divider
multisynth.MS_b is Output Multisynth Divider
multisynth.MS_c is Output Multisynth Divider
multisynth.MS_P1 is Output Multisynth Divider
multisynth.MS_P2 is Output Multisynth Divider
multisynth.MS_P3 is Output Multisynth Divider

multisynth.MS_Fout = multisynth.PLL_Fvco * (multisynth.MS_a + multisynth.MS_b/multisynth.MS_c)

To convert a, b and c to P1, P2 and P3 the following calculation is done for either PLL multisynch divider or output multisynch output
temp = (128*multisynth.MS_b)/multisynth.MS_c;      // Note that 128*multisynth.MS_b is done first to have the most accurate integer value
multisynth.MS_P1 = 128 * multisynth.MS_a + temp - 512;
multisynth.MS_P2 = 128 * multisynth.MS_b - multisynth.MS_c * temp;
multisynth.MS_P3 = multisynth.MS_c;
where a, b, c are fractional dividers for output
a is multiplier, b is numerator and c is demonator (a+b/c)

multisynth.R_DIV R_DIV MUST be used for frequencies below 500 Hz (to min of 8 Khz).  These routines used R_DIV for frequencies below 1 Mhz.  R_DIV indicates that the output frequency should be divided by 1, 2, 4, 8,....128. 
R_DIV = 0 means divide by 1 (the default configuration).  Bascially the output frequecy is multiplied by an interger to get the frequency above
1 Mhz, then the R_DIV is applied to bring the frequency back to the frequency.
For example, for 800 Khz:
- The frequency output is multiplied by 4 (4x800=3.2 Mhz). That is the PLL and Output Multisynth for Clock is set to 3.2 Mhz
- The R_DIV is set to 4 so the actual output frequency is 3.2/4 = 800 Khz
- This may see silly but its necessary for the Si5351 below 500 Khz.  This was done below 1 Mhz for convenience.

multisynth.MS_DIVBY4 is a similar divider that is used for frequencies above 150 Mhz to 160 Mhz.  The Multisynth cannot generate frequencies easily above 150 Mhz (its too small). 
So you need to setup the PLL and output multisynch for 4xDesiged frequency.  PLL frequencies must be between 600 to 900 Mhz.  4x150 Mhz is 600 Mhz.  So, we setup the PLL and Multisynch 
to generate 4x the desired fequency and then apply the divide by 4 (MS_DIVBY4) at the end to bring the frequency back to the desired frequency.

multisynth.ClkEnable is a parameter that is used to enable and disable the clock.

Si5351_CLK_def is structure that defines clock control (clk0ctl, clk1ctl, clk2ctl) structure as follows
=======================================================================================================
The clock control structure is used to define parameters for the current running state of the clock.
clk0ctl, clk1ctl and clk2ctl structure are all the same except they are dedicated to the specific clock. 
At any point in time, these structures defines how the clock is configured.

clk0ctl.PLL is the PLL that is assigned to the clock. Either "A" or "B"
clk0ctl.mAdrive is the actual mA for the output i.e. 2, 4, 6, 8.  The UpdateDrive() routine converts to appropriate register value
clk0ctl.phase is the phase from 0 to clk0ctl.maxangle. Each binary value increments the delay for the output frequency by 1 / (4 x clk0ctl.PLLFreq)
clk0ctl.maxangle is the maximum phase based on the PLL Frequency used to generate the output frequency
clk0ctl.reg is the control register (16,17,18) value that was last written to the Si5351
clk0ctl.freq is the output frequency for the running clock
clk0ctl.PLLFreq is the PLL Frequency that is used to generate the output frequency
*/

void ResetSi5351 (unsigned int loadcap)
// This routine zeros out all structures and write 0 to various control Si5351 registers to reset the chip
// Note when switching between > 150 Mhz to < 150 Mhz, a reset is required!
// It also sets the crystal load capacitance and the Crystal frequency.
// For the Adafruit module, its 25 Mhz. There are no other Adafruit modules.
{
  unsigned char i;
  
  // Zero all clk registers and multisynth variables
  memset ((char *)&clk0ctl, 0, sizeof(clk0ctl));
  memset ((char *)&clk1ctl, 0, sizeof(clk1ctl));
  memset ((char *)&clk2ctl, 0, sizeof(clk2ctl));
  memset ((char *)&multisynth, 0, sizeof(multisynth));

  i2cInit();

  // Disable clock outputs
  Si5351WriteRegister (SIREG_3_OUTPUT_ENABLE_CTL, 0xFF);  // Each bit corresponds to a clock outpout.  1 to disable, 0 to enable

  // Power off CLK0, CLK1, CLK2
  Si5351WriteRegister (SIREG_16_CLK0_CTL, 0x80);      // Bit 8 must be set to power down clock, clear to enable. 1 to disable, 0 to enable
  Si5351WriteRegister (SIREG_17_CLK1_CTL, 0x80);
  Si5351WriteRegister (SIREG_18_CLK2_CTL, 0x80);

  // Zero ALL multisynth registers.
  for (i = 0; i < SI_MSREGS; i++) {
    Si5351WriteRegister (SIREG_26_MSNA_1 + i, 0x00);
    Si5351WriteRegister (SIREG_34_MSNB_1 + i, 0x00);
    Si5351WriteRegister (SIREG_42_MSYN0_1 + i, 0x00);
    Si5351WriteRegister (SIREG_50_MSYN1_1 + i, 0x00);
    Si5351WriteRegister (SIREG_58_MSYN2_1 + i, 0x00);
  }

  // Set Crystal Internal Load Capacitance. For Adafruit module its 8 pf
  Si5351WriteRegister (SIREG_183_CRY_LOAD_CAP, loadcap);

  // Define XTAL frequency. For Aadfruit it 25 Mhz.
  multisynth.Fxtal =  SI_CRY_FREQ_25MHZ;
}

void DisableSi5351Clocks ( void )
// This routine turns off CLKs by setting the corresponding bit in the CLK control register
{
  // Power off CLK0, CLK1, CLK2
  Si5351WriteRegister (SIREG_16_CLK0_CTL, 0x80);      // Bit 8 must be set to power down clock, clear to enable. 1 to disable, 0 to enable
  Si5351WriteRegister (SIREG_17_CLK1_CTL, 0x80);
  Si5351WriteRegister (SIREG_18_CLK2_CTL, 0x80);

}


void SetFrequency (unsigned char clk, char pll, unsigned long freq, unsigned char drive)
// This routing simplifies the setting of a frequency.  Only need CLK, PLL (A or B), freq and mA drive (2, 4, 6, 8 mA)
// The routine assumes phase is 0 and autodetermines PLL frequency
// Note that drive needs to be either 2, 4, 6, or 8.  The UpdateDrive() is called from the SetupFrequency() function and converts to Register Coded Value.
{

// The SetupFrequency() routine is a robust mechanism for setting the frequency
  SetupFrequency (clk, pll, SI_MAX_PLL_FREQ, freq, 0, drive);
 
}

void SetupFrequency (unsigned char clk, char pll, unsigned long pllfreq, unsigned long freq, unsigned int phase, unsigned char mAdrive)
// This is the detailed call to configure a frequency.  It requires Clk (0,1,2), PLL (A or B), pllfreq (0 for autodetermine or 600-900 Mhz), phase (0-max angle), mAdrive (2,4,6,8 mA)
// See note above about phase configuration and programming note AN619
// Note that SI_XTAL can be use instead of PLL "A" or "B".  This simply passes crystal frequency to the output (i.e. output is 25 Mhz and multiplier and dividers are not used).
{
//  unsigned long temp;
//  unsigned char base;
//  unsigned char clkreg;

  // Validate frequency limits
  if (freq > SI_MAX_OUT_FREQ) {
    freq = SI_MAX_OUT_FREQ;

  } else if (freq < SI_MIN_OUT_FREQ) {
    freq = SI_MIN_OUT_FREQ;
  }

  // define the fequency to be used. This variable is used by other routines.
  multisynth.MS_Fout = freq;

  // In this case use provided PLL frequency and then calculate MS dividers for the given PLL frequency. This is stable however Si5351 states that integer multipler/dividers are preferred.
  multisynth.PLL_Fvco = pllfreq;
  
  // The ValidateFrequency() call checks if frequency is below 1 Mhz or above 100 Mhz or above 150 Mhz.  See note above for frequencies below 1 Mhz or above 150 Mhz.  Frequencies
  // between 100 Mhz and 150 Mhz can be easily done using an integer multipler (i.e. use a fixed multipler of 6 - 6x100 Mhx is 600 Mhz which is inside PLL frequency requirement
  freq = ValidateFrequency (freq);
  
  // CalculateCLKDividers() determines A, B and C for multisynth divider for clock.
  unsigned long numerator, remainder;
  uint64_t numerator64;

  // Use the higher PLL frequency possible (one less calculation to make)
  multisynth.PLL_Fvco = SI_MAX_PLL_FREQ;

  // define the fequency to be used. This variable is used by other routines.
  multisynth.MS_Fout = freq;
  multisynth.PLL = pll;

/*
Process to program Si5351 in simple mode
1. Use Max PLL freq which is 900 Mhz

2. Identify Multisynth dividers to generate 900 Mhz from Crystal frequency
a) However need to first correct Xtal freq.
b) Then figure out PLL divider
i.e. PLL_Freq/Xtal_freq gives divider to be converted to a+b/c (PLL_a, PLL_b, PLL_c)

3) Once the PLL frequency is known can calculate the clock Multisynth divider 
necessary to convert the PLL frequency to the desired output frequency.
i.e. PLL_Freq/Freq gives divider which must be converted to a+b/c (MS_a, MS_b, MS-c)

Need to convert fractional part of divider into b/c. Can use Farey or other rational 
number approximation.
*/

// Step 1. Calculate PLL Frequency Divider
  // Calculate the corrected/calibrated crystal frequency that will be used to calculate the PLL frequency
  multisynth.Fxtalcorr = multisynth.Fxtal + (long) ((double)(multisynth.correction / 10000000.0) * (double) multisynth.Fxtal);

  // Calculate the multipler (quotient) to get the PLL frequency
  multisynth.PLL_a = multisynth.PLL_Fvco / multisynth.Fxtalcorr;
/*
The basic approach is that x/y will result in a quotent q and remainder r/y (e.g. 3/2 is 1.5 Quotient is 1, remainder is 1/2). 
We need to get the remainder from the division and then figure out the most accurate fraction based on the largest number that can be used (i.e. SI_MAXIMUM_DENOMINATOR).
To do this take the remainder and do a linear extrapolation to make it a   

Because the remainder would be a large number then multiplying, it by another large number (e.g. SI_MAXIMUM_DENOMINATOR) 
may result in a 32bit integer to overflow.  So we need to use a 64bit integer which is not fully supported in Arduino.

So define a 64bit interger and completed the math, then convert back to a 32 bit integer.

*/  
  // Calculate the remainder for the multiplier to get the PLL frequency
  remainder = multisynth.PLL_Fvco % multisynth.Fxtalcorr;
  numerator64 = remainder;
  numerator64 *= SI_MAXIMUM_DENOMINATOR;

  do_div(numerator64, multisynth.Fxtalcorr); 
  
  numerator = (unsigned long)numerator64;
  
  multisynth.PLL_b = numerator;
  multisynth.PLL_c = SI_MAXIMUM_DENOMINATOR;
  
// Encode Fractional PLL Feedback Multisynth Divider into P1, P2 and P3
  temp = (128 * multisynth.PLL_b) / multisynth.PLL_c;
  multisynth.MSN_P1 = 128 * multisynth.PLL_a + temp - 512;
  multisynth.MSN_P2 = 128 * multisynth.PLL_b - multisynth.PLL_c * temp;
  multisynth.MSN_P3 = multisynth.PLL_c;

  // Encode Fractional PLL Feedback Multisynth Divider into P1, P2 and P3
  temp = (128 * multisynth.PLL_b) / multisynth.PLL_c;
  multisynth.MSN_P1 = 128 * multisynth.PLL_a + temp - 512;
  multisynth.MSN_P2 = 128 * multisynth.PLL_b - multisynth.PLL_c * temp;
  multisynth.MSN_P3 = multisynth.PLL_c;




// Step 2. Calculate clock divider  
  // The PLL clock frequency is based on an interger multiple of the desired frequency
  // multisynth.PLL_Fvco = multisynth.MS_Fout * multisynth.SimpleDivider
  // so (a+b/c) is a = multisynth.SimpleDivider, b=0, c=1
  // encode A, B and C for multisynth divider into P1, P2 and P3
  // Calculate the multipler to get the maximum PLL frequency
  // The divider below is used to calculate the multisynch dividers (a+b/c) to get the clock frequency from the PLL frequency calculated below

  // Calculate the multipler (quotient) to get the Output frequency
  multisynth.MS_a = multisynth.PLL_Fvco /freq;
  
  // Calculate the remainder for the multiplier to get the Output frequency
  remainder = multisynth.PLL_Fvco % freq;
  numerator64 = remainder;
  numerator64 *= SI_MAXIMUM_DENOMINATOR;

  do_div(numerator64, freq); 
  
  numerator = (unsigned long)numerator64;
  
//  RationalNumberApproximation(numerator, SI_MAXIMUM_DENOMINATOR, (SI_MAXIMUM_DENOMINATOR - 1), SI_MAXIMUM_DENOMINATOR, &b, &c);   
//  multisynth.MS_b = b;
//  multisynth.MS_c = c;
  multisynth.MS_b = numerator;
  multisynth.MS_c = SI_MAXIMUM_DENOMINATOR;

//  multisynth.MS_Fout = freq;
  if (freq <= SI_MAX_MS_FREQ) {
    // Fractional mode
    // encode A, B and C for multisynth divider into P1, P2 and P3
    temp = (128 * multisynth.MS_b) / multisynth.MS_c;
    multisynth.MS_P1 = 128 * multisynth.MS_a + temp - 512;
    multisynth.MS_P2 = 128 * multisynth.MS_b - multisynth.MS_c * temp;
    multisynth.MS_P3 = multisynth.MS_c;

  } else {
    // Integer mode used only when fequency is over 150 Mhz.
    multisynth.MS_P1 = 0;
    multisynth.MS_P2 = 0;
    multisynth.MS_P3 = 1;
  }

  // clkreg is the actual data that will be written to the clock control register and we need to build it up based on parameters passed to this routine
  // We first restore the last clock control register for the clock being configured.
  // Note that when the ResetSi5351() is called all registered are zeroed
  if (clk == SI_CLK0) {
     clkreg = clk0ctl.reg;
  } else if (clk == SI_CLK1) {
    clkreg = clk1ctl.reg;
  } else if (clk == SI_CLK2) {
    clkreg = clk2ctl.reg;
  }

/*
Reg 16-18: Power up clock, set fractional mode, set PLLA, set MultiSynth 0 as clock source, current output
Reg 165-167: Sets phase

For clock control register, the bits are as follows
  7  Power down (o power up)
  6  MultiSynth 0 Integer Mode (0 Fractional mode, 1 Intefer mode)
  5  MultiSynth Source (0 for PLLA, 1 for PLLB)
  4  Clock Invert
  3  set to 1 for MultiSynth source
  2  set to 1 for MultiSynth source
  1  set to 1 for 8mA source (50R output impedence)
  0  set to 1 for 8mA source (50R output impedence)
*/

  // clkreg is the actual data that will be written to the clock control register and we need to build it up based on parameters passed to this routine
  clkreg &= SI_CLK_CLR_DRIVE;          // Clear original mA drive setting

  // Define the source for the clock. It can be PLLA, PLLB or XTAL pass through. XTAL passthrough simply take the clock frequency and passes it through
  switch (pll) {
    case SI_PLL_B:
      clkreg |= SI_CLK_SRC_PLLB;      // Set to use PLLB
      break;
    case SI_PLL_A:
      clkreg &= ~SI_CLK_SRC_PLLB;     // Set to use PLLA i.e. clear using PLLB define
      break;
    case SI_XTAL:
      clkreg &= ~SI_CLK_SRC_MS;       // Set to use XTAL - i.e. XTAL passthrough.
      break;                          // PLL setting ignored
  }

  // if frequency is above 150 Mhz then must use integer mode. See note above for details
  if (freq > SI_MAX_MS_FREQ) {
    clkreg |= SI_CLK_MS_INT;  // Set MSx_INT bit for interger mode
  } else {
    clkreg &= ~SI_CLK_MS_INT;                       // Clear MSx_INT bit for interger mode
    clkreg |= SI_CLK_SRC_MS;                        // Set CLK to use MultiSyncth as source
  }
  
  // The bit values that are written to the register is different from the
  // interger ma numbers.  For example 2ma, a value of 0 is written into bits 0 & 1
  // For 6mA, a value of 4 is written into bits 0 & 1 of clock control register
  // mAdrive is the interger value for drive current (i.e. 2, 4, 6 8 mA)
  // "SI_CLK_2MA" is the actual value that is used to set appropriate bits in the clock control register
  // clkreg is the variable that has the actual data that will be written to the clock control register
  switch (mAdrive) {
    case 2:
      clkreg |= SI_CLK_2MA;
      break;
    case 4:
      clkreg |= SI_CLK_4MA;
      break;
    case 6:
      clkreg |= SI_CLK_6MA;
      break;
    case 8:
      clkreg |= SI_CLK_8MA;
      break;
    default:
      clkreg |= SI_CLK_8MA;
  }


  // Define clk setting in corresponding CLK structure. Then update the clock
  // Save the current control register value so that can then change drive and phase 
  // on the fly by simply updating the register with corresponding value
  switch (clk) {
    case 0:
      clk0ctl.PLL = pll;
      clk0ctl.PLLFreq = multisynth.PLL_Fvco;
      clk0ctl.mAdrive = mAdrive;
      clk0ctl.freq = freq;
      clk0ctl.reg = clkreg;
      // See AN619 regarding how phase is calculated.  This defines the max phase allowed.  The register only 
      // allow 127 values and therefore a maximum phase shift is allowed
      clk0ctl.maxangle = (unsigned int)( SI_PHASE_CONSTANT * (double)freq / (double)multisynth.PLL_Fvco );
      multisynth.ClkEnable &= ~SI_ENABLE_CLK0;       // Enable clk0, bit must be cleared to enable
      break;

    case 1:
      clk1ctl.PLL = pll;
      clk1ctl.PLLFreq = multisynth.PLL_Fvco;
      clk1ctl.mAdrive = mAdrive;
      clk1ctl.freq = freq;
      clk1ctl.reg = clkreg;
      clk1ctl.maxangle = (unsigned int)( SI_PHASE_CONSTANT * (double)freq / (double)multisynth.PLL_Fvco );
      multisynth.ClkEnable &= ~SI_ENABLE_CLK1;      // Enable clk1
      break;

    case 2:
      clk2ctl.PLL = pll;
      clk2ctl.PLLFreq = multisynth.PLL_Fvco;
      clk2ctl.mAdrive = mAdrive;
      clk2ctl.freq = freq;
      clk2ctl.reg = clkreg;
      clk2ctl.maxangle = (unsigned int)( SI_PHASE_CONSTANT * (double)freq / (double)multisynth.PLL_Fvco );
      multisynth.ClkEnable &= ~SI_ENABLE_CLK2;      // Enable clk2
      break;
  }

  ProgramSi5351 (clk, phase);

}

void ProgramSi5351 (unsigned char clk, unsigned int phase)
{

  unsigned char Si5351RegBuffer[10];
  
 
  // define the base resister for PLLA or PLLB
  if (multisynth.PLL == SI_PLL_A) {
    base = SIREG_26_MSNA_1;                        // Base register address for PLL A
  } else {
    base = SIREG_34_MSNB_1;                        // Base register address for PLL b
  }

  // Zero all Buffer used for repeated write of all registers
  memset ((char *)&Si5351RegBuffer, 0, sizeof(Si5351RegBuffer));

  //Load the buffer with MSN register data
  Si5351RegBuffer[0] = (multisynth.MSN_P3 & 0x0000FF00) >> 8;
  Si5351RegBuffer[1] = (multisynth.MSN_P3 & 0x000000FF);
  Si5351RegBuffer[2] = (multisynth.MSN_P1 & 0x00030000) >> 16;
  Si5351RegBuffer[3] = (multisynth.MSN_P1 & 0x0000FF00) >> 8;
  Si5351RegBuffer[4] = (multisynth.MSN_P1 & 0x000000FF);
  Si5351RegBuffer[5] = ((multisynth.MSN_P3 & 0x000F0000) >> 12) |
                       ((multisynth.MSN_P2 & 0x000F0000) >> 16);
  Si5351RegBuffer[6] = (multisynth.MSN_P2 & 0x0000FF00) >> 8;
  Si5351RegBuffer[7] = (multisynth.MSN_P2 & 0x000000FF);
  
  // Write the data to the Si5351
  Si5351RepeatedWriteRegister(base, 8, Si5351RegBuffer);

  // Set the base register for the Multisynth diveder for the clock
  // clkreg is the actual data that will be written to the clock control register and we need to build it up based on parameters passed to this routine
  // We first restore the last clock control register for the clock being configured.
  // Note that when the ResetSi5351() is called all registered are zeroed
  if (clk == SI_CLK0) {
    base = SIREG_42_MSYN0_1;    // Base register address for Out 0
  } else if (clk == SI_CLK1) {
    base = SIREG_50_MSYN1_1;          // Base register address for Out 1
  } else if (clk == SI_CLK2) {
    base = SIREG_58_MSYN2_1;          // Base register address for Out 2
  }
  
  // Zero all Buffer used for repeated write of all registers
  memset ((char *)&Si5351RegBuffer, 0, sizeof(Si5351RegBuffer));

  Si5351RegBuffer[0] = (multisynth.MS_P3 & 0x0000FF00) >> 8;
  Si5351RegBuffer[1] = (multisynth.MS_P3 & 0x000000FF);
  Si5351RegBuffer[2] = ( ((multisynth.MS_P1 & 0x00030000) >> 16) |
                         ((multisynth.R_DIV & 0x7) << 4) |
                         ((multisynth.MS_DIVBY4 & 0x3) << 2)) ;
  Si5351RegBuffer[3] = (multisynth.MS_P1 & 0x0000FF00) >> 8;
  Si5351RegBuffer[4] = (multisynth.MS_P1 & 0x000000FF);
  Si5351RegBuffer[5] = ((multisynth.MS_P3 & 0x000F0000) >> 12) |
                       ((multisynth.MS_P2 & 0x000F0000) >> 16);
  Si5351RegBuffer[6] = (multisynth.MS_P2 & 0x0000FF00) >> 8;
  Si5351RegBuffer[7] = (multisynth.MS_P2 & 0x000000FF);


  
  // Write the values to the corresponding register
  Si5351RepeatedWriteRegister(base, 8, Si5351RegBuffer);
  
  
  // Write the values to the corresponding register
//  Si5351WriteRegister( base++, (multisynth.MS_P3 & 0x0000FF00) >> 8);
//  Si5351WriteRegister( base++, (multisynth.MS_P3 & 0x000000FF));

//  Si5351WriteRegister( base++, (
//                         ((multisynth.MS_P1 & 0x00030000) >> 16) |
//                         ((multisynth.R_DIV & 0x7) << 4) |
//                         ((multisynth.MS_DIVBY4 & 0x3) << 2)) );

//  Si5351WriteRegister( base++, (multisynth.MS_P1 & 0x0000FF00) >> 8);
//  Si5351WriteRegister( base++, (multisynth.MS_P1 & 0x000000FF));
//  Si5351WriteRegister( base++, ((multisynth.MS_P3 & 0x000F0000) >> 12) |
//                       ((multisynth.MS_P2 & 0x000F0000) >> 16) );
//  Si5351WriteRegister( base++, (multisynth.MS_P2 & 0x0000FF00) >> 8);
//  Si5351WriteRegister( base, (multisynth.MS_P2 & 0x000000FF));

  // Update clk control based on above settings
  UpdateClkControlRegister (clk);

  // Calculate the phase and then set the phase register. Note that the PLL must be reset for the phase to take effect.
  // Note phase is defined as degrees however the phase control register uses time based on PLL frequency period
  UpdatePhase (clk, phase);
  UpdatePhaseControlRegister (clk);
  
  // The ResetSi5351() routine disables all output clocks and they need to be enabled.  Below enables the specific clock referenced in this routine
  Si5351WriteRegister (SIREG_3_OUTPUT_ENABLE_CTL, multisynth.ClkEnable);

  
}

unsigned long ValidateFrequency (unsigned long freq)
// This routines determine if the frequency need any special configuration
// For example frequencies below 500 Khz and above 150 Mhz need special processing to make them work
// For convenience, the same approach used to calculate frequencies below 500 Khz is used to calculate 
// frequencies below is 1 Mhz
{
  unsigned long freq_temp;    // this will contain the new frequency based on divider used.

  /* Low frequency - for Frequencies below 500 but above 8 Khz.
  need to use the R_DIV to reduce the frequency
  the trick here is to set the Output freq such that
  when when div by R_DIV you get frequency you want
  */
  multisynth.R_DIV = 0;

  /* High frequency - for Frequencies above 150 Mhz to 160 Mhz.
  Need to set PLL to 4x Freq then used DIV_BY4.  All P1,P2 dividers are 0, P3 is 1
  Need to also set MSx_INT bit in clock control register (bit 0x40)
  */
  multisynth.MS_DIVBY4 = 0;

  // The idea here is that multiply frequency to be over 1 Mhz then we can generate multisynth dividers easily
  // When frequency is below 500 Khz, multisynch dividers are too big to generate the frequency
  // We then use the R_DIV divider to divide the output 
    
  if (freq < 1000000 && freq >= 200000) {
    // Here we multiple frequency by 4 but then set R_DIV to divide output frequency by 4
    multisynth.R_DIV = 0x2;      // 0x2 divide by 4
    freq_temp = freq * 4;

  } else if (freq < 200000 && freq >= 50000) {
    // Here we multiple frequency by 16 but then set R_DIV to divide output frequency by 16
    multisynth.R_DIV = 0x4;      // 0x4 divide by 16
    freq_temp = freq * 16;

  } else if (freq < 50000 && freq >= 8000) {
    // Etc...
    multisynth.R_DIV = 0x7;      // 0x7 divide by 128
    freq_temp = freq * 128;

  } else freq_temp = freq;

  // return the updated frequency to be used to calculate multisynth dividers
  return freq_temp;
}


void CalculatePLLFrequency (unsigned long freq)
// This routine tries to determine the best PLL frequency based on the following conditions (see AN619 for details)
// 1.  PLL multisynth multipler (a,b,c) must be 24 to 36 (based on 25 Mhz crystal frequency)
// 2.  Output multisynth divider must be between 8 to 900
// Bascially the routines scans dividers to generate output from PLL frequency and determine 
// The best PLL multiplier to use. It prefers whole number (e.g. 24.0, 29.0, 30.0, etc) or numbers closest to whole numbers (24.98, 26.01, etc)
{
  unsigned long pllmult, freq_temp;               // used to identify pll multiplier as well as actually frequency. See ValidateFrequency()
  double fraction, minfraction, maxfraction;      // Used to find whole number or numbers closest to whole numbers
  unsigned int i;
  unsigned int maxpll, minpll;                    // Used to find whole number or numbers closest to whole numbers
  unsigned int maxms, minms;                      // Used to find whole number or numbers closest to whole numbers

  // initialize values
  minfraction = 1;
  maxfraction = 0;
  maxpll = 0;
  minpll = 0;
  minms = 0;
  maxms = 0;

  // The ValidateFrequency() call checks if frequency is below 1 Mhz or above 100 Mhz or above 150 Mhz.  See note above for frequencies below 1 Mhz or above 150 Mhz.  Frequencies
  // between 100 Mhz and 150 Mhz can be easily done using an integer multipler (i.e. use a fixed multipler of 6 - 6x100 Mhx is 600 Mhz which is inside PLL frequency requirement
  freq_temp = ValidateFrequency (freq);
  
  // For frequencies above 100 Mhz its easier to deal with because these frequencies are nice integers 
  // for example 100 Mhz x 6 = 600 Mhz, 150 Mhz x 6 = 900 Mhz which fits nicely into the PLL frequency range
  if (freq < 100000000) {
    // Scan through all possible dividers
    for (i = 8; i <= 900; i++) {
      // Calculate the fractional component and the integer component.
      fraction = (double)(i * (double)freq_temp / multisynth.Fxtal);
      pllmult = (unsigned long)fraction;                  

      // interger multiplier must be between 24 to 36. These numbers are based on 25 Mhz clock
      if (pllmult >= (SI_MIN_PLL_FREQ/multisynth.Fxtal) && pllmult <= (SI_MAX_PLL_FREQ/multisynth.Fxtal)) {
        // Calculate fraction. Do it inside this loop to make processing faster.
        if (fraction > 1.0) fraction -= pllmult;

        // Check for integer values
        if (fraction < 0.001) break;
        if (fraction > 0.9) break;
        
        // Check for a 1/2 value
        if (fraction <= 0.51 and fraction >= 0.49) break;
        
        // Check for the max and min values in case a integer cannot be found
        if (minfraction > fraction) {
          minfraction = fraction;
          minpll = pllmult;
          minms = i;
        }
        if (maxfraction < fraction) {
          maxfraction = fraction;
          maxpll = pllmult;
          maxms = i;
        }
      // if the multiplier is over 36 (based inb 25 Mhz Xtal) exit loop - we are done
      } else if (pllmult > (SI_MAX_PLL_FREQ/multisynth.Fxtal)) break;
    }

  // If frequency is above 100 but below 150 Mhz, can apply a multiplier of 6 - easy case
  } else if (freq >= 100000000 && freq <= SI_MAX_MS_FREQ) {
    i = 6;
    fraction = (double)(i * freq) / (double) multisynth.Fxtal;
    pllmult = (unsigned long)fraction;

  // if frequency is above 150 Mhz, then can apply multipler of 4 but must use MS_DIVBY4
  } else if (freq > SI_MAX_MS_FREQ) {
    i = 4;
    fraction = (double)(i * freq) / (double) multisynth.Fxtal;
    pllmult = (unsigned long)fraction;
    multisynth.MS_DIVBY4 = 0x3;

  }

  // Could not find a integer so use biggest or smallest fraction
  if (pllmult > (SI_MAX_PLL_FREQ/multisynth.Fxtal)) {
    if ( minfraction < (1.0 - maxfraction) ) {
      multisynth.PLL_a = minpll;
      multisynth.MS_a = minms;
      multisynth.MS_b = 0;
      multisynth.MS_c = 1;

    } else {
      multisynth.PLL_a = maxpll;
      multisynth.MS_a = maxms;
      multisynth.MS_b = 0;
      multisynth.MS_c = 1;

    }
    
  // Integer found!!
  } else {
    multisynth.PLL_a = pllmult;
    multisynth.MS_a = i;
    multisynth.MS_b = 0;
    multisynth.MS_c = 1;
  }

  // Calculate PLL frequency based on multiplier determined.
  multisynth.PLL_Fvco = multisynth.MS_a * freq_temp;

}

void UpdateDrive (unsigned char clk, unsigned char idrive)
{
// This routine does not enable the clock.  Its assumed that its been enabled elsewhere
// Note that drive needs to be either 2, 4, 6, or 8.  The UpdateDrive() converts to Register Coded Value.

  unsigned char mAdrive, clkreg;

  // The bit values that are written to the register is different from the
  // interger numbers.  For example 2ma, a value of 0 is written into bits 0 & 1
  // For 6mA, a value of 4 is written into bits 0 & 1 of clock control register
  // idrive is the interger value for drive current (i.e. 2, 4, 6 8 mA)
  // mAdrive is the actual value that is written to the clock control register
  // clkreg is the variable that has the actual data that will be written to the clock control register
  switch (idrive) {
    case 2:
      mAdrive = SI_CLK_2MA;
      break;
    case 4:
      mAdrive = SI_CLK_4MA;
      break;
    case 6:
      mAdrive = SI_CLK_6MA;
      break;
    case 8:
      mAdrive = SI_CLK_8MA;
      break;
    default:
      mAdrive = SI_CLK_8MA;
  }


  // Start with the prior ctl register then update the clk register for mA drive
  switch (clk) {
    case 0:
      clkreg = clk0ctl.reg;              // Get the old register value
      clk0ctl.mAdrive = idrive;          // save the current drive value
      clkreg &= SI_CLK_CLR_DRIVE;        // Clear the old mA drive bits in the register
      clkreg |= mAdrive;                 // set the bits for the new mA drive in the register
      clk0ctl.reg = clkreg;              // save the new register for future use
      break;

    case 1:
      clkreg = clk1ctl.reg;              // Same comments as above except for clk1
      clk1ctl.mAdrive = idrive;
      clkreg &= SI_CLK_CLR_DRIVE;
      clkreg |= mAdrive;
      clk1ctl.reg = clkreg;
      break;

    case 2:
      clkreg = clk2ctl.reg;              // Same comments as above except for clk2
      clk2ctl.mAdrive = idrive;
      clkreg &= SI_CLK_CLR_DRIVE;
      clkreg |= mAdrive;
      clk2ctl.reg = clkreg;
      break;
  }

  // Update clk control
  UpdateClkControlRegister (clk);
}

void InvertClk (unsigned char clk, unsigned char invert)
// This routine inverts the CLK0_INV bit in the clk control register.  
// When a sqaure wave is inverted, its the same as a 180 deg phase shift.
// This routine does not enable the clock.  Its assumed that its been enabled elsewhere
{

  unsigned char clkreg;

  // Start with the prior ctl register then update the clk register to invert the output
  switch (clk) {
    case 0:
      clkreg = clk0ctl.reg;                          // Retrieve the prior register value
      if (invert) clkreg |= SI_CLK_INVERT;           // Set the invert bit in register to be written 
      else clkreg &= ~SI_CLK_INVERT;                 // clear the invert bit
      clk0ctl.reg = clkreg;                          // save new register for future use
      break;

    case 1:
      clkreg = clk1ctl.reg;                          // same comments as above except for clk1
      if (invert) clkreg |= SI_CLK_INVERT;
      else clkreg &= ~SI_CLK_INVERT;
      clk1ctl.reg = clkreg;
      break;

    case 2:
      clkreg = clk2ctl.reg;                          // Same comments as above except for clk2
      if (invert) clkreg |= SI_CLK_INVERT;
      else clkreg &= ~SI_CLK_INVERT;
      clk2ctl.reg = clkreg;
      break;
  }

  // Update clk control
  UpdateClkControlRegister (clk);
}


void UpdatePhase (unsigned char clk, unsigned int phase)
// This routing changes the phase of the output frequency
// Phase shift is based on period of PLL that generates the output frequency.
// The phase shift register can be 0 to 127 (128 values). Each increment of 1 to the register delays the output by
// 1/4 of the period of the PLL Clock. For example, is PLL is 900 Mhz, each increment of 1 in register delays 
// output by 0.25/900Mhz or 1/900/4 or 0.28 ns.  So max phase delay for 900 Mhz is 128*0.28=35ns.  For a 7 Mhz output,
// this is same as a 88deg phase shift.  i.e. Period of 7Mhz is 142 ns and 35/142*360=88
// Note: In order for the phase to be applied, the PLL must be reset!!
{
  
  // First need to convert the provided phase angle to bit value that will configure the phase control register
  switch (clk) {
    case 0:
      // If an angle if provide that is greater that the max angle supported (see note above) need to indicate and error or correct
      // In this case, just set to the maximum
      if ( phase < clk0ctl.maxangle) {
        // Its below max angle then calculate value that can fit into phase control register (i.e. between 0 to 127) 
        clk0ctl.phase =  (4 * phase * (clk0ctl.PLLFreq / clk0ctl.freq)) / 360;
      } else clk0ctl.phase = clk0ctl.maxangle;
      break;

    // same as above except for clk1
    case 1:
      if ( phase < clk1ctl.maxangle) {
        clk1ctl.phase =  (4 * phase * (clk1ctl.PLLFreq / clk1ctl.freq)) / 360;
      } else clk1ctl.phase = clk1ctl.maxangle;
      break;

    // same as above except for clk2
    case 2:
      if ( phase < clk2ctl.maxangle) {
        clk2ctl.phase =  (4 * phase * (clk2ctl.PLLFreq / clk2ctl.freq)) / 360;
      } else clk2ctl.phase = clk2ctl.maxangle;
      break;
  }

// Update phase registers based on the defined phase value in clk?ctl.phase 
  UpdatePhaseControlRegister (clk);
  
// Reset PLLA (bit 5 set) & PLLB (bit 7 set)
// If PLL not reset, phase is not applied.
  Si5351WriteRegister (SIREG_177_PLL_RESET, SI_PLLA_RESET | SI_PLLB_RESET );
}


void UpdateClkControlRegister (unsigned char clk)
{
// This routine write the clock control register variable stored in the clock control strucutre to the
// Si5351 clock control register.  The register must be defined elsewhere.
// This routine does not enable the clock.  Its assumed that its been enabled elsewhere
  switch (clk) {
    case 0:
      Si5351WriteRegister (SIREG_16_CLK0_CTL, clk0ctl.reg);
      break;

    case 1:
      Si5351WriteRegister (SIREG_17_CLK1_CTL, clk1ctl.reg);
      break;

    case 2:
      Si5351WriteRegister (SIREG_18_CLK2_CTL, clk2ctl.reg);
      break;
  }
}

void UpdatePhaseControlRegister (unsigned char clk)
// This routine write the phase control data to the Si5351 phase register 
// This routine does not
//   1) enable the clock.  Its assumed that its been enabled elsewhere
//   2) define the phase.  The phase should be calculated elsewhere and stored in the clock contol structure
// This routing only writes the phase defined in the clock control structure to the phase control register
{
  switch (clk) {
    case 0:
      Si5351WriteRegister (SIREG_165_CLK0_PHASE_OFFSET, clk0ctl.phase);
      break;

    case 1:
      Si5351WriteRegister (SIREG_166_CLK1_PHASE_OFFSET, clk1ctl.phase);
      break;

    case 2:
      Si5351WriteRegister (SIREG_167_CLK2_PHASE_OFFSET, clk2ctl.phase);
      break;
  }
}

/*
Cannot use Wire Library because its Interupt based and need to change fequency from a timer (which is an interupt)
Interupts cannot be layered.  Its one at a time.

For I2C communication must be manually done
*/


void Si5351RepeatedWriteRegister(unsigned char  addr, unsigned char  bytes, unsigned char *data)
{
// I2C communication must be manually done. Wire library is Interupt based and cannot be used!!
  i2cSendRepeatedRegister(addr, bytes, data);
}


void Si5351WriteRegister (unsigned char reg, unsigned char value)
// Routine uses the I2C protcol to write data to the Si5351 register.
{
  unsigned char err;
  
// I2C communication must be manually done. Wire library is Interupt based and cannot be used!!
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

// I2C communication must be manually done. Wire library is Interupt based and cannot be used!!
  err=i2cReadRegister(reg, &value);  
  if (err) {
    Serial.print ("I2C R Err ");
    Serial.println (err);
    value = 0;
  }

  return value;
}
