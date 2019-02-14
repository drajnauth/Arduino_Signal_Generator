# Arduino_Signal_Generator
Arduino programs that allows manipulation of the Si5351 to produce a poor man's Sig Gen. 

Its a work in progress and there are a few improvements and a few bugs to be fixed. Caveat emptor.

Videos available from my YouTube Channel: 
http://www.youtube.com/c/DaveVE3OOI 

There are three version of the Sig Gen software here:

1) My original (OLD) 20x2 LCD Sig Gen version: "VE3OOI_Si5351_Signal_Generator_v0.3". Lots of bugs in this version.  Needs the "LiquidCrystal_I2C (NewliquidCrystal)" library.  It uses my own functions to control the SI5351 and no additional library is needed.  However this verion is buggy and the Si5351 control is limited.


2) Interim release of the 40x4 LCD PARC Sig Gen version: "PARC_Si5351_Signal_Generator_E_v0.1a. Needs the "hd44780" LCD Library and the "Etherkit_Si5351-2.1.2" Si5351 library.  Limited functionality. The NT7S Si5351 library works between 4 Khz and 114 Mhz and supports I/Q between 5 Mhz and 60 Mhz (I hardcoded 60 Mhz even though it works to just under 100 Mhz).  This software is EXTREMELY large and I had to remove CLI support to make it fit. This is an interim release.


3) Final release of the 40x4 LCD PARC Sig Gen version: "PARC_Si5351_Signal_Generator_A_v0.1b". Needs the "hd44780" LCD Library. It uses my own functions to control the SI5351 and no additional library is needed. This version currently supports 1.5 Khz to 225 Mhz on CLK1 and 8Khz to 114 Mhz on CLK 0/2.  It provides quadature support on CLK0/CLK2 from 3 Mhz to 80 Mhz.  Around 80 Mhz the ouput is no longer in quadarture.


For the PARC 2019 Sig Gen Buildathon, use the "PARC_Si5351_Signal_Generator_A_v0.1b" version.  


Dave, VE300I
