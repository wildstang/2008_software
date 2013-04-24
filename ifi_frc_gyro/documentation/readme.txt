****************************************************************

You are free to use this source code for any non-commercial
use. Please do not make copies of this source code, modified
or un-modified, publicly available on the internet or
elsewhere without permission. Thanks.

Copyright ©2007-2008 R. Kevin Watson. All rights are reserved.

****************************************************************

This is experimental software and the author cannot assume any 
responsibility for its use (or misuse). If you are not confident
in your ability to use and modify this software, please use the
instructions and default code provided on the C-Bot CD included 
in your kit of parts. Support is available in the programming
forum at the Chief Delphi website at http://www.chiefdelphi.com.
Before seeking help, please read the various readme files located
in the documentation folder. This code is *not* supported by IFI
or FIRST.

This software was built using MPLAB 8.00 and C18 version 3.10.
Until Microchip fixes a few problems with header files, I wouldn't
suggest upgrading to any version later than 3.10.

Before upgrading your compiler, you might consider copying your
entire mcc18 directory to a backup directory. Once backed-up, you
can apply the free upgrade to 3.10 that can be found on Microchip's
website:

http://ww1.microchip.com/downloads/en/DeviceDoc/MPLAB-C18-Upgrade-doc-v3_10.exe

Once upgraded, you can switch between the two compilers by just
making sure the correct directory is named mcc18 at compile time.



This code can be built for 2004-2008 FRC robot controllers using
Microchip C18 versions 2.4 and 3.0+. Follow the steps for your
particular configuration:

Using a 2006-2008 (PIC18F8722) FRC robot controller and C18 3.1:

 1) Use the ifi_frc_8722_30.lib library file.
 2) Use the 18f8722.lkr linker script in your project.
 3) Set the target microcontroller to the PIC18F8722 by selecting
    "Configure" then "Select Device..." within the MPLAP IDE.  
 4) Make sure the #define USE_C18_30 is enabled (uncommented) and
    #define USE_C18_244 is disabled (commented out) in ifi_frc.h.

Using a 2004-2005 (PIC18F8520) FRC robot controller and C18 3.1:

 1) Use the ifi_frc_8520_30.lib library file.
 2) Use the 18f8520.lkr linker script in your project.
 3) Set the target microcontroller to the PIC18F8520 by selecting
    "Configure" then "Select Device..." within the MPLAP IDE.  
 4) Make sure the #define USE_C18_30 is enabled (uncommented) and
    #define USE_C18_24 is disabled (commented out) in ifi_frc.h.

Using a 2006-2008 (PIC18F8722) FRC robot controller and C18 2.4:

 1) Use the ifi_frc_8722_24.lib library file.
 2) Use the 18f8722.lkr linker script in your project.
 3) Set the target microcontroller to the PIC18F8722 by selecting
    "Configure" then "Select Device..." within the MPLAP IDE.  
 4) Make sure the #define USE_C18_24 is enabled (uncommented) and
    #define USE_C18_30 is disabled (commented out) in ifi_frc.h.

Using a 2004-2005 (PIC18F8520) FRC robot controller and C18 2.4:

 1) Use the ifi_frc_8520_30.lib library file.
 2) Use the 18f8520.lkr linker script in your project.
 3) Set the target microcontroller to the PIC18F8520 by selecting
    "Configure" then "Select Device..." within the MPLAP IDE.  
 4) Make sure the #define USE_C18_24 is enabled (uncommented) and
    #define USE_C18_30 is disabled (commented out) in ifi_frc.h.



By default all encoder channels are disabled. Perform these steps
to enable your encoder(s):

1) Enable the initialization routine(s) in teleop.c/Initialization().
2) Enable the individual encoder channels at the top of the encoder.h
   header file.
3) Enable the interrupt service routines associated with each encoder
   channel at the top of ifi_frc.h.
4) Make sure there are no conflicting interrupt service routines
   enabled at the top of interrupt.h. 
5) The code located in teleop.c/Teleop() can be used to verify proper
   operation of your encoder(s). For more information, read the 
   encoder.h and encoder_readme.txt files.

Perform these steps if you would like to remove all encoder software
from your project:

1) Delete all "#include encoder.h" lines from your source code.
   You'll find these at the top of disabled.c, autonomous.c,
   teleop.c and ifi_frc.c.
2) Remove the calls to the various encoder initialization
   functions in teleop.c/Initialization().
3) Remove the encoder.c and encoder.h files from your project.
4) Delete the encoder.c and encoder.h files from your build directory.
5) Unless you will be using them for other uses, disable all encoder
   related interrupt service routines at the top of ifi_frc.h



By default the analog to digital and gyro software is disabled.
Perform these steps to get your gyro working:

1) Enable the Initialize_ADC() and Initialize_Gyro() functions in
   teleop.c/Initialization().
2) Enable the timer 4 interrupt service routine in ifi_frc.h
3) Make sure timer 4 is disabled at the top of timers.h.
4) Depending on the operating modes you will be using the gyro,
   place calls to the Process_Gyro_Data() function in the disabled.c/
   Disabled_Spin(), autonomous.c/Autonomous_Spin() and/or teleop.c/
   Teleop_Spin() functions.
5) Add gyro bias calibration code that will execute before you
   use your gyro. Example calibration code has been placed in 
   disabled.c/Disabled() that can be used for the competition (this
   requires that you use a mode dongle to emulate the field
   controller, which will put you in disabled mode for a period of 
   time before transitioning to autonomous mode). For testing
   purposes you can also use the example code in teleop.c/Teleop()
   to make sure your gyro is working.
6) Follow the instructions in adc_readme.txt, adc.h, gyro_readme.txt,
   and gyro.h for information related to installation and calibration
   of your gyro.
7) If building for the 2004 or 2005 PIC18F8520 based robot controller,
   you will need to use the adc_8520.c source file instead of adc.c.
   The header file adc.h will work with either version.
8) If building for the 2004 or 2005 PIC18F8520 based robot controller,
   you will need to enable the ADC interrupt service routine in 
   ifi_frc.h

Perform these steps if you would like to remove all gyro software
from your project:

1) Delete all "#include adc.h" and "#include gyro.h" lines from your
   source code. You'll find these at the top of disabled.c, teleop.c
   autonomous.c, and ifi_frc.c.
2) Remove the calls to the gyro and ADC initialization functions in
   teleop.c/Initialization().
3) Delete the calls to the Process_Gyro_Data() function that may be
   located in the disabled.c/Disabled_Spin(), teleop.c/Teleop_Spin(), 
   and autonomous.c/Autonomous_Spin() functions.
4) Delete the calls to Start_Gyro_Bias_Calc() and Stop_Gyro_Bias_Calc()
   found in disabled.c/Disabled_Init(), autonomous.c/Autonomous_Init(),
   and possibly elsewhere.
5) Unless you will be using them for another use, disable the timer 4
   and ADC interrupt service routines at the top of ifi_frc.h.
6) Remove the adc.c, adc_8520.c, adc.h, gyro.c, and gyro.h files from
   your project.
7) Delete the adc.c, adc_8520.c, adc.h, gyro.c, and gyro.h files from
   your build directory.

R. Kevin Watson
kevinw@jpl.nasa.gov

