/*******************************************************************************
*
* TITLE:    interrupts.h
*
* VERSION:  0.4 (Beta)
*
* DATE:   03-Jan-2008
*
* AUTHOR:   R. Kevin Watson
*           kevinw@jpl.nasa.gov
*
* COMMENTS: This file contains template interrupt initialization & handling
*           code for the IFI FRC robot controller.
*
*               This file best viewed with tabs set to four.
*
*           You are free to use this source code for any non-commercial
*           use. Please do not make copies of this source code, modified
*           or un-modified, publicly available on the internet or elsewhere
*           without permission. Thanks.
*
*           Copyright ©2004-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
* CHANGE LOG:
*
* DATE         REV  DESCRIPTION
* -----------  ---  ----------------------------------------------------------
* 22-Dec-2003  0.1  RKW Original
* 25-Feb-2004  0.2  RKW - Added the ability to clear the interrupt flag before
*                   enabling the interrupt.
* 24-Dec-2007  0.3  RKW - Modified code to be compatible with version 3.0+
*                   of the Microchip C18 compiler.
* 03-Jan-2008  0.4  RKW - Renamed all ISRs for consistancy across all
*                   modules of the new robot controller code.
*
*******************************************************************************/

#ifndef _interrupts_h
#define _interrupts_h

// Remove the comment slashes from one or more of the following lines to
// enable the respective external interrupt(s). By doing so, you only
// enable the code within interrupts.c to become part of your software
// build. For your software to be fully functional, you must also enable
// the interrupt(s) in ifi_frc.h.
// #define ENABLE_INT_1_ISR
// #define ENABLE_INT_2_ISR
// #define ENABLE_INT_3_6_ISR

// function prototypes
void Initialize_Int_1(void);    // initializes interrupt 1
void Initialize_Int_2(void);    // initializes interrupt 2
void Initialize_Int_3_6(void);  // initializes interrupts 3 through 6
void Int_1_ISR(void);           // external interrupt 1 service routine
void Int_2_ISR(void);           // external interrupt 2 service routine
void Int_3_ISR(unsigned char);  // external interrupt 3 service routine
void Int_4_ISR(unsigned char);  // external interrupt 4 service routine
void Int_5_ISR(unsigned char);  // external interrupt 5 service routine
void Int_6_ISR(unsigned char);  // external interrupt 6 service routine

#endif
