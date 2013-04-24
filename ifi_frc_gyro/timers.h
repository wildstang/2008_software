/*******************************************************************************
*
*	TITLE:		timers.c 
*
*	VERSION:	0.2 (Beta)                           
*
*	DATE:		03-Jan-2008
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This file contains template timer initialization & interrupt
*				handling code for the IFI FRC robot controller.
*
*				This version is compatible with Microchip C18 3.0+ only.
*
*               This file best viewed with tabs set to four.
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright ©2007-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	Change log:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	24-Dec-2007  0.1  RKW Original
*	03-Jan-2008  0.2  RKW - Renamed all ISRs for consistancy across all
*	                  modules of the new robot controller code.
*
*******************************************************************************/
#ifndef _timers_h
#define _timers_h

// Remove the comment slashes from one or more of the following lines
// to enable the respective timer(s). By doing so, you only enable the
// code within timers.c to become part of your software build. For your
// software to be fully functional, you must also enable the timer(s)
// in ifi_frc.h.
#define ENABLE_TIMER_0_ISR
#define ENABLE_TIMER_1_ISR
#define ENABLE_TIMER_2_ISR
// #define ENABLE_TIMER_3_ISR // used by the PWM software
// #define ENABLE_TIMER_4_ISR // used by the ADC software

//
// If you modify stuff below this line, you'll break the software.
//

// function prototypes
void Initialize_Timer_0(void);	// initializes the timer 0 hardware
void Initialize_Timer_1(void);	// initializes the timer 1 hardware
void Initialize_Timer_2(void);	// initializes the timer 2 hardware
void Initialize_Timer_3(void);	// initializes the timer 3 hardware
void Initialize_Timer_4(void);	// initializes the timer 4 hardware
void Timer_0_ISR(void);			// timer 0 interrupt service routine
void Timer_1_ISR(void);			// timer 1 interrupt service routine
void Timer_2_ISR(void);			// timer 2 interrupt service routine
void Timer_3_ISR(void);			// timer 3 interrupt service routine
void Timer_4_ISR(void);			// timer 4 interrupt service routine

#endif // _timers_h
