/*******************************************************************************
*
*	TITLE:		encoder.h 
*
*	VERSION:	0.6 (Beta)                           
*
*	DATE:		15-Jan-2008
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This version is compatible with Microchip C18 3.0+ only.
*
*				This file best viewed with tabs set to four.
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or
*				elsewhere without permission. Thanks.
*
*				Copyright ©2003-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	20-Dec-2003  0.1  RKW - Original code.
*	18-Feb-2004  0.2  RKW - Reassigned the encoder digital inputs to run
*	                  on the FRC robot controller too.
*	01-Jan-2005  0.3  RKW - Get_Left_Encoder_Count(), Get_Right_Encoder_Count(),
*	                  Reset_Left_Encoder_Count() and Reset_Right_Encoder_Count()
*	                  functions added.
*	01-Jan-2005  0.3  RKW - Renamed Int_1_Handler() and Int_2_Handler() to
*	                  Left_Encoder_Int_Handler() and Right_Encoder_Int_Handler
*	                  respectively.
*	01-Jan-2005  0.3  RKW - Altered the interrupt service routines to easily
*	                  flip the direction the encoders count by altering the
*	                  RIGHT_ENCODER_TICK_DELTA and LEFT_ENCODER_TICK_DELTA
*	                  #defines found in encoder.h
*	06-Jan-2005  0.4  RKW - Rebuilt with C18 version 2.40.
*	17-Dec-2005  0.5  RKW - Added code to accomodate four more encoder on
*	                  interrupts 3 through 6. These additional encoder inputs
*	                  are optimized for position control.
*	13-Jan-2006  0.5  RKW - Verified code works properly on new PIC18F8722-
*	                  based robot controllers.
*	15-Jan-2008  0.6  RKW - renamed ISRs to be compatible with new robot
*	                  controller code; restructured the ENABLE_ENCODER_n
*	                  scheme to be compatible with new robot controller code.
*
*******************************************************************************/
#ifndef _encoder_h
#define _encoder_h

// Remove the comment slashes from one or more of the following lines to
// enable the respective encoder channel(s). By doing so, you only enable
// the code within encoder.c to become part of your software build. For 
// your software to be fully functional, you must also enable the
// interrupt(s) in ifi_frc.h.
// #define ENABLE_ENCODER_1		// also enable interrupt 1 in ifi_frc.h
// #define ENABLE_ENCODER_2		// also enable interrupt 2 in ifi_frc.h
// #define ENABLE_ENCODER_3		// also enable interrupt 3 in ifi_frc.h
// #define ENABLE_ENCODER_4		// also enable interrupt 4 in ifi_frc.h
// #define ENABLE_ENCODER_5		// also enable interrupt 5 in ifi_frc.h
// #define ENABLE_ENCODER_6		// also enable interrupt 6 in ifi_frc.h

// Digital input pin(s) assigned to the encoder's phase-B output. Make sure 
// this pin is configured as an input in teleop.c/Initialization().
#define ENCODER_1_PHASE_B_PIN	rc_dig_in11
#define ENCODER_2_PHASE_B_PIN	rc_dig_in12
#define ENCODER_3_PHASE_B_PIN	rc_dig_in13
#define ENCODER_4_PHASE_B_PIN	rc_dig_in14
#define ENCODER_5_PHASE_B_PIN	rc_dig_in15
#define ENCODER_6_PHASE_B_PIN	rc_dig_in16

// Change the sign of these if you need	to flip the way the encoders count. 
// For instance, if a given encoder count increases in the positive direction 
// when rotating counter-clockwise, but you want it to count in the negative 
// direction when rotating in the counter-clockwise direction, flip the sign 
// (i.e., change 1 to -1) and it'll work the way you need it to.
#define ENCODER_1_TICK_DELTA	1
#define ENCODER_2_TICK_DELTA	1
#define ENCODER_3_TICK_DELTA	1
#define ENCODER_4_TICK_DELTA	1
#define ENCODER_5_TICK_DELTA	1
#define ENCODER_6_TICK_DELTA	1

//
// If you modify stuff below this line, you'll break the software.
//

// #define ENABLE_ENCODER_3_6 if encoder 3, 4, 5 or 6 is enabled
#ifdef ENABLE_ENCODER_3
#ifndef ENABLE_ENCODER_3_6
#define ENABLE_ENCODER_3_6
#endif
#endif
#ifdef ENABLE_ENCODER_4
#ifndef ENABLE_ENCODER_3_6
#define ENABLE_ENCODER_3_6
#endif
#endif
#ifdef ENABLE_ENCODER_5
#ifndef ENABLE_ENCODER_3_6
#define ENABLE_ENCODER_3_6
#endif
#endif
#ifdef ENABLE_ENCODER_6
#ifndef ENABLE_ENCODER_3_6
#define ENABLE_ENCODER_3_6
#endif
#endif

// function prototypes
#ifdef ENABLE_ENCODER_1
void Initialize_Encoder_1(void);
long Get_Encoder_1_Count(void);
void Reset_Encoder_1_Count(void);
void Int_1_ISR(void);
#endif

#ifdef ENABLE_ENCODER_2
void Initialize_Encoder_2(void);
long Get_Encoder_2_Count(void);
void Reset_Encoder_2_Count(void);
void Int_2_ISR(void);
#endif

#ifdef ENABLE_ENCODER_3
void Initialize_Encoder_3_6(void);
long Get_Encoder_3_Count(void);
void Reset_Encoder_3_Count(void);
void Int_3_ISR(unsigned char);
#endif

#ifdef ENABLE_ENCODER_4
void Initialize_Encoder_3_6(void);
long Get_Encoder_4_Count(void);
void Reset_Encoder_4_Count(void);
void Int_4_ISR(unsigned char);
#endif

#ifdef ENABLE_ENCODER_5
void Initialize_Encoder_3_6(void);
long Get_Encoder_5_Count(void);
void Reset_Encoder_5_Count(void);
void Int_5_ISR(unsigned char);
#endif

#ifdef ENABLE_ENCODER_6
void Initialize_Encoder_3_6(void);
long Get_Encoder_6_Count(void);
void Reset_Encoder_6_Count(void);
void Int_6_ISR(unsigned char);
#endif


#endif
