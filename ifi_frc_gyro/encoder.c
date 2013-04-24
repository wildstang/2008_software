/*******************************************************************************
*
*	TITLE:		encoder.c 
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
*				Copyright �2003-2008 R. Kevin Watson. All rights are reserved.
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

#include "ifi_frc.h"
#include "encoder.h"

// These variables are used to keep track of the encoder position over time.
// Though these are global variables, they shouldn't be modified directly. 
// Functions to modify these variables are included below.
#ifdef ENABLE_ENCODER_1
volatile long Encoder_1_Count = 0;
#endif

#ifdef ENABLE_ENCODER_2
volatile long Encoder_2_Count = 0;
#endif

#ifdef ENABLE_ENCODER_3
unsigned char Encoder_3_State;
volatile long Encoder_3_Count = 0;
#endif

#ifdef ENABLE_ENCODER_4
unsigned char Encoder_4_State;
volatile long Encoder_4_Count = 0;
#endif

#ifdef ENABLE_ENCODER_5
unsigned char Encoder_5_State;
volatile long Encoder_5_Count = 0;
#endif

#ifdef ENABLE_ENCODER_6
unsigned char Encoder_6_State;
volatile long Encoder_6_Count = 0;
#endif

/*******************************************************************************
*
*	FUNCTION:		Initialize_Encoder_1()
*
*	PURPOSE:		Initializes encoder #1.
*
*	CALLED FROM:	teleop.c/Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_1
void Initialize_Encoder_1(void)  
{
	// make sure interrupt 1 is configured as an input
	TRISBbits.TRISB2 = 1;

	// interrupt 1 is low priority
	INTCON3bits.INT2IP = 0;

	// trigger on rising edge
	INTCON2bits.INTEDG2 = 1;

	// make sure the interrupt flag is reset before enabling
	INTCON3bits.INT2IF = 0;

	// enable interrupt 1
	INTCON3bits.INT2IE = 1;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Initialize_Encoder_2()
*
*	PURPOSE:		Initializes encoder #2.
*
*	CALLED FROM:	teleop.c/Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_2
void Initialize_Encoder_2(void)  
{
	// make sure interrupt 2 is configured as an input
	TRISBbits.TRISB3 = 1;

	// interrupt 2 is low priority
	INTCON2bits.INT3IP = 0;

	// trigger on rising edge
	INTCON2bits.INTEDG3 = 1;

	// make sure the interrupt flag is reset before enabling
	INTCON3bits.INT3IF = 0;

	// enable interrupt 2
	INTCON3bits.INT3IE = 1;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Initialize_Encoder_3_6()
*
*	PURPOSE:		Initializes encoders 3 through 6.
*
*	CALLED FROM:	teleop.c/Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_3_6
void Initialize_Encoder_3_6(void)
{
	// make sure interrupts 3 through 6 are configured as inputs
	TRISBbits.TRISB4 = 1;
	TRISBbits.TRISB5 = 1;
	TRISBbits.TRISB6 = 1;
	TRISBbits.TRISB7 = 1;

	// interrupts 3 through 6 are low priority
  	INTCON2bits.RBIP = 0;

	// make sure the interrupt flag is reset before enabling
	INTCONbits.RBIF = 0;

	// enable interrupts 3 through 6
	INTCONbits.RBIE = 1;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_1_Count()
*
*	PURPOSE:		Gets the current number of encoder 1 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 1 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_1
long Get_Encoder_1_Count(void)
{
	long count;

	// Since we're about to access the Encoder_1_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_1_Count variable doesn't get altered while
	// we're using it.
	INTCON3bits.INT2IE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_1_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCON3bits.INT2IE = 1;

	// Return the encoder count to the caller.
	return(count);
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_1_Count()
*
*	PURPOSE:		Resets the encoder 1 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_1
void Reset_Encoder_1_Count(void)
{
	// Since we're about to access the Encoder_1_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_1_Count variable doesn't get altered while
	// we're using it.
	INTCON3bits.INT2IE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_1_Count = 0;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCON3bits.INT2IE = 1;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_1_ISR()
*
*	PURPOSE:		If enabled, the encoder 1 interrupt handler is called when
*					the interrupt 1 pin changes from a logic 0 to a logic 1.
*
*	CALLED FROM:	ifi_frc.c/Interrupt_Handler_Low()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_1
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata low_isr_tmpdata
#endif
void Int_1_ISR(void)
{
	// Encoder 1's phase a signal just transitioned from zero to one, causing 
	// this interrupt service routine to be called. We know that the encoder 
	// just rotated one count or "tick" so now check the logical state of the 
	// phase b signal to determine which way the the encoder shaft rotated.
	if(ENCODER_1_PHASE_B_PIN == 0)
	{
		Encoder_1_Count -= ENCODER_1_TICK_DELTA;
	}
	else
	{
		Encoder_1_Count += ENCODER_1_TICK_DELTA;
	}
}
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata
#endif
#endif

/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_2_Count()
*
*	PURPOSE:		Gets the current number of encoder 2 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 2 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_2
long Get_Encoder_2_Count(void)
{
	long count;

	// Since we're about to access the Encoder_2_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_2_Count variable doesn't get altered while
	// we're using it.
	INTCON3bits.INT3IE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_2_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCON3bits.INT3IE = 1;

	// Return the encoder count to the caller.
	return(count);
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_2_Count()
*
*	PURPOSE:		Resets the encoder 2 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_2
void Reset_Encoder_2_Count(void)
{
	// Since we're about to access the Encoder_2_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_2_Count variable doesn't get altered while
	// we're using it.
	INTCON3bits.INT3IE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_2_Count = 0;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCON3bits.INT3IE = 1;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_2_ISR()
*
*	PURPOSE:		If enabled, the encoder 2 interrupt handler is called when
*					the interrupt 2 pin changes from a logic 0 to a logic 1.
*
*	CALLED FROM:	ifi_frc.c/Interrupt_Handler_Low()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_2
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata low_isr_tmpdata
#endif
void Int_2_ISR(void)
{
	// Encoder 2's phase a signal just transitioned from zero to one, causing 
	// this interrupt service routine to be called. We know that the encoder 
	// just rotated one count or "tick" so now check the logical state of the 
	// phase b signal to determine which way the the encoder shaft rotated.
	if(ENCODER_2_PHASE_B_PIN == 0)
	{
		Encoder_2_Count -= ENCODER_2_TICK_DELTA;
	}
	else
	{
		Encoder_2_Count += ENCODER_2_TICK_DELTA;
	}
}
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata
#endif
#endif

/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_3_Count()
*
*	PURPOSE:		Gets the current number of encoder 3 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 3 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_3
long Get_Encoder_3_Count(void)
{
	long count;

	// Since we're about to access the Encoder_3_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_3_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_3_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCONbits.RBIE = 1;

	// Return the encoder count to the caller.
	return(count);
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_3_Count()
*
*	PURPOSE:		Resets the encoder 3 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_3
void Reset_Encoder_3_Count(void)
{
	// Since we're about to access the Encoder_3_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_3_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_3_Count = 0;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCONbits.RBIE = 1;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_3_ISR()
*
*	PURPOSE:		If enabled, the encoder 3 interrupt handler is called when
*					the interrupt 3 pin changes logic state
*
*	CALLED FROM:	ifi_frc.c/Interrupt_Handler_Low()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_3
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata low_isr_tmpdata
#endif
void Int_3_ISR(unsigned char state)
{
	// Encoder 3's phase a signal just changed logic level, causing this 
	// interrupt service routine to be called.
	if(state == 1)
	{
		// rising-edge interrupt
		if(ENCODER_3_PHASE_B_PIN == 0)
		{
			// backward
			if(Encoder_3_State == 1)
			{
				Encoder_3_Count -= ENCODER_3_TICK_DELTA;
			}
		}
		else
		{
			// forward
			if(Encoder_3_State == 0)
			{
				Encoder_3_Count += ENCODER_3_TICK_DELTA;
			}
		}
	}
	else
	{
		// falling-edge interrupt
		//   phase b is zero if moving forward
		//   phase b is one if moving backward
		Encoder_3_State = ENCODER_3_PHASE_B_PIN;
	}
}
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata
#endif
#endif

/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_4_Count()
*
*	PURPOSE:		Gets the current number of encoder 4 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 4 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_4
long Get_Encoder_4_Count(void)
{
	long count;

	// Since we're about to access the Encoder_4_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_4_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_4_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCONbits.RBIE = 1;

	// Return the encoder count to the caller.
	return(count);
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_4_Count()
*
*	PURPOSE:		Resets the encoder 4 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_4
void Reset_Encoder_4_Count(void)
{
	// Since we're about to access the Encoder_4_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_4_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_4_Count = 0;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCONbits.RBIE = 1;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_4_ISR()
*
*	PURPOSE:		If enabled, the encoder 4 interrupt handler is called when
*					the interrupt 4 pin changes logic state
*
*	CALLED FROM:	ifi_frc.c/Interrupt_Handler_Low()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_4
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata low_isr_tmpdata
#endif
void Int_4_ISR(unsigned char state)
{
	// Encoder 4's phase a signal just changed logic level, causing this 
	// interrupt service routine to be called.
	if(state == 1)
	{
		// rising-edge interrupt
		if(ENCODER_4_PHASE_B_PIN == 0)
		{
			// backward
			if(Encoder_4_State == 1)
			{
				Encoder_4_Count -= ENCODER_4_TICK_DELTA;
			}
		}
		else
		{
			// forward
			if(Encoder_4_State == 0)
			{
				Encoder_4_Count += ENCODER_4_TICK_DELTA;
			}
		}
	}
	else
	{
		// falling-edge interrupt
		//   phase b is zero if moving forward
		//   phase b is one if moving backward
		Encoder_4_State = ENCODER_4_PHASE_B_PIN;
	}
}
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata
#endif
#endif

/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_5_Count()
*
*	PURPOSE:		Gets the current number of encoder 5 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 5 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_5
long Get_Encoder_5_Count(void)
{
	long count;

	// Since we're about to access the Encoder_5_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_5_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_5_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCONbits.RBIE = 1;

	// Return the encoder count to the caller.
	return(count);
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_5_Count()
*
*	PURPOSE:		Resets the encoder 5 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_5
void Reset_Encoder_5_Count(void)
{
	// Since we're about to access the Encoder_5_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_5_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_5_Count = 0;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCONbits.RBIE = 1;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_5_ISR()
*
*	PURPOSE:		If enabled, the encoder 5 interrupt handler is called when
*					the interrupt 5 pin changes logic state
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_5
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata low_isr_tmpdata
#endif
void Int_5_ISR(unsigned char state)
{
	// Encoder 5's phase a signal just changed logic level, causing this 
	// interrupt service routine to be called.
	if(state == 1)
	{
		// rising-edge interrupt
		if(ENCODER_5_PHASE_B_PIN == 0)
		{
			Encoder_5_Count -= ENCODER_5_TICK_DELTA;
		}
		else
		{
			Encoder_5_Count += ENCODER_5_TICK_DELTA;
		}
	}
	else
	{
		// falling-edge interrupt
		if(ENCODER_5_PHASE_B_PIN == 0)
		{
			Encoder_5_Count += ENCODER_5_TICK_DELTA;
		}
		else
		{
			Encoder_5_Count -= ENCODER_5_TICK_DELTA;
		}
	}
}
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata
#endif
#endif


/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_6_Count()
*
*	PURPOSE:		Gets the current number of encoder 6 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 6 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_6
long Get_Encoder_6_Count(void)
{
	long count;

	// Since we're about to access the Encoder_6_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_6_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_6_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCONbits.RBIE = 1;

	// Return the encoder count to the caller.
	return(count);
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_6_Count()
*
*	PURPOSE:		Resets the encoder 6 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_6
void Reset_Encoder_6_Count(void)
{
	// Since we're about to access the Encoder_6_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_6_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_6_Count = 0;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCONbits.RBIE = 1;
}
#endif

/*******************************************************************************
*
*	FUNCTION:		Int_6_ISR()
*
*	PURPOSE:		If enabled, the encoder 6 interrupt handler is called when
*					the interrupt 6 pin changes logic state
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
#ifdef ENABLE_ENCODER_6
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata low_isr_tmpdata
#endif
void Int_6_ISR(unsigned char state)
{
	// Encoder 6's phase a signal just changed logic level, causing this 
	// interrupt service routine to be called.
	if(state == 1)
	{
		// rising-edge interrupt
		if(ENCODER_6_PHASE_B_PIN == 0)
		{
			Encoder_6_Count -= ENCODER_6_TICK_DELTA;
		}
		else
		{
			Encoder_6_Count += ENCODER_6_TICK_DELTA;
		}
	}
	else
	{
		// falling-edge interrupt
		if(ENCODER_6_PHASE_B_PIN == 0)
		{
			Encoder_6_Count += ENCODER_6_TICK_DELTA;
		}
		else
		{
			Encoder_6_Count -= ENCODER_6_TICK_DELTA;
		}
	}
}
#ifdef USE_ALTERNATE_TMPDATA
#pragma tmpdata
#endif
#endif
