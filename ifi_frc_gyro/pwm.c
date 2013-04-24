/*******************************************************************************
*
*	TITLE:		pwm.c
*
*	VERSION:	0.2 (Beta)                           
*
*	DATE:		19-Jan-2008
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This file contains CCP based precision PWM generation code
*				for the IFI FRC robot controller. Additional information can
*				be found in the pwm_readme.txt file.
*
*				This software uses timer 3. If this conflicts with your
*				own software, it can be easily modified to use timer 1
*				instead. 
*
*				This file best viewed with tabs set to four.
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright ©2006-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	29-Dec-2006  0.1  RKW - Original code.
*	19-Jan-2008  0.2  RKW - Modified PWM() to execute faster by using unsigned
*	                  interger calculations instead of signed integer. 
*
*******************************************************************************/

#include "ifi_frc.h"
#include "pwm.h"

/*******************************************************************************
*
*	FUNCTION:		Initialize_PWM()
*
*	PURPOSE:		CCP and timer initialization				
*
*	CALLED FROM:	PWM()
*
*	PARAMETERS:		none
*
*	RETURNS:		nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Initialize_PWM(void)
{
	// select 16-bit read/writes to timer 3
	T3CONbits.RD16 = 1;

	// CCP2 through CCP5 will use timer 3 for compare mode
	T3CONbits.T3CCP1 = 1;
	T3CONbits.T3CCP2 = 0;

	// use a 1:1 prescale for timer 3
	T3CONbits.T3CKPS1 = 0;
	T3CONbits.T3CKPS0 = 0;

	// use the internal 10MHz clock for timer 3 (each timer "tick" equals 100ns)
	T3CONbits.TMR3CS = 0;

	// disable the timer 3 overflow interrupt
	PIE2bits.TMR3IE = 0;

	// disable the CCP interrupts
	PIE2bits.CCP2IE = 0;
	PIE3bits.CCP3IE = 0;
	PIE3bits.CCP4IE = 0;
	PIE3bits.CCP5IE = 0;	
}

/*******************************************************************************
*
*	FUNCTION:		PWM()
*
*	PURPOSE:		Replacement for IFI's Generate_Pwms() function						
*
*	CALLED FROM:	ifi_frc.c/main()
*
*	PARAMETERS:		Four unsigned char PWM position/velocity values
*					for PWM outputs 13, 14, 15 and 16.
*
*	RETURNS:		nothing
*
*	COMMENTS:
*
*******************************************************************************/
void PWM(unsigned char pwm_13, unsigned char pwm_14, unsigned char pwm_15, unsigned char pwm_16)
{
	unsigned int temp_pwm_13;
	unsigned int temp_pwm_14;
	unsigned int temp_pwm_15;
	unsigned int temp_pwm_16;

	// stop timer 3
	T3CONbits.TMR3ON = 0;

	// initialize timer 3 register to zero
	TMR3H = 0;
	TMR3L = 0;

	// reset CCP hardware
	CCP2CON = 0;
	CCP3CON = 0;
	CCP4CON = 0;
	CCP5CON = 0;

	// calculate the number of 100 ns timer ticks 
	// needed to match the desired PWM pulse width 
	temp_pwm_13 = ((unsigned int)pwm_13 * PWM_13_GAIN) + PWM_13_CENTER - ((unsigned int)127 * PWM_13_GAIN);
	temp_pwm_14 = ((unsigned int)pwm_14 * PWM_14_GAIN) + PWM_14_CENTER - ((unsigned int)127 * PWM_14_GAIN);
	temp_pwm_15 = ((unsigned int)pwm_15 * PWM_15_GAIN) + PWM_15_CENTER - ((unsigned int)127 * PWM_15_GAIN);
	temp_pwm_16 = ((unsigned int)pwm_16 * PWM_16_GAIN) + PWM_16_CENTER - ((unsigned int)127 * PWM_16_GAIN);

	// load the CCP compare registers
	CCPR2L = LOBYTE(temp_pwm_13);
	CCPR2H = HIBYTE(temp_pwm_13);

	CCPR3L = LOBYTE(temp_pwm_14);
	CCPR3H = HIBYTE(temp_pwm_14);

	CCPR4L = LOBYTE(temp_pwm_15);
	CCPR4H = HIBYTE(temp_pwm_15);

	CCPR5L = LOBYTE(temp_pwm_16);
	CCPR5H = HIBYTE(temp_pwm_16);

	// disable all interrupts to prevent an interrupt routine
	// from executing after the CCP hardware is initialized
	// and the moment the timer is started
	INTCONbits.GIEH = 0;

	// setup CCP hardware for compare mode (each PWM output 
	// transitions from low to high at this point) 
	CCP2CON = 0x09;
	CCP3CON = 0x09;
	CCP4CON = 0x09;
	CCP5CON = 0x09;

	// start timer 3
	T3CONbits.TMR3ON = 1;

	//enable interrupts
	INTCONbits.GIEH = 1;
}
