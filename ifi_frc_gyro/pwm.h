/*******************************************************************************
*
*	TITLE:		pwm.h 
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
#ifndef _pwm_h
#define _pwm_h

// These values define how much each respective PWM output 
// pulse will increase or decrease relative to the center/
// neutral pulse width (defined below) for each PWM step.
// The default value of fifty provides for a pulse width
// range of 1.28 ms (256*0.000005=0.00128), which is the
// same provided by Generate_Pwms(). If you're using
// servos, you should consider decreasing the gain to
// 40 or less. 
#define PWM_13_GAIN   50	// 5.0 microseconds per step
#define PWM_14_GAIN   50	// 5.0 microseconds per step
#define PWM_15_GAIN   50	// 5.0 microseconds per step
#define PWM_16_GAIN   50	// 5.0 microseconds per step

// These values define how wide each respective center/
// neutral pulse is with a PWM input value of 127. The 
// default value of 15000 provides for a pulse width of 
// 1.5 ms, which is the same provided by Generate_Pwms().
#define PWM_13_CENTER 15000 // 1.5 milliseconds
#define PWM_14_CENTER 15000 // 1.5 milliseconds
#define PWM_15_CENTER 15000 // 1.5 milliseconds
#define PWM_16_CENTER 15000 // 1.5 milliseconds

#define HIBYTE(value) ((unsigned char)(((unsigned int)(value)>>8)&0xFF))
#define LOBYTE(value) ((unsigned char)(value))

// function prototypes
void Initialize_PWM(void);
void PWM(unsigned char,unsigned char,unsigned char,unsigned char);

#endif
