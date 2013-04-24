/*******************************************************************************
* FILE NAME: ifi_code.h
*
* DESCRIPTION: 
*  This is the include file which corresponds to ifi_code.c
*  It contains some aliases and function prototypes used in that file.
*
*******************************************************************************/
#ifndef _ifi_code_h
#define _ifi_code_h

// used in limit switch routines in ifi_code.c
#define OPEN        1     // limit switch is open (input is floating high)
#define CLOSED      0     // limit switch is closed (input connected to ground)

// function prototypes for routines in ifi_code.c
unsigned int Get_Analog_Value(unsigned char channel);
void Default_Routine(void);
void Update_OI_LEDs(void);
void Limit_Switch_Max(unsigned char, unsigned char*);
void Limit_Switch_Min(unsigned char, unsigned char*);
unsigned char Limit_Mix (int);

#endif
