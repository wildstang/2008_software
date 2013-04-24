/*******************************************************************************
*
*	TITLE:		eeprom.h 
*
*	VERSION:	0.2 (Beta)                           
*
*	DATE:		10-Jan-2005
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright ©2005-2006 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	08-Oct-2005  0.1  RKW Original
*	18-Oct-2005  0.2  RKW - Modified code to globally disable interrupts
*	                  only during the pre-write sequence.
*	10-Jan-2006  0.2  RKW - Verified code works with PIC18F8722.
*
*******************************************************************************/
#ifndef _EEPROM_H
#define _EEPROM_H

/*********** Wildstang EEPROM Addresses ***********/
/* Addresses used for calibration */
#define ADDR_KNOWN_BYTE1            0
#define ADDR_KNOWN_BYTE2            1

#define ADDR_DATA_BITMASK_CRAB      2
#define ADDR_DATA_BITMASK_ARM       3
#define ADDR_DATA_BITMASK_3         4
#define ADDR_DATA_BITMASK_4         5

#define ADDR_F_CRAB_LEFT            6
#define ADDR_F_CRAB_MID             7
#define ADDR_F_CRAB_RIGHT           8
#define ADDR_B_CRAB_LEFT            9
#define ADDR_B_CRAB_MID             10
#define ADDR_B_CRAB_RIGHT           11

#define ADDR_SHOULDER_F_HI          12
#define ADDR_SHOULDER_F_LO          13
#define ADDR_SHOULDER_B_HI          14
#define ADDR_SHOULDER_B_LO          15
#define ADDR_ELBOW_TOP_HI           16
#define ADDR_ELBOW_TOP_LO           17
#define ADDR_ELBOW_BOT_HI           18
#define ADDR_ELBOW_BOT_LO           19
#define ADDR_ROTATE_L_HI            20
#define ADDR_ROTATE_L_LO            21
#define ADDR_ROTATE_R_HI            22
#define ADDR_ROTATE_R_LO            23

#define ADDR_SHOULDER_HORIZ_HI      24
#define ADDR_SHOULDER_HORIZ_LO      25
#define ADDR_ELBOW_HORIZ_HI         26
#define ADDR_ELBOW_HORIZ_LO         27
#define ADDR_ROTATE_HORIZ_HI        28
#define ADDR_ROTATE_HORIZ_LO        29

/*********** Wildstang EEPROM Addresses ***********/


// This value defines the size of the circular queue used
// to buffer EEPROM data to be written. This value must be
// a power of two (i.e., 2, 4, 8, 16, 32,) for the circular 
// queue algorithm to function correctly.
#define EEPROM_QUEUE_SIZE 32

// Modifying stuff below this line will break the software

#define EEPROM_QUEUE_INDEX_MASK EEPROM_QUEUE_SIZE - 1;

#define HIBYTE(value) ((unsigned char)(((unsigned int)(value)>>8)&0xFF))
#define LOBYTE(value) ((unsigned char)(value))

#ifndef FALSE
#define TRUE 1
#define FALSE 0
#endif

// function prototypes
unsigned char EEPROM_Read(unsigned int);
unsigned char EEPROM_Write(unsigned int, unsigned char);
void EEPROM_Write_Handler(void);
unsigned char EEPROM_Queue_Free_Space(void);

#endif
