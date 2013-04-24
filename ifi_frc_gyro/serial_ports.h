/*******************************************************************************
*
*	TITLE:		serial_ports.c 
*
*	VERSION:	0.6 (Beta)                           
*
*	DATE:		21-Jan-2008
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	The source code in this file implements a fully buffered,
*				interrupt-driven serial port driver that can be used with
*				the on-board serial ports.
*
*				This version is compatible with Microchip C18 3.0+ only.
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or
*				elsewhere without permission. Thanks.
*
*				Copyright ©2004-2008 R. Kevin Watson. All rights are reserved.
*		
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	22-Dec-2004  0.1  RKW - Original code.
*	28-Dec-2004  0.2  RKW - Using preprocessor directives, added the ability 
*	                  to enable/disable individual serial port receive and 
*	                  transmit code. Updated documentation.
*	05-Feb-2005  0.3  RKW - Added _user_putc() interface to C18's new output
*	                  stream capabilities. Updated documentation.
*	05-Jan-2006  0.4  RKW - Partial port to 18F8722. Updated documentation.
*	10-Jan-2006  0.4  RKW - Modified the #pragma interruptlow line to also
*	                  save the .tmpdata section.
*	01-Jan-2008  0.5  RKW - Modified code to accept a baud rate when calling
*	                  the Init_Serial_Port_One() and Init_Serial_Port_Two()
*	                  functions.
*	21-Jan-2008  0.6  RKW - Modified code to use the universal device header
*	                  file found in ifi_frc.h, but found and corrected a
*	                  problem with the definition of the SENDB and ADDEN bits.
*
*******************************************************************************/
#ifndef _serial_ports_h
#define _serial_ports_h

#include <stdio.h>

// comment out the next line to disable serial port one 
// receive functionality
#define ENABLE_SERIAL_PORT_ONE_RX

// comment out the next line to disable serial port one 
// transmit functionality
#define ENABLE_SERIAL_PORT_ONE_TX

// comment out the next line to disable serial port two 
// receive functionality
#define ENABLE_SERIAL_PORT_TWO_RX

// comment out the next line to disable serial port two 
// transmit functionality
#define ENABLE_SERIAL_PORT_TWO_TX

// predefined baud rate values for use with the 
// Init_Serial_Port_One() and Init_Serial_Port_Two()
// functions 
#define BAUD_4800	1
#define BAUD_9600	2
#define BAUD_14400	3
#define BAUD_19200	4
#define BAUD_38400	5
#define BAUD_57600	6
#define BAUD_115200	7
#define BAUD_230400	8


// These values define the size, in bytes, of the four circular queues used to 
// buffer incoming and outgoing serial port data. The size of the receive queue 
// is critical to prevent data loss. As an example, if you're continuously 
// receiving data at 38,400 baud and only call Read_Serial_Port_xxx() in the 
// Process_Data_From_Master_uP() loop, which is called every 26.2ms, you'll 
// need to size the queue to accept all of the data you expect to receive in 
// those 26.2ms. To do this, multiply the rate at which bytes of data are being 
// received times the amount of time between calls to Read_Serial_Port_xxx(). 
// In the above example, we're receiving data at a rate of 3840 bytes per second 
// (8 bits of data plus 2 bits of overhead per byte transferred) and we need to 
// store at least 26.2ms-worth of that data for a calculated queue size of 101 
// bytes. Because the queue size must be a power of two, we'll need to round-up 
// to a queue size of 128. Another solution is to check for received serial data 
// at a higher rate by putting the call to Read_Serial_Port_xxx() in the much 
// faster Process_Data_From_Local_IO() loop. As mentioned above, these values 
// must be a power of two (i.e.,8,16,32,64,128) for the circular queue algorithm 
// to function correctly.
#define RX_1_QUEUE_SIZE 32
#define TX_1_QUEUE_SIZE 32
#define RX_2_QUEUE_SIZE 32
#define TX_2_QUEUE_SIZE 32

// The circular queue algorithm will break if these values are altered.
#define RX_1_QUEUE_INDEX_MASK RX_1_QUEUE_SIZE-1
#define TX_1_QUEUE_INDEX_MASK TX_1_QUEUE_SIZE-1
#define RX_2_QUEUE_INDEX_MASK RX_2_QUEUE_SIZE-1
#define TX_2_QUEUE_INDEX_MASK TX_2_QUEUE_SIZE-1

#ifndef FALSE
#define TRUE 1
#define FALSE 0
#endif

// #defines used with the stdout_serial_port global variable
#define NUL 0
#define SERIAL_PORT_ONE 1
#define SERIAL_PORT_TWO 2

// if needed, declare functions and global variables that
// are specific to serial port one receiver functionality
#ifdef ENABLE_SERIAL_PORT_ONE_RX
void Init_Serial_Port_One(unsigned char);
unsigned char Serial_Port_One_Byte_Count(void);
unsigned char Read_Serial_Port_One(void);
void Rx_1_ISR(void);
extern volatile unsigned char RX_1_Framing_Errors;
extern volatile unsigned char RX_1_Overrun_Errors;
#endif

// if needed, declare functions that are specific to serial
// port one transmit functionality
#ifdef ENABLE_SERIAL_PORT_ONE_TX
extern unsigned char stdout_serial_port;
void _user_putc(unsigned char);
void Init_Serial_Port_One(unsigned char);
void Write_Serial_Port_One(unsigned char);
void Tx_1_ISR(void);
#endif

// if needed, declare functions and global variables that
// are specific to serial port two receiver functionality
#ifdef ENABLE_SERIAL_PORT_TWO_RX
void Init_Serial_Port_Two(unsigned char);
unsigned char Serial_Port_Two_Byte_Count(void);
unsigned char Read_Serial_Port_Two(void);
void Rx_2_ISR(void);
extern volatile unsigned char RX_2_Framing_Errors;
extern volatile unsigned char RX_2_Overrun_Errors;
#endif

// if needed, declare functions that are specific to serial
// port two transmit functionality
#ifdef ENABLE_SERIAL_PORT_TWO_TX
extern unsigned char stdout_serial_port;
void _user_putc(unsigned char);
void Init_Serial_Port_Two(unsigned char);
void Write_Serial_Port_Two(unsigned char);
void Tx_2_ISR(void);
#endif

#endif
