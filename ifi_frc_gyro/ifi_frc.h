/*******************************************************************************
*
*	TITLE:		ifi_frc.h 
*
*	VERSION:	0.4 (Beta)                           
*
*	DATE:		21-Jan-2008
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This file contains the startup and interrupt service routine
*				dispatch code for the IFI FRC robot controller. 
*
*				This file best viewed with tabs set to four.
*
********************************************************************************
*
*	Change log:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	22-Dec-2007  0.1  RKW Original
*	24-Dec-2007  0.2  RKW - Added memory initialization code to _startup().
*	31-Dec-2007  0.3  RKW - Added calls to Autonomous_Init() and Teleop_Init().
*	21-Jan-2008  0.4  RKW - Added support for a ADC ISR
*
*******************************************************************************/
#ifndef _ifi_frc_h
#define _ifi_frc_h

#include <p18cxxx.h>

// Remove the comment slashes from one of the two following
// lines to select the C18 version we're building with.
   #define USE_C18_30 		// we're building with C18 3.0+
// #define USE_C18_24		// we're building with C18 2.4


// Remove the comment slashes from one or more of the following lines to
// enable the respective external interrupt(s) and/or timer(s). By doing
// so, you only enable the code within ifi_frc.c to become part of your
// software build.
// #define ENABLE_ADC			// enable if using ADC on a 18F8520
// #define ENABLE_INT_1			// enable if using encoder channel 1
// #define ENABLE_INT_2			// enable if using encoder channel 2
// #define ENABLE_INT_3			// enable if using encoder channel 3
// #define ENABLE_INT_4			// enable if using encoder channel 4
// #define ENABLE_INT_5			// enable if using encoder channel 5
// #define ENABLE_INT_6			// enable if using encoder channel 6
// #define ENABLE_TIMER_0		// unused
// #define ENABLE_TIMER_1		// unused
// #define ENABLE_TIMER_2		// unused
// #define ENABLE_TIMER_3	 	// leave disabled if using PWM code
// #define ENABLE_TIMER_4 		// enable if using the ADC or gyro


// DEBUG() is a macro that can be used just like printf() to send 
// debugging information to the screen. It has the added benefit that 
// it can be turned off and removed from the entire project by 
// commenting out the #define _DEBUG line below. Use it just like 
// printf(), just make sure you use double parentheses. As an example: 
// DEBUG(("Error: I'm about to cause a red-light-of-death\r\n")).
// The use of double parentheses is a common method employed to send a 
// variable number of arguments to a macro. Also, don't forget to
// include ifi_frc.h in every source file you use the DEBUG() macro.
#define _DEBUG

//
// If you modify stuff below this line, you'll break the software.
//

// use a different tmpdata section for ISRs if we're using C18 3.0 or later
#ifdef USE_C18_30
#define USE_ALTERNATE_TMPDATA
#endif

// DEBUG macro mentioned above
#ifdef _DEBUG
#define DEBUG(x) printf x
#else
#define DEBUG(x)
#endif

// #define ENABLE_INT_3_6 if interrupt 3, 4, 5 or 6 is enabled
#ifdef ENABLE_INT_3
#ifndef ENABLE_INT_3_6
#define ENABLE_INT_3_6
#endif
#endif
#ifdef ENABLE_INT_4
#ifndef ENABLE_INT_3_6
#define ENABLE_INT_3_6
#endif
#endif
#ifdef ENABLE_INT_5
#ifndef ENABLE_INT_3_6
#define ENABLE_INT_3_6
#endif
#endif
#ifdef ENABLE_INT_6
#ifndef ENABLE_INT_3_6
#define ENABLE_INT_3_6
#endif
#endif

#define	DATA_SIZE			30
#define	SPI_TRAILER_SIZE	2
#define	SPI_XFER_SIZE		DATA_SIZE + SPI_TRAILER_SIZE
#define	FALSE				0
#define	TRUE				!FALSE
#define INPUT				1
#define OUTPUT				0
#define	RESET_VECTOR		0x800
#define	HIGH_INT_VECTOR		0x808
#define	LOW_INT_VECTOR		0x818

//aliases for the analog inputs located on the operator interface
#define p1_y			rxdata.oi_analog01
#define p2_y			rxdata.oi_analog02
#define p3_y			rxdata.oi_analog03
#define p4_y			rxdata.oi_analog04
#define p1_x			rxdata.oi_analog05
#define p2_x			rxdata.oi_analog06
#define p3_x			rxdata.oi_analog07
#define p4_x			rxdata.oi_analog08
#define p1_wheel		rxdata.oi_analog09
#define p2_wheel		rxdata.oi_analog10
#define p3_wheel		rxdata.oi_analog11
#define p4_wheel		rxdata.oi_analog12
#define p1_aux			rxdata.oi_analog13
#define p2_aux			rxdata.oi_analog14
#define p3_aux			rxdata.oi_analog15
#define p4_aux			rxdata.oi_analog16

// aliases for the operator interface switches
#define p1_sw_trig		rxdata.oi_swA_byte.bitselect.bit0 // Joystick Trigger Button
#define p1_sw_top		rxdata.oi_swA_byte.bitselect.bit1 // Joystick Top Button
#define p1_sw_aux1		rxdata.oi_swA_byte.bitselect.bit2 // Aux input
#define p1_sw_aux2		rxdata.oi_swA_byte.bitselect.bit3 // Aux input
#define p3_sw_trig		rxdata.oi_swA_byte.bitselect.bit4 // Joystick Trigger Button
#define p3_sw_top		rxdata.oi_swA_byte.bitselect.bit5 // Joystick Top Button
#define p3_sw_aux1		rxdata.oi_swA_byte.bitselect.bit6 // Aux input
#define p3_sw_aux2		rxdata.oi_swA_byte.bitselect.bit7 // Aux input
#define p2_sw_trig		rxdata.oi_swB_byte.bitselect.bit0 // Joystick Trigger Button
#define p2_sw_top		rxdata.oi_swB_byte.bitselect.bit1 // Joystick Top Button
#define p2_sw_aux1		rxdata.oi_swB_byte.bitselect.bit2 // Aux input
#define p2_sw_aux2		rxdata.oi_swB_byte.bitselect.bit3 // Aux input
#define p4_sw_trig		rxdata.oi_swB_byte.bitselect.bit4 // Joystick Trigger Button
#define p4_sw_top		rxdata.oi_swB_byte.bitselect.bit5 // Joystick Top Button
#define p4_sw_aux1		rxdata.oi_swB_byte.bitselect.bit6 // Aux input
#define p4_sw_aux2		rxdata.oi_swB_byte.bitselect.bit7 // Aux input

// aliases for the robot controller digital I/O pins
#define digital_io_01	TRISBbits.TRISB2
#define digital_io_02	TRISBbits.TRISB3
#define digital_io_03	TRISBbits.TRISB4
#define digital_io_04	TRISBbits.TRISB5
#define digital_io_05	TRISBbits.TRISB6
#define digital_io_06	TRISBbits.TRISB7
#define digital_io_07	TRISHbits.TRISH0
#define digital_io_08	TRISHbits.TRISH1
#define digital_io_09	TRISHbits.TRISH2
#define digital_io_10	TRISHbits.TRISH3
#define digital_io_11	TRISJbits.TRISJ1
#define digital_io_12	TRISJbits.TRISJ2
#define digital_io_13	TRISJbits.TRISJ3
#define digital_io_14	TRISCbits.TRISC0
#define digital_io_15	TRISJbits.TRISJ4
#define digital_io_16	TRISJbits.TRISJ5
#define digital_io_17	TRISJbits.TRISJ6
#define digital_io_18	TRISJbits.TRISJ7

// aliases for the robot controller digital I/O pins when configured as inputs
#define rc_dig_in01		PORTBbits.RB2	// external interrupt RB2/INT2
#define rc_dig_in02		PORTBbits.RB3	// external interrupt RB3/INT3
#define rc_dig_in03		PORTBbits.RB4	// external interrupt-on-change RB4
#define rc_dig_in04		PORTBbits.RB5	// external interrupt-on-change RB5
#define rc_dig_in05		PORTBbits.RB6	// external interrupt-on-change RB6
#define rc_dig_in06		PORTBbits.RB7	// external interrupt-on-change RB7
#define rc_dig_in07		PORTHbits.RH0
#define rc_dig_in08		PORTHbits.RH1
#define rc_dig_in09		PORTHbits.RH2
#define rc_dig_in10		PORTHbits.RH3
#define rc_dig_in11		PORTJbits.RJ1
#define rc_dig_in12		PORTJbits.RJ2
#define rc_dig_in13		PORTJbits.RJ3
#define rc_dig_in14		PORTCbits.RC0
#define rc_dig_in15		PORTJbits.RJ4
#define rc_dig_in16		PORTJbits.RJ5
#define rc_dig_in17		PORTJbits.RJ6
#define rc_dig_in18		PORTJbits.RJ7

// aliases for the robot controller digital I/O pins when configured as outputs
#define rc_dig_out01	LATBbits.LATB2
#define rc_dig_out02	LATBbits.LATB3
#define rc_dig_out03	LATBbits.LATB4
#define rc_dig_out04	LATBbits.LATB5
#define rc_dig_out05	LATBbits.LATB6
#define rc_dig_out06	LATBbits.LATB7
#define rc_dig_out07	LATHbits.LATH0
#define rc_dig_out08	LATHbits.LATH1
#define rc_dig_out09	LATHbits.LATH2
#define rc_dig_out10	LATHbits.LATH3
#define rc_dig_out11	LATJbits.LATJ1
#define rc_dig_out12	LATJbits.LATJ2
#define rc_dig_out13	LATJbits.LATJ3
#define rc_dig_out14	LATCbits.LATC0
#define rc_dig_out15	LATJbits.LATJ4
#define rc_dig_out16	LATJbits.LATJ5
#define rc_dig_out17	LATJbits.LATJ6
#define rc_dig_out18	LATJbits.LATJ7

// aliases for the robot controller PWM outputs 
#define pwm01			txdata.rc_pwm01
#define pwm02			txdata.rc_pwm02
#define pwm03			txdata.rc_pwm03
#define pwm04			txdata.rc_pwm04
#define pwm05			txdata.rc_pwm05
#define pwm06			txdata.rc_pwm06
#define pwm07			txdata.rc_pwm07
#define pwm08			txdata.rc_pwm08
#define pwm09			txdata.rc_pwm09
#define pwm10			txdata.rc_pwm10
#define pwm11			txdata.rc_pwm11
#define pwm12			txdata.rc_pwm12
#define pwm13			txdata.rc_pwm13
#define pwm14			txdata.rc_pwm14
#define pwm15			txdata.rc_pwm15
#define pwm16			txdata.rc_pwm16

// aliases for the robot controller relay outputs
#define relay1_fwd		LATEbits.LATE0
#define relay1_rev		LATDbits.LATD0
#define relay2_fwd		LATEbits.LATE1
#define relay2_rev		LATDbits.LATD1
#define relay3_fwd		LATEbits.LATE2
#define relay3_rev		LATDbits.LATD2
#define relay4_fwd		LATEbits.LATE3
#define relay4_rev		LATDbits.LATD3
#define relay5_fwd		LATEbits.LATE4
#define relay5_rev		LATDbits.LATD4
#define relay6_fwd		LATEbits.LATE5
#define relay6_rev		LATDbits.LATD5
#define relay7_fwd		LATEbits.LATE6
#define relay7_rev		LATDbits.LATD6
#define relay8_fwd		LATJbits.LATJ0
#define relay8_rev		LATDbits.LATD7

// aliases for the analog input pins
#define rc_ana_in01		ADC_CH0
#define rc_ana_in02		ADC_CH1
#define rc_ana_in03		ADC_CH2
#define rc_ana_in04		ADC_CH3
#define rc_ana_in05		ADC_CH4
#define rc_ana_in06		ADC_CH5
#define rc_ana_in07		ADC_CH6
#define rc_ana_in08		ADC_CH7
#define rc_ana_in09		ADC_CH8
#define rc_ana_in10		ADC_CH9
#define rc_ana_in11		ADC_CH10
#define rc_ana_in12		ADC_CH11
#define rc_ana_in13		ADC_CH12
#define rc_ana_in14		ADC_CH13
#define rc_ana_in15		ADC_CH14
#define rc_ana_in16		ADC_CH15

// aliases for the operator interface "ROBOT FEEDBACK" LEDs
#define User_Mode_byte	txdata.LED_byte1.data
#define Pwm1_green		txdata.LED_byte1.bitselect.bit0
#define Pwm1_red		txdata.LED_byte1.bitselect.bit1 
#define Pwm2_green		txdata.LED_byte1.bitselect.bit2
#define Pwm2_red		txdata.LED_byte1.bitselect.bit3 
#define Relay1_red		txdata.LED_byte1.bitselect.bit4 
#define Relay1_green	txdata.LED_byte1.bitselect.bit5
#define Relay2_red		txdata.LED_byte1.bitselect.bit6 
#define Relay2_green	txdata.LED_byte1.bitselect.bit7
#define Switch1_LED		txdata.LED_byte2.bitselect.bit0
#define Switch2_LED		txdata.LED_byte2.bitselect.bit1
#define Switch3_LED		txdata.LED_byte2.bitselect.bit2

// aliases for the user bytes
#define User_Byte1		txdata.user_byte1.allbits
#define User_Byte2		txdata.user_byte2.allbits
#define User_Byte3		txdata.user_byte3	// breaker panel byte 3
#define User_Byte4		txdata.user_byte4	// breaker panel byte 4
#define User_Byte5		txdata.user_byte5	// breaker panel byte 5
#define User_Byte6		txdata.user_byte6	// breaker panel byte 6

// aliases for serial port two
#define usart2_TX		LATGbits.LATG1
#define usart2_RX		PORTGbits.RG2

// aliases for the battery voltage values
// formula to calculate voltage in volts: current_voltage = battery_voltage * 0.038 + 0.05; 
#define battery_voltage		rxdata.rc_main_batt*15.64/256
#define backup_voltage		rxdata.rc_backup_batt*15.64/256

// aliases for the different user modes
#define user_display_mode	rxdata.rc_mode_byte.mode.user_display
#define autonomous_mode		rxdata.rc_mode_byte.mode.autonomous
#define disabled_mode		rxdata.rc_mode_byte.mode.disabled

//
// structure definitions
//
// this structure contains important system status information
typedef struct
{
	unsigned int  :5;
	unsigned int  user_display:1;  // User display enabled = 1, disabled = 0
	unsigned int  autonomous:1;    // Autonomous enabled = 1, disabled = 0
	unsigned int  disabled:1;      // Robot disabled = 1, enabled = 0
} modebits;


// this structure allows you to address specific bits of a byte
typedef struct
{
	unsigned int  bit0:1;
	unsigned int  bit1:1;
	unsigned int  bit2:1;
	unsigned int  bit3:1;
	unsigned int  bit4:1;
	unsigned int  bit5:1;
	unsigned int  bit6:1;
	unsigned int  bit7:1;
} bitid;

//
// this structure defines the contents of the
// data received from the master processor
//
typedef struct {				// begin rx_data_record structure
  unsigned char packet_num;
  union
  { 
    bitid bitselect;
    modebits mode;				// rxdata.rc_mode_byte.mode.(user_display|autonomous|disabled)
    unsigned char allbits;		// rxdata.rc_mode_byte.allbits
  } rc_mode_byte;
  union
  {
    bitid bitselect;			// rxdata.oi_swA_byte.bitselect.bit0
    unsigned char allbits;		// rxdata.oi_swA_byte.allbits
  } oi_swA_byte;  
  union
  {
    bitid bitselect;			// rxdata.oi_swB_byte.bitselect.bit0
    unsigned char allbits;		// rxdata.oi_swB_byte.allbits
  } oi_swB_byte;  
  union
  {
    bitid bitselect;			// rxdata.rc_swA_byte.bitselect.bit0
    unsigned char allbits;		// rxdata.rc_swA_byte.allbits
  } rc_swA_byte;
  union
  {
    bitid bitselect;			// rxdata.rc_swB_byte.bitselect.bit0
    unsigned char allbits;		// rxdata.rc_swB_byte.allbits
  } rc_swB_byte;
  unsigned char oi_analog01;	// rxdata.oi_analog01
  unsigned char oi_analog02;
  unsigned char oi_analog03;
  unsigned char oi_analog04;  
  unsigned char oi_analog05;
  unsigned char oi_analog06;
  unsigned char oi_analog07;
  unsigned char oi_analog08;         
  unsigned char oi_analog09;
  unsigned char oi_analog10;
  unsigned char oi_analog11;
  unsigned char oi_analog12;
  unsigned char oi_analog13;
  unsigned char oi_analog14;
  unsigned char oi_analog15;
  unsigned char oi_analog16;
  unsigned char rc_main_batt;
  unsigned char rc_backup_batt;
  unsigned char reserve[8];
} rx_data_record;

typedef rx_data_record *rx_data_ptr;

//
// this structure defines the contents of the
// data transmitted to the master processor
//
typedef struct {				// begin tx_data_record structure
  union
  { 
    bitid bitselect;			// txdata.LED_byte1.bitselect.bit0
    unsigned char data;			// txdata.LED_byte1.data
  } LED_byte1;
  union
  { 
    bitid bitselect;			// txdata.LED_byte2.bitselect.bit0
    unsigned char data;			// txdata.LED_byte2.data
  } LED_byte2;
  union
  {
    bitid bitselect;			// txdata.user_byte1.bitselect.bit0
    unsigned char allbits;		// txdata.user_byte1.allbits
  } user_byte1;					// for OI feedback
  union
  {
    bitid bitselect;			// txdata.user_byte2.bitselect.bit0
    unsigned char allbits;		// txdata.user_byte2.allbits
  } user_byte2;					// for OI feedback
  unsigned char rc_pwm01;		// txdata.rc_pwm01
  unsigned char rc_pwm02;
  unsigned char rc_pwm03;
  unsigned char rc_pwm04;   
  unsigned char rc_pwm05;
  unsigned char rc_pwm06;
  unsigned char rc_pwm07;
  unsigned char rc_pwm08;
  unsigned char rc_pwm09;
  unsigned char rc_pwm10;
  unsigned char rc_pwm11;
  unsigned char rc_pwm12;
  unsigned char rc_pwm13;
  unsigned char rc_pwm14;
  unsigned char rc_pwm15;
  unsigned char rc_pwm16;
  unsigned char user_cmd;		// reserved
  unsigned char cmd_byte1;		// reserved
  unsigned char pwm_mask;		// <EDU> make sure you know how this works before changing
  unsigned char warning_code;	// reserved
  unsigned char user_byte3;		// <FRC> break panel byte 3
  unsigned char user_byte4;		// <FRC> break panel byte 4
  unsigned char user_byte5;		// <FRC> break panel byte 5
  unsigned char user_byte6;		// <FRC> break panel byte 6
  unsigned char error_code;		// reserved
  unsigned char packetnum;		// reserved
  unsigned char current_mode;	// reserved
  unsigned char control;		// reserved
} tx_data_record;

typedef tx_data_record *tx_data_ptr;


// this structure defines some flags which are used by the system

typedef struct
{
  unsigned int  NEW_SPI_DATA:1;
  unsigned int  TX_UPDATED:1;
  unsigned int  FIRST_TIME:1;
  unsigned int  TX_BUFFSELECT:1;
  unsigned int  RX_BUFFSELECT:1;
  unsigned int  SPI_SEMAPHORE:1;
  unsigned int  :2;
} packed_struct;


//
// variable declarations
//
extern near char FPFLAGS;		// needed for C18 2.4
extern tx_data_record txdata;	// located in ifi_frc_xxxx.lib 
extern rx_data_record rxdata;	// located in ifi_frc_xxxx.lib 
extern packed_struct statusflag;// located in ifi_frc_xxxx.lib

// function prototypes for code in ifi_frc_xxxx_yy.lib
void IFI_Initialization(void);
void User_Proc_Is_Ready(void);
void Putdata(tx_data_ptr ptr);
void Getdata(rx_data_ptr ptr);

// function prototypes for code in ifi_frc.c
void main(void);
void Interrupt_Vector_Low (void);
void Interrupt_Handler_Low (void);
void _entry(void);
void _startup(void);
void _do_cinit(void);
void Reset_Outputs(void);

#endif
