/*******************************************************************************
*
*	TITLE:		teleop.c 
*
*	VERSION:	0.1 (Beta)                           
*
*	DATE:		31-Dec-2007
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This file best viewed with tabs set to four.
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
*	31-Dec-2007  0.1  RKW Original
*
*******************************************************************************/

#include "ifi_frc.h"
#include "adc.h"
#include "gyro.h"
#include "pwm.h"
#include "timers.h"
#include "interrupts.h"
#include "encoder.h"
#include "serial_ports.h"
#include "ifi_code.h"
#include "teleop.h"

/*******************************************************************************
*
*	FUNCTION:		Initialization()
*
*	PURPOSE:		This function is called once when the robot controller
*					is cold or warm booted. You should initialize your code
*					here.		
* 
*	CALLED FROM:	main() in ifi_frc.c
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
* 
*	COMMENTS:
*
*******************************************************************************/
void Initialization(void)
{
	// Setup the digital I/O pins. Use "INPUT" to setup the pin
	// as an input and "OUTPUT" to setup the pin as an output.
	digital_io_01 = INPUT;
	digital_io_02 = INPUT;
	digital_io_03 = INPUT;
	digital_io_04 = INPUT;
	digital_io_05 = INPUT;
	digital_io_06 = INPUT;
	digital_io_07 = INPUT;
	digital_io_08 = INPUT;
	digital_io_09 = INPUT;
	digital_io_10 = INPUT;
	digital_io_11 = INPUT;
	digital_io_12 = INPUT;
	digital_io_13 = INPUT;
	digital_io_14 = INPUT;
	digital_io_15 = INPUT;
	digital_io_16 = INPUT;
	digital_io_17 = INPUT;
	digital_io_18 = INPUT;


	// Initialize the digital outputs. If the pin is configured
	// as an input above, it doesn't matter what state you
	// initialize it to here.
	rc_dig_out01 = 0;
	rc_dig_out02 = 0;
	rc_dig_out03 = 0;
	rc_dig_out04 = 0;
	rc_dig_out05 = 0;
	rc_dig_out06 = 0;
	rc_dig_out07 = 0;
	rc_dig_out08 = 0;
	rc_dig_out09 = 0;
	rc_dig_out10 = 0;
	rc_dig_out11 = 0;
	rc_dig_out12 = 0;
	rc_dig_out13 = 0;
	rc_dig_out14 = 0;
	rc_dig_out15 = 0;
	rc_dig_out16 = 0;
	rc_dig_out17 = 0;
	rc_dig_out18 = 0;

	// Initialize timers and interrupts here


	// Remove the // below to initialize the analog to digital converter (ADC) code
	// Initialize_ADC();

	// Remove the // below to initialize the gyro code (ADC code must be enabled too)
	// Initialize_Gyro();

	// Remove the // below to initialize encoder #1
	// Initialize_Encoder_1();

	// Remove the // below to initialize encoder #2
	// Initialize_Encoder_2();

	// Remove the // below to initialize encoders #3 through #6
	// Initialize_Encoder_3_6();

	printf("IFI User Processor Initialized ...\r\n");
}

/*******************************************************************************
*
*	FUNCTION:		Teleop_Init()
*
*	PURPOSE:		This is where you put code that needs to execute
*					just once at the start of teleoperation mode.		
* 
*	CALLED FROM:	main() in ifi_frc.c
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
* 
*	COMMENTS:		While in this mode, all operator interface data is valid
*					and all robot controller outputs are enabled.
*
*******************************************************************************/
void Teleop_Init(void)
{

}

/*******************************************************************************
*
*	FUNCTION:		Teleop()
*
*	PURPOSE:		This is where you put code that you want to execute while
*					your robot is in teleoperation mode. While in teleoperation
*					mode, this function is called every	26.2ms after new data
*					is received from the master processor.		
* 
*	CALLED FROM:	main() in ifi_frc.c
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
* 
*	COMMENTS:		While in this mode, all operator interface data is valid
*					and all robot controller outputs are enabled.
*
*******************************************************************************/
void Teleop(void)
{
	// example code to test your gyro and encoder(s)

	static unsigned int i = 0;
	static unsigned int j = 0;
	int temp_gyro_rate;
	long temp_gyro_angle;
	int temp_gyro_bias;
	long temp_gyro_angle_fast;
	long Encoder_Count;

	i++;
	j++; // this will rollover every ~1000 seconds

/*
	// enable this block of code to test your gyro
	if(j == 10)
	{
		printf("\r\nCalculating Gyro Bias...\r\n");
	}
	if(j == 38) // let the gyro stablize for a second before starting a calibration
	{
		// start a gyro bias calculation
		Start_Gyro_Bias_Calc();
	}
	if(j == 191) // allow calibration routine to run for four seconds 
	{
		// terminate the gyro bias calculation
		Stop_Gyro_Bias_Calc();

		// reset the gyro heading angle
		Reset_Gyro_Angle();

		temp_gyro_bias = Get_Gyro_Bias();
		printf("Gyro Bias=%d\r\n", temp_gyro_bias);
	}
	if(i == 35 && j >= 191)
	{
		temp_gyro_rate = Get_Gyro_Rate();
		temp_gyro_angle = Get_Gyro_Angle();
		printf("Gyro Rate=%d\r\n", temp_gyro_rate);
		printf("Gyro Angle=%d\r\n\r\n", (int)temp_gyro_angle);
	}
*/

/*
	// enable this block of code to test your encoder(s)
	if(i == 37 && j >= 191)
	{
		Encoder_Count = Get_Encoder_1_Count();
		printf("E1=%d\r\n",(int)Encoder_Count);
	
		Encoder_Count = Get_Encoder_2_Count();
		printf("E2=%d\r\n",(int)Encoder_Count);
	
		Encoder_Count = Get_Encoder_3_Count();
		printf("E3=%d\r\n",(int)Encoder_Count);
	
		Encoder_Count = Get_Encoder_4_Count();
		printf("E4=%d\r\n",(int)Encoder_Count);
	
		Encoder_Count = Get_Encoder_5_Count();
		printf("E5=%d\r\n",(int)Encoder_Count);
	
		Encoder_Count = Get_Encoder_6_Count();
		printf("E6=%d\r\n\r\n",(int)Encoder_Count);
	}
*/

	// reset the loop counter
	if(i >= 38)
	{
		i = 0;
	}

	// enable this to use IFI's default robot code
	// Default_Routine();	// located in ifi_code.c

	// update the state of the LEDs on the operator interface
	Update_OI_LEDs();		// located in ifi_code.c
}

/*******************************************************************************
*
*	FUNCTION:		Teleop_Spin()
*
*	PURPOSE:		While in teleoperation mode, this function is called
*					continuously between calls to Teleop().		
* 
*	CALLED FROM:	main() in ifi_frc.c
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
* 
*	COMMENTS:		While in this mode, all operator interface data is valid
*					and all robot controller outputs are enabled.
*
*******************************************************************************/
void Teleop_Spin(void)
{
	// enable this function if you want to process gyro data during teleop mode
	// Process_Gyro_Data();
}
