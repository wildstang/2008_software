/*******************************************************************************
*
*	TITLE:		autonomous.c 
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
#include "pwm.h"
#include "gyro.h"
#include "timers.h"
#include "interrupts.h"
#include "encoder.h"
#include "serial_ports.h"
#include "ifi_code.h"
#include "autonomous.h"

/*******************************************************************************
*
*	FUNCTION:		Autonomous_Init()
*
*	PURPOSE:		This is where you put code that you want to execute
*					just once at the start of autonomous mode.
* 
*	CALLED FROM:	main() in ifi_frc.c
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
* 
*	COMMENTS:		While in this mode, all operator interface data is invalid
*					(i.e., all analog inputs are set to 127 and digital inputs
*					are set to zero) and all robot controller outputs (PWMs and
*					relays) are	enabled.
*
*******************************************************************************/
void Autonomous_Init(void)
{
	// this is where the the gyro bias calculation should end
	// Stop_Gyro_Bias_Calc()
}

/*******************************************************************************
*
*	FUNCTION:		Autonomous()
*
*	PURPOSE:		This is where you put code that you want to execute while
*					your robot is in autonomous mode. While in autonomous mode,
*					this function is called every 26.2ms after new data is 
*					received from the master processor.				
* 
*	CALLED FROM:	main() in ifi_frc.c
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
* 
*	COMMENTS:		While in this mode, all operator interface data is invalid
*					(i.e., all analog inputs are set to 127 and digital inputs
*					are set to zero) and all robot controller outputs (PWMs and
*					relays) are	enabled.
*
*******************************************************************************/
void Autonomous(void)
{

}

/*******************************************************************************
*
*	FUNCTION:		Autonomous_Spin()
*
*	PURPOSE:		While in autonomous mode, this function is called
*					continuously between calls to Autonomous().			
* 
*	CALLED FROM:	main() in ifi_frc.c
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
* 
*	COMMENTS:		While in this mode, all operator interface data is invalid
*					(i.e., all analog inputs are set to 127 and digital inputs
*					are set to zero) and all robot controller outputs (PWMs and
*					relays) are	enabled.
*
*******************************************************************************/
void Autonomous_Spin(void)
{
	// enable this function if you want to process gyro data during autonomous mode
	// Process_Gyro_Data();
}