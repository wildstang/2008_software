/*******************************************************************************
*
*	TITLE		gyro.c 
*
*	VERSION:	0.7 (Beta)                           
*
*	DATE:		28-Jan-2008
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This file best viewed with tabs set to four.
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or
*				elsewhere without permission. Thanks.
*
*				Copyright ©2005-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	21-Nov-2004  0.1  RKW - Original code.
*	12-Jan-2005  0.2  RKW - Altered Get_Gyro_Rate() and Get_Gyro_Angle() to use
*	                  long integers for internal calculations, allowing larger
*	                  numerators and denominators in the GYRO_RATE_SCALE_FACTOR
*	                  and GYRO_ANGLE_SCALE_FACTOR #defines.
*	12-Jan-2005  0.2  RKW - GYRO_RATE_SCALE_FACTOR and GYRO_ANGLE_SCALE_FACTOR
*	                  #defines added for Analog Devices' ADXRS401, ADXRS150 and
*	                  ADXRS300 gyros.
*	16-Jan-2005  0.3  RKW - Using preprocessor directives, added the ability 
*	                  to select the gyro type, angular units, sample rate and
*	                  number of averaged samples per update.
*	21-Jan-2005  0.3  RKW - Added scaling factors for the BEI GyroChip.
*	30-Jan-2005  0.4  RKW - Revised the way bias calculations are done.
*	                  Instead of using only one data set as a bias, multiple
*	                  sample sets can now be averaged over a much longer period
*	                  of time to derive the gyro bias. Updated documentation.
*	04-Sep-2005  0.5  RKW - Significant overhaul of gyro code to strip-out ADC-
*	                  specific code and use new adc.c/.h interface. Added
*	                  deadband option.
*	21-Nov-2005  0.5  RKW - Added support for Murata's ENV-05D gyro.
*	10-Jan-2006  0.5  RKW - Verified code works on PIC18F8722.
*	03-Jan-2008  0.6  RKW - Modified Process_Gyro_Data() to wait for new ADC
*	                  data so the user just needs to call Process_Gyro_Data()
*	                  from any/all of the *_spin functions found in the new
*	                  robot controller code.
*	28-Jan-2008  0.7  RKW - Fixed bug in Get_Gyro_Rate() that would return the
*	                  wrong value at high angular change rates.
*	                  RKW - Modified calculations within Get_Gyro_Angle() to
*	                  gain more rotation headroom before rollover occurs.
*	                  RKW - Modified gyro bias code to use a circular queue
*	                  to store samples until Stop_Gyro_Queue() is called,
*	                  which averages the queue to derive the bias.
*
*******************************************************************************/

#include "ifi_frc.h"
#include "adc.h"
#include "gyro.h"

int gyro_bias;
int gyro_rate;
long gyro_angle;
long rate_correction_numerator;
long rate_correction_denominator;
long angle_correction_numerator;
long angle_correction_denominator;

unsigned char calc_gyro_bias;
unsigned long accumulator;
unsigned int Gyro_Queue[GYRO_QUEUE_SIZE];
unsigned char Gyro_Bias_Status = GYRO_BIAS_NOT_DONE;
unsigned char Gyro_Queue_Index = 0;


/*******************************************************************************
*
*	FUNCTION:		Initialize_Gyro()
*
*	PURPOSE:		Initializes the gyro code.
*
*	CALLED FROM:	teleop.c/Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		Place "#include "gyro.h" in the includes section
*					of teleop.c then call Initialize_Gyro() in
*					teleop.c/Initialization().
*
*					After calling Initialize_Gyro(), gyro rate and angle
*					data will be meaningless until a gyro bias calculation
*					has taken place by calling Start_Gyro_Bias_Calc() and
*					Stop_Gyro_Bias_Calc().
*
*******************************************************************************/
void Initialize_Gyro(void)
{
	// reset the heading angle to zero
	gyro_angle = 0;

	// reset the bias calculation flag
	calc_gyro_bias = 0;

	// Calculate the gyro rate correction numerator and denominator
	rate_correction_numerator = GYRO_SENSITIVITY * 5L;
	rate_correction_denominator = ADC_RANGE;

	// Calculate the gyro angle correction numerator and denominator
	// These equations are optimized versions of the commented out equations below
	// correction_factor_numerator = (GYRO_SENSITIVITY * 5L);
	// correction_factor_denominator = (ADC_RANGE * ADC_UPDATE_RATE);
	angle_correction_numerator = GYRO_SENSITIVITY / 2L;
	angle_correction_denominator = (ADC_RANGE * ADC_UPDATE_RATE / 10L);
}

/*******************************************************************************
*
*	FUNCTION:		Get_Gyro_Rate()
*
*	PURPOSE:		Returns the current heading angular rate of change.
*
*	CALLED FROM:
*
*	PARAMETERS:		None
*
*	RETURNS:		Signed integer with the current heading rate of change.
*
*	COMMENTS:
*
*******************************************************************************/
int Get_Gyro_Rate(void)
{
	long temp_rate;

	// calculate the angular rate
	temp_rate = ((long)gyro_rate * rate_correction_numerator) / rate_correction_denominator;

	// apply the calibration factor
	temp_rate = temp_rate * GYRO_CAL_FACTOR;

	// return the calculated gyro rate to the caller.
	return((int)temp_rate);
}

/*******************************************************************************
*
*	FUNCTION:		Get_Gyro_Angle()
*
*	PURPOSE:		Returns the current heading angle.
*
*	CALLED FROM:
*
*	PARAMETERS:		None
*
*	RETURNS:		Signed long with the current heading angle.
*
*	COMMENTS:
*
*******************************************************************************/
long Get_Gyro_Angle(void)
{
	long temp_angle;

	// calculate angle
	temp_angle = (gyro_angle * angle_correction_numerator) / angle_correction_denominator;

	// apply the calibration factor
	temp_angle = temp_angle * GYRO_CAL_FACTOR;

	// return the calculated gyro angle to the caller.
	return(temp_angle);
}

/*******************************************************************************
*
*	FUNCTION:		Start_Gyro_Bias_Calc()
*
*	PURPOSE:		Starts an ongoing gyro bias calculation.
*
*	CALLED FROM:	disabled.c/Disabled_Init()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		IMPORTANT NOTE: Only call this function when your 
*					robot will be absolutely still and free of vibration 
*					(e.g., the air compressor is off) until the call to
*					Stop_Gyro_Bias_Calc() is made.
*
*******************************************************************************/
void Start_Gyro_Bias_Calc(void)
{
	if(calc_gyro_bias == 0)
	{
		// clear the accumulator for later use by Stop_Gyro_Bias_Calc()
		accumulator = 0L;

		// reset the circular queue index
		Gyro_Queue_Index = 0;

		// update the gyro bias status
		Gyro_Bias_Status = GYRO_BIAS_IN_PROCESS;

		// set flag informing the Process_Gyro_Data() 
		// function to start a gyro bias calculation
		calc_gyro_bias = 1;
	}
}

/*******************************************************************************
*
*	FUNCTION:		Stop_Gyro_Bias_Calc()
*
*	PURPOSE:		Ends the current ongoing gyro bias calculation
*					and updates the internal gyro_bias variable.
*
*	CALLED FROM:	autonomous.c/Autonomous_Init()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Stop_Gyro_Bias_Calc(void)
{
	unsigned char i;

	if(calc_gyro_bias == 1)
	{
		// add the contents of the circular queue together
		for(i=0; i<GYRO_QUEUE_SIZE; i++)
		{
			accumulator += Gyro_Queue[i];
		}

		// this is the same as dividing the accumulator by
		// the number of samples to derive an average value
		accumulator >>= GYRO_QUEUE_SIZE_EXPONENT;

		// convert the result to an integer and save
		gyro_bias = (int)accumulator;

		// update the gyro bias status
		Gyro_Bias_Status = GYRO_BIAS_READY;

		// inform Process_Gyro_Data() function that
		// the ongoing bias calculation needs to stop
		calc_gyro_bias = 0;
	}
}

/*******************************************************************************
*
*	FUNCTION:		Get_Gyro_Bias_Status()
*
*	PURPOSE:		Returns status of a gyro bias calculation.
*
*	CALLED FROM:
*
*	PARAMETERS:		None
*
*	RETURNS:		GYRO_BIAS_NOT_DONE if a gyro bias has not been calculated.
*
*					GYRO_BIAS_IN_PROCESS after Start_Gyro_Bias_Calc() has been
*					called and data is being collected
*
*					GYRO_BIAS_BUFFER_FULL if the circular buffer is full,
*					indicating that it is safe to call Stop_Gyro_Bias_Calc().
*
*					GYRO_BIAS_READY if a gyro bias has been calculated.
*
*	COMMENTS:
*
*******************************************************************************/
unsigned char Get_Gyro_Bias_Status(void)
{
	// return bias calculation status
	return(Gyro_Bias_Status);
}

/*******************************************************************************
*
*	FUNCTION:		Get_Gyro_Bias()
*
*	PURPOSE:		Returns the current gyro bias.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None
*
*	RETURNS:		Signed integer with the current gyro bias.
*
*	COMMENTS:
*
*******************************************************************************/
int Get_Gyro_Bias(void)
{
	// return the gyro bias to the caller
	return(gyro_bias);
}

/*******************************************************************************
*
*	FUNCTION:		Set_Gyro_Bias()
*
*	PURPOSE:		Manually sets the gyro bias.
*
*	CALLED FROM:
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Set_Gyro_Bias(int new_gyro_bias)
{
	// update gyro_bias
	gyro_bias = new_gyro_bias;
}

/*******************************************************************************
*
*	FUNCTION:		Reset_Gyro_Angle()
*
*	PURPOSE:		Resets the heading angle to zero.
*
*	CALLED FROM:
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Reset_Gyro_Angle(void)
{
	// zero out gyro_angle
	gyro_angle = 0L;
}

/*******************************************************************************
*
*	FUNCTION:		Process_Gyro_Data()
*
*	PURPOSE:		Manages ADC data and does gyro rate integration
*
*	CALLED FROM:	ifi_frc.c/Disabled_Spin(),
*					ifi_frc.c/Autonomous_Spin(),
*					ifi_frc.c/Teleop_Spin()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Process_Gyro_Data(void)
{
	int temp_gyro_rate;

	// fresh ADC data available?
	if(Get_ADC_Result_Count())
	{	
		// should the completed sample set be used to calculate the gyro bias?
		if(calc_gyro_bias == 1)
		{
			// put the ADC reading on the circular queue
			Gyro_Queue[Gyro_Queue_Index] = Get_ADC_Result(GYRO_CHANNEL);
		
			// increment the write pointer
			Gyro_Queue_Index++;
		
			// is the circular queue now full?
			if(Gyro_Queue_Index == GYRO_QUEUE_SIZE-1)
			{ 
				// update the gyro bias status
				Gyro_Bias_Status = GYRO_BIAS_BUFFER_FULL;
			}

			// If the index pointer overflowed, cut-off the high-order bit. Doing this
			// every time is quicker than checking for overflow every time with an if()
			// statement and only then occasionally setting it back to zero. For this 
			// to work, the queue size must be a power of 2 (e.g., 16,32,64,128).
			Gyro_Queue_Index &= GYRO_QUEUE_INDEX_MASK;
		}
		else
		{
			// get the latest measured gyro rate
			temp_gyro_rate = (int)Get_ADC_Result(GYRO_CHANNEL) - gyro_bias;
	
			// update reported gyro rate and angle only if 
			// measured gyro rate lies outside the deadband
			if(temp_gyro_rate < -GYRO_DEADBAND || temp_gyro_rate > GYRO_DEADBAND)
			{
				// update the gyro rate
				gyro_rate = temp_gyro_rate;
	
				// integrate the gyro rate to derive the heading 
				gyro_angle += (long)temp_gyro_rate;
			}
			else
			{
				gyro_rate = 0;
			}
		}

		Reset_ADC_Result_Count();
	}		
}
