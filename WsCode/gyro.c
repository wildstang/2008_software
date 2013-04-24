/*******************************************************************************
*
* TITLE   gyro.c
*
* VERSION:  0.6 (Beta)
*
* DATE:   03-Jan-2008
*
* AUTHOR:   R. Kevin Watson
*           kevinw@jpl.nasa.gov
*
* COMMENTS: This file best viewed with tabs set to four.
*
*           You are free to use this source code for any non-commercial
*           use. Please do not make copies of this source code, modified
*           or un-modified, publicly available on the internet or
*           elsewhere without permission. Thanks.
*
*           Copyright �2005-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
* CHANGE LOG:
*
* DATE         REV  DESCRIPTION
* -----------  ---  ----------------------------------------------------------
* 21-Nov-2004  0.1  RKW - Original code.
* 12-Jan-2005  0.2  RKW - Altered Get_Gyro_Rate() and Get_Gyro_Angle() to use
*                   long integers for internal calculations, allowing larger
*                   numerators and denominators in the GYRO_RATE_SCALE_FACTOR
*                   and GYRO_ANGLE_SCALE_FACTOR #defines.
* 12-Jan-2005  0.2  RKW - GYRO_RATE_SCALE_FACTOR and GYRO_ANGLE_SCALE_FACTOR
*                   #defines added for Analog Devices' ADXRS401, ADXRS150 and
*                   ADXRS300 gyros.
* 16-Jan-2005  0.3  RKW - Using preprocessor directives, added the ability
*                   to select the gyro type, angular units, sample rate and
*                   number of averaged samples per update.
* 21-Jan-2005  0.3  RKW - Added scaling factors for the BEI GyroChip.
* 30-Jan-2005  0.4  RKW - Revised the way bias calculations are done.
*                   Instead of using only one data set as a bias, multiple
*                   sample sets can now be averaged over a much longer period
*                   of time to derive the gyro bias. Updated documentation.
* 04-Sep-2005  0.5  RKW - Significant overhaul of gyro code to strip-out ADC-
*                   specific code and use new adc.c/.h interface. Added
*                   deadband option.
* 21-Nov-2005  0.5  RKW - Added support for Murata's ENV-05D gyro.
* 10-Jan-2006  0.5  RKW - Verified code works on PIC18F8722.
* 03-Jan-2008  0.6  RKW - Modified Process_Gyro_Data() to wait for new ADC
*                   data so the user just needs to call Process_Gyro_Data()
*                   from any/all of the *_spin functions found in the new
*                   robot controller code.
*
*******************************************************************************/

#include "adc.h"
#include "gyro.h"
#include "ifi_frc.h"

int gyro_bias;
int gyro_rate;
long gyro_angle;

unsigned long avg_accum;
unsigned int avg_samples;

unsigned char calc_gyro_bias;

/*******************************************************************************
*
* FUNCTION:   Initialize_Gyro()
*
* PURPOSE:    Initializes the gyro code.
*
* CALLED FROM:  teleop.c/Initialization()
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   Place "#include "gyro.h" in the includes section
*         of teleop.c then call Initialize_Gyro() in
*         teleop.c/Initialization().
*
*         After calling Initialize_Gyro(), gyro rate and angle
*         data will be meaningless until a gyro bias calculation
*         has taken place by calling Start_Gyro_Bias_Calc() and
*         Stop_Gyro_Bias_Calc().
*
*******************************************************************************/
void Initialize_Gyro(void)
{
  // reset the heading angle to zero
  gyro_angle = 0;

  // reset the bias calculation flag
  calc_gyro_bias = 0;
}

/*******************************************************************************
*
* FUNCTION:   Get_Gyro_Rate()
*
* PURPOSE:    Returns the current heading angular rate of change.
*
* CALLED FROM:
*
* PARAMETERS:   None
*
* RETURNS:    Signed integer with the current heading rate of change.
*
* COMMENTS:
*
*******************************************************************************/
int Get_Gyro_Rate(void)
{
  // Return the calculated gyro rate to the caller.
  return((int)((((long)gyro_rate * GYRO_SENSITIVITY * 5L) / ADC_RANGE)) * GYRO_CAL_FACTOR);
}

/*******************************************************************************
*
* FUNCTION:   Get_Gyro_Angle()
*
* PURPOSE:    Returns the current heading angle.
*
* CALLED FROM:
*
* PARAMETERS:   None
*
* RETURNS:    Signed long with the current heading angle.
*
* COMMENTS:
*
*******************************************************************************/
long Get_Gyro_Angle(void)
{
  // Return the calculated gyro angle to the caller.
  return(((gyro_angle * GYRO_SENSITIVITY * 5L) / (ADC_RANGE * ADC_UPDATE_RATE)) * GYRO_CAL_FACTOR);
}

/*******************************************************************************
*
* FUNCTION:   Start_Gyro_Bias_Calc()
*
* PURPOSE:    Starts an ongoing gyro bias calculation.
*
* CALLED FROM:
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   IMPORTANT NOTE: Only call this function when your
*         robot will be absolutely still and free of vibration
*         (e.g., the air compressor is off) until the call to
*         Stop_Gyro_Bias_Calc() is made.
*
*******************************************************************************/
void Start_Gyro_Bias_Calc(void)
{
  if(calc_gyro_bias == 0)
  {
    // reset the averaging accumulator
    avg_accum = 0;

    // reset the number of number of average samples counters
    avg_samples = 0;

    // set flag informing the Process_Gyro_Data()
    // function to start a gyro bias calculation
    calc_gyro_bias = 1;
  }
}

/*******************************************************************************
*
* FUNCTION:   Stop_Gyro_Bias_Calc()
*
* PURPOSE:    Ends the current ongoing gyro bias calculation
*         and updates the internal gyro_bias variable.
*
* CALLED FROM:
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
void Stop_Gyro_Bias_Calc(void)
{
  if(calc_gyro_bias == 1)
  {
    // Update the gyro bias
    gyro_bias = (int)(avg_accum / avg_samples);

    // inform Process_Gyro_Data() function that
    // the ongoing bias calculation needs to stop
    calc_gyro_bias = 0;
  }
}

/*******************************************************************************
*
* FUNCTION:   Get_Gyro_Bias()
*
* PURPOSE:    Returns the current gyro bias.
*
* CALLED FROM:
*
* PARAMETERS:   None
*
* RETURNS:    Signed integer with the current gyro bias.
*
* COMMENTS:
*
*******************************************************************************/
int Get_Gyro_Bias(void)
{
  // return the gyro bias to the caller
  return(gyro_bias);
}

/*******************************************************************************
*
* FUNCTION:   Set_Gyro_Bias()
*
* PURPOSE:    Manually sets the gyro bias.
*
* CALLED FROM:
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
void Set_Gyro_Bias(int new_gyro_bias)
{
  // update gyro_bias
  gyro_bias = new_gyro_bias;
}

/*******************************************************************************
*
* FUNCTION:   Reset_Gyro_Angle()
*
* PURPOSE:    Resets the heading angle to zero.
*
* CALLED FROM:
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
void Reset_Gyro_Angle(void)
{
  // zero out gyro_angle
  gyro_angle = 0L;
}

/*******************************************************************************
*
* FUNCTION:   Process_Gyro_Data()
*
* PURPOSE:    Manages ADC data and does gyro rate integration
*
* CALLED FROM:  ifi_frc.c/Disabled_Spin(),
*         ifi_frc.c/Autonomous_Spin(),
*         ifi_frc.c/Teleop_Spin()
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:
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
      // convert the accumulator to an integer and update gyro_bias
      avg_accum += Get_ADC_Result(GYRO_CHANNEL);
      avg_samples++;
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
