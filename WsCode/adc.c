/*******************************************************************************
 *
 *  TITLE   adc.c
 *
 *  VERSION:  0.6 (Beta)
 *
 *  DATE:     13-Jan-2008
 *
 *  AUTHOR:   R. Kevin Watson
 *            kevinw@jpl.nasa.gov
 *
 *  COMMENTS: This version will only work with PIC18F8722 based robot
 *            controllers. You should use version 0.3 of this software
 *            with a PIC18F8520 based robot controller.
 *
 *            This file best viewed with tabs set to four.
 *
 *            You are free to use this source code for any non-commercial
 *            use. Please do not make copies of this source code, modified
 *            or un-modified, publicly available on the internet or
 *            elsewhere without permission. Thanks.
 *
 *            Copyright ©2005-2008 R. Kevin Watson. All rights are reserved.
 *
 ********************************************************************************
 *
 *  CHANGE LOG:
 *
 *  DATE         REV  DESCRIPTION
 *  -----------  ---  ----------------------------------------------------------
 *  10-Jul-2005  0.1  RKW - Original code.
 *  17-Jul-2005  0.2  RKW - Added x128 and x256 oversampling options.
 *  13-Dec-2005  0.3  RKW - Altered code to use the "special event trigger"
 *                    functionality of the CCP2 hardware to initiate ADC
 *                    conversions. This was formally done using timer 2.
 *  10-Jan-2006  0.4  RKW - Ported to PIC18F8722, which required going back
 *                    to using timer 2 to initiate an analog to digital
 *                    conversion due to a bug in the PIC18F8722 design.
 *                    Modified #pragma interruptlow statement to include
 *                    .tmpdata section.
 *  03-Jan-2008  0.5  RKW - Modified code to use the PIC18F8722's built in
 *                    acquisition delay functionality to initiate ADC
 *                    conversions automatically. This means only one
 *                    interrupt is generated for each conversion instead
 *                    of two. Unfortunately, this breaks compatibility
 *                    with the PIC18F8520 based robot controllers.
 *  03-Jan-2008  0.5  RKW - Modified code to use timer 4 instead of 2.
 *  03-Jan-2008  0.6  RKW - Modified ISR code to use the new C18 3.0+
 *                    interrupt scheme used in the new robot controller
 *                    code.
 *  13-Jan-2008  0.6  RKW - ISR modification to improve efficiency. Made
 *                    modifications to documentation to be consistant with
 *                    new robot controller code.
 *
 *******************************************************************************/

#include "adc.h"
#include "ifi_frc.h"


unsigned int adc_sample_rate;
unsigned int adc_samples_per_update;
unsigned char num_adc_channels;
unsigned char adc_result_divisor;

volatile unsigned long accum[NUM_ADC_CHANNELS]; // sample accumulator
volatile unsigned int adc_result[NUM_ADC_CHANNELS]; // ADC recults
volatile unsigned int samples; // current number of samples accumulated
volatile unsigned char channel; // current ADC channel
volatile unsigned char adc_update_count = 0; // ADC update flag


/*******************************************************************************
 *
 *  FUNCTION:   Initialize_ADC()
 *
 *  PURPOSE:    Initializes the Analog to Digital Conversion (ADC) hardware.
 *
 *  CALLED FROM:  teleop.c/Initialization()
 *
 *  PARAMETERS:   None
 *
 *  RETURNS:    Nothing
 *
 *  COMMENTS:   Place '#include "adc.h"' in the includes section
 *          of teleop.c then call Initialize_ADC() in
 *          teleop.c/Initialization().
 *
 *******************************************************************************/
void Initialize_ADC(void)
{
  unsigned char i;

  // Initialize ADC related global variables using default values from adc.h.
  // Another way to do this would be to pass the values in, read them from
  // values stored in EEPROM or perhaps set them via a menu-based terminal
  // interface.
  adc_sample_rate = ADC_SAMPLE_RATE;
  num_adc_channels = NUM_ADC_CHANNELS;
  adc_result_divisor = ADC_RESULT_DIVISOR;
  adc_samples_per_update = ADC_SAMPLES_PER_UPDATE;

  // reset the sample accumulator(s) to zero
  for(i=0; i < num_adc_channels; i++)
  {
    accum[i] = 0L;
  }

  // start a new sample set
  samples = 0;

  // start at ADC channel zero
  channel = 0;

  // enable the ADC hardware and select ADC channel 0
  ADCON0 = 0b00000001;

  // enable all sixteen analog inputs
  ADCON1 = 0;

  // select Fosc/64 as the ADC Conversion Clock, right justify data
  // in the conversion result register and select 20 Tad acquisition
  // period
  ADCON2 = 0b10111110;

  // make sure the ADC interrupt is disabled
  PIE1bits.ADIE = 0;

  // initialize and start timer 4, which is responsible for starting
  // analog to digital conversions
  switch(adc_sample_rate)
  {
    case 200:
      // use these parameters for a 200Hz ADC sample rate

      // use a 1:16 prescaler and 1:14 postscaler
      T4CON = 0b01101010;

      // Count to 221 before rolling over and generating
      // an interrupt (223.21 - 2 is ideal)
      PR4 = 221;
      break;

    case 400:
      // use these parameters for a 400Hz ADC sample rate

      // use a 1:16 prescaler and 1:11 postscaler
      T4CON = 0b01010010;

      // Count to 140 before rolling over and generating
      // an interrupt (142.05 - 2 is ideal)
      PR4 = 140;
      break;

    case 800:
      // use these parameters for a 800Hz ADC sample rate

      // use a 1:16 prescaler and 1:11 postscaler
      T4CON = 0b01010010;

      // Count to 69 before rolling over and generating
      // an interrupt (71.02 - 2 is ideal)
      PR4 = 69;
      break;

    case 1600:
      // use these parameters for a 1600Hz ADC sample rate

      // use a 1:4 prescaler and 1:11 postscaler
      T4CON = 0b01010001;

      // Count to 140 before rolling over and generating
      // an interrupt (142.05 - 2 is ideal)
      PR4 = 140;
      break;

    case 3200:
      // use these parameters for a 3200Hz ADC sample rate

      // use a 1:1 prescaler and 1:11 postscaler
      T4CON = 0b01010001;

      // Count to 69 before rolling over and generating
      // an interrupt (71.02 - 2 is ideal)
      PR4 = 69;
      break;

    case 6400:
      // use these parameters for a 6400Hz ADC sample rate

      // use a 1:4 prescaler and 1:11 postscaler
      T4CON = 0b01010000;

      // Count to 140 before rolling over and generating
      // an interrupt (142.05 - 2 is ideal)
      PR4 = 140;
      break;

    default:
      // if a non-supported rate is specified,
      // default to a 200Hz ADC sample rate

      // use a 1:16 prescaler and 1:14 postscaler
      T4CON = 0b01101010;

      // Count to 221 before rolling over and generating
      // an interrupt (223.21 - 2 is ideal)
      PR4 = 221;
      break;
  }

  // make sure the timer 4 register starts at zero
  TMR4 = 0x00;

  // timer 4 interrupt is low priority
  IPR3bits.TMR4IP = 0;

  // to prevent a spurious interrupt, make sure the interrupt flag is reset
  PIR3bits.TMR4IF = 0;

  // enable the timer 4 interrupt
  PIE3bits.TMR4IE = 1;

  // enable timer 4
  T4CONbits.TMR4ON = 1;
}

/*******************************************************************************
 *
 *  FUNCTION:   Disable_ADC()
 *
 *  PURPOSE:    Disables the Analog to Digital Conversion (ADC) hardware.
 *
 *  CALLED FROM:
 *
 *  PARAMETERS:   None
 *
 *  RETURNS:    Nothing
 *
 *  COMMENTS:
 *
 *******************************************************************************/
void Disable_ADC(void)
{
  unsigned char i;

  // disable the timer 4 interrupt
  PIE3bits.TMR4IE = 0;

  // disable timer 4
  T4CONbits.TMR4ON = 0;

  // disable the ADC hardware and select ADC channel 0
  ADCON0 = 0b00000000;
}

/*******************************************************************************
 *
 *  FUNCTION:   Get_ADC_Result()
 *
 *  PURPOSE:    Given the ADC channel number, returns the last ADC result
 *          expressed in "data number" units.
 *
 *  CALLED FROM:
 *
 *  PARAMETERS:   ADC channel number
 *
 *  RETURNS:    Nothing
 *
 *  COMMENTS:
 *
 *******************************************************************************/
unsigned int Get_ADC_Result(unsigned char channel)
{
  unsigned int temp_adc_result;

  if(channel <= num_adc_channels)
  {
    // disable the timer interrupt
    PIE3bits.TMR4IE = 0;

    // one is subtracted because analog input 1 maps to adc_result[0],
    // input 2 maps to adc_result[1],..., input 16 maps to adc_result[15]
    temp_adc_result = adc_result[channel - 1];

    // enable the timer interrupt
    PIE3bits.TMR4IE = 1;
  }
  else
  {
    // bad channel number; return zero
    temp_adc_result = 0;
  }

  return(temp_adc_result);
}

/*******************************************************************************
 *
 *  FUNCTION:   Convert_ADC_to_mV()
 *
 *  PURPOSE:    Converts the raw output of the ADC to millivolts.
 *
 *  CALLED FROM:
 *
 *  PARAMETERS:   ADC output value to convert
 *
 *  RETURNS:    Millivolts
 *
 *  COMMENTS:
 *
 *******************************************************************************/
unsigned int Convert_ADC_to_mV(unsigned int adc)
{
  return((unsigned int)(((long)adc * (VREF_POS_MV - VREF_NEG_MV)) / ADC_RANGE));
}

/*******************************************************************************
 *
 *  FUNCTION:   Get_ADC_Result_Count()
 *
 *  PURPOSE:
 *
 *  CALLED FROM:
 *
 *  PARAMETERS:   None
 *
 *  RETURNS:    Nothing
 *
 *  COMMENTS:
 *
 *******************************************************************************/
unsigned char Get_ADC_Result_Count()
{
  unsigned char temp_adc_update_count;

  // disable the timer interrupt
  PIE3bits.TMR4IE = 0;

  temp_adc_update_count = adc_update_count;

  // enable the timer interrupt
  PIE3bits.TMR4IE = 1;

  return(temp_adc_update_count);
}

/*******************************************************************************
 *
 *  FUNCTION:   Reset_ADC_Result_Count()
 *
 *  PURPOSE:    Resets the ADC update counter to zero
 *
 *  CALLED FROM:
 *
 *  PARAMETERS:   None
 *
 *  RETURNS:    Nothing
 *
 *  COMMENTS:
 *
 *******************************************************************************/
void Reset_ADC_Result_Count()
{
  // disable the timer interrupt
  PIE3bits.TMR4IE = 0;

  adc_update_count = 0;

  // enable the timer interrupt
  PIE3bits.TMR4IE = 1;
}

/*******************************************************************************
 *
 *  FUNCTION:   Timer_4_ISR()
 *
 *  PURPOSE:    Timer 4 interrupt service routine
 *
 *  CALLED FROM:  ifi_frc.c/Interrupt_Handler_Low()
 *
 *  PARAMETERS:   None
 *
 *  RETURNS:    Nothing
 *
 *  COMMENTS:
 *
 *******************************************************************************/
#pragma tmpdata low_isr_tmpdata
void Timer_4_ISR(void)
{
  unsigned int adc;
  unsigned char adcon0_temp;
  int i;

  // get conversion results
  adc = ADRESH;
  adc <<= 8;
  adc += ADRESL;

  // add the ADC data to the appropriate accumulator
  accum[channel] += (long)adc;

  // increment the ADC channel index
  channel++;

  // do we need to wrap around to zero?
  if(channel >= num_adc_channels)
  {
    channel = 0;
    samples++;
  }

  // Select next ADC channel. This also starts the process whereby the ADC
  // sample and hold capacitor is allowed to start charging, which must be
  // completed before the next analog to digital conversion can be started.
  adcon0_temp = channel;
  adcon0_temp <<= 2;
  adcon0_temp |= 0b00000001;
  ADCON0 = adcon0_temp;

  // start a new analog to digital conversion
  ADCON0bits.GO = 1;

  // check to see if we've got a full sample set
  if(samples >= adc_samples_per_update)
  {
    // update the ADC result array and reset the sample accumulator(s) to zero
    for(i=0; i < num_adc_channels; i++)
    {
      adc_result[i] = (long)(accum[i] >> adc_result_divisor);
      accum[i] = 0L;
    }

    // signal that a fresh sample set is available
    adc_update_count++;

    // start a fresh sample set
    samples = 0;
  }
}
#pragma tmpdata
