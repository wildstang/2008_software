/*******************************************************************************
* FILE NAME: ws_general.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_frc.h"
#include "adc.h"
#include "ws_includes.h"
#include "ws_io.h"
#include "ws_general.h"

/*******************************************************************************
* FUNCTION NAME: pump_control()
* PURPOSE:       Turns pump on and off based on pressure sensor
* ARGUMENTS:     none
* RETURNS:       none
*
*******************************************************************************/
void pump_control(void)
{
  /* Check to see if pressure switch is off
     This means that we are less than 120 PSI.
     In this case we want the pump to turn on.

     If the pressure switch indicates that we are
     greater than 120PSI, we want to run for PUMP_TOP_OFF
     cycles before it turns off

     Input variables:
     -------------------
     Dig_in_pressure - The variable containing the state of the pressure switch
                       0 = Less than or equal to 120PSI
                       1 = Greater than 120PSI

     competition_mode - The variable containing the state of the robot.  This
                        will have one of the following values.
                        ROBOT_ENABLED
                        ROBOT_DISABLED

     Output variables:
     -------------------
     motor_vals.pump - The output variable that will control
                       the pump.  It can be set to the following
                       values - PUMP_ON, PUMP_OFF
  */

  static int counter = PUMP_LOOP_COUNT;

  if (Dig_in_pressure == PRESSURE_BELOW_120)
  {
    counter = 0;
  }
  else if (motor_vals.pump == PUMP_ON)
  {
    counter++;
  }

  if (counter < PUMP_LOOP_COUNT)
  {
    motor_vals.pump = PUMP_ON;
  }
  else
  {
    motor_vals.pump = PUMP_OFF;

    /* Protects counter from wrapping around */
    counter = PUMP_LOOP_COUNT;
  }

  /*
  printf("S: %d  C %d  MV.PUMP %d\r", Dig_in_pressure, counter, motor_vals.pump);
  */
}

/*******************************************************************************
* FUNCTION NAME: process_adc
* PURPOSE:       get current adc values and store in globals
*******************************************************************************/
UINT8 process_adc(void)
{
#if USE_DIGITAL_POT
  static INT8 startup_loops = 0;
  static INT16 front_raw_val_prev;
  static INT16 back_raw_val_prev;
  static INT16 last_front_diff = 0;
  static INT16 last_back_diff = 0;
  static UINT8 is_init = FALSE;
  INT16 front_raw_val, back_raw_val;
  INT16 diff;
#endif
  INT8 ret = FALSE;

  if (Get_ADC_Result_Count())
  {
#if USE_DIGITAL_POT
    if (startup_loops > ADC_WAIT)
    {
      /* Store the values from the ADC read locally*/
      front_raw_val = Get_ADC_Result(Analog_in_front_crab);
      back_raw_val = Get_ADC_Result(Analog_in_back_crab);

      if (is_init == FALSE)
      {
        front_raw_val_prev = front_raw_val;
        back_raw_val_prev = back_raw_val;
        is_init = TRUE;
      }

      ROLLOVER(back_raw_val, back_raw_val_prev, last_back_diff, diff);
      ROLLOVER(front_raw_val, front_raw_val_prev, last_front_diff, diff);

      /* Calculate the spiral_count and absolute pot value for the crabs */
#if DEBUG_POT_ROLLOVER
      printf("F ");
#endif
      process_crab_pot_data(&pot_vals.front_crab, front_raw_val,
                            calibration_vals.front_crab.pot_mid);
#if DEBUG_POT_ROLLOVER
      printf("B ");
#endif
      process_crab_pot_data(&pot_vals.back_crab, back_raw_val,
                            calibration_vals.back_crab.pot_mid);

      Reset_ADC_Result_Count();
      ret = TRUE;

#if ADC_RESULT_PRINTS
      printf("fr %4d fs %d br %4d bs %d ", pot_vals.front_crab.raw_val,
             pot_vals.front_crab.spiral_count, pot_vals.back_crab.raw_val,
             pot_vals.back_crab.spiral_count);
#endif
    }
    else
    {
      Reset_ADC_Result_Count();
      startup_loops++;
    }
#else
    pot_vals.front_crab.abs_val = Get_ADC_Result(Analog_in_front_crab);
    pot_vals.back_crab.abs_val = Get_ADC_Result(Analog_in_back_crab);
    Reset_ADC_Result_Count();
    ret = TRUE;

#if ADC_RESULT_PRINTS
    printf("fa %4d ba %4d ", pot_vals.front_crab.abs_val,
           pot_vals.back_crab.abs_val);
#endif

#endif
  }
  return ret;
}

/*******************************************************************************
* FUNCTION NAME: calc_pot_rollover
* PURPOSE:       determine when the crab wheels rollover the spiral
*******************************************************************************/
#if USE_DIGITAL_POT
INT8 calc_pot_rollover(UINT16 val, UINT16 val_prev)
{
  INT8 ret = 0;

  if((val <= (CRAB_POT_RES / 2)) &&
      (val > ((CRAB_POT_RES / 2) - (CRAB_POT_RES / 8))) &&
      (val_prev > (CRAB_POT_RES / 2)) &&
      (val_prev < ((CRAB_POT_RES / 2) + (CRAB_POT_RES / 8))))
  {
#if DEBUG_POT_ROLLOVER
    printf(" RD ");
#endif
    ret = -1;
  }
  else if((val > (CRAB_POT_RES / 2)) &&
      (val < ((CRAB_POT_RES / 2) + (CRAB_POT_RES / 8))) &&
      (val_prev <= (CRAB_POT_RES / 2)) &&
      (val_prev > ((CRAB_POT_RES / 2) - (CRAB_POT_RES / 8))))
  {
#if DEBUG_POT_ROLLOVER
    printf(" RU ");
#endif
    ret = 1;
  }
  else
  {
#if DEBUG_POT_ROLLOVER
    printf(" RN ");
#endif
  }

  return ret;
}
#endif


/*******************************************************************************
* FUNCTION NAME: process_crab_pot_data
* PURPOSE:       determine when the crab wheels rollover the spiral
*******************************************************************************/
#if USE_DIGITAL_POT
void process_crab_pot_data(PotDataType *pot_data, INT16 raw_val,
                           UINT16 calib_mid)
{
  INT16 rot_raw_val, rot_raw_val_prev;
  INT16 back_of_pot;

  if (pot_data->is_init == 1)
  {
    /* Copy the current values to the previous values.  This is here not at the
       end because we need to use it in calculations this loop */
    pot_data->raw_val_prev = pot_data->raw_val;
  }
  else
  {
    /* Initialize things the first time through */
    pot_data->spiral_count = SPIRAL_POS_INIT;
    pot_data->raw_val_prev = raw_val;
    pot_data->is_init = 1;
  }

  pot_data->raw_val = raw_val;

  /* Get the previous raw value of the pot rotated so that 0 is forward
     Also, if the value is negative, we need to add a pot turn to get it
     back on the circle */
  rot_raw_val_prev = pot_data->raw_val_prev - calib_mid;
  if(rot_raw_val_prev < 0)
  {
    rot_raw_val_prev += CRAB_POT_RES;
  }

  /* Get the raw value of the pot rotated so that 0 is forward
     Also, if the value is negative, we need to add a pot turn to get it
     back on the circle */
  rot_raw_val = raw_val - calib_mid;
  if(rot_raw_val < 0)
  {
    rot_raw_val += CRAB_POT_RES;
  }

#if DEBUG_POT_ROLLOVER
  printf("RV %4d cm %4d rrvp %4d rrv %4d ", raw_val, calib_mid,
      rot_raw_val_prev, rot_raw_val);
#endif

  /* Update the spiral count based on the movement in the last loop */
  pot_data->spiral_count += calc_pot_rollover(rot_raw_val, rot_raw_val_prev);

  /* Find the back of the pot (half a pot turn from mid */
  if(calib_mid >= (CRAB_POT_RES / 2))
  {
    back_of_pot = calib_mid - (CRAB_POT_RES / 2);
  }
  else
  {
    back_of_pot = calib_mid + (CRAB_POT_RES / 2);
  }

  /* Based on where the raw value is, apply the spiral to it and store
     into the absolute value */
  if((raw_val >= 0) && (raw_val < back_of_pot))
  {
    pot_data->abs_val = ((INT16)CRAB_POT_RES * pot_data->spiral_count) + raw_val;
  }
  else
  {
    pot_data->abs_val = ((INT16)CRAB_POT_RES * (pot_data->spiral_count - 1)) + raw_val;
  }

  /* This is added because if the calibration midpoint is in this range, we
     have already added a spiral and need to account for it */
  if(calib_mid >= (CRAB_POT_RES / 2))
  {
    pot_data->abs_val += CRAB_POT_RES;
  }
}
#endif


