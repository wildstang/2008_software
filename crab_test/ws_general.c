/*******************************************************************************
* FILE NAME: ws_general.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "delays.h"       /*defined locally*/
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

  static int counter = 0;

  if(Dig_in_pressure == PRESSURE_BELOW_120)
  {
    counter = 0;
  }
  else if ((motor_vals.pump == PUMP_ON) && (disabled_mode == ROBOT_ENABLED))
  {
    /* Only increment when the robot is enabled */
      counter++;
  }

  //if(counter < PUMP_LOOP_COUNT)
  if(Dig_in_pressure == PRESSURE_BELOW_120)
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
* FUNCTION NAME: deploy_ramp_n_tower()
* PURPOSE:       release ramp and tower
*******************************************************************************/
void deploy_ramp_n_tower(void)
{
  static UINT8 ramp_press_ctr = 0;
  static UINT8 tower_press_ctr = 0;
  static UINT8 ramp_btn_safe = FALSE;
  static UINT8 tower_btn_safe = FALSE;

  /* only allow the ramp to deploy if button was not pressed while enabled &
     not in calibrate mode.  This safeguards against the switch being left in
     the deploy position when disabled */
  if ((disabled_mode == ROBOT_ENABLED) &&
      (Oi_calibrate > OI_CALIBRATE_ENCODERS))
  {
    if (Oi_sw_ramp_release == 0)
    {
      ramp_btn_safe = TRUE;
    }
  }
  else
  {
    ramp_btn_safe = FALSE;
  }

  /* only deploy the ramp if the button has been pressed for 10 loops */
  if ((Oi_sw_ramp_release == 1) && (ramp_btn_safe == TRUE))
  {
    if (motor_vals.ramp_release == RAMP_NO_RELEASE)
    {
      ramp_press_ctr++;
    }
  }
  else
  {
    ramp_press_ctr = 0;
  }

  if (ramp_press_ctr >= BTN_PRESS_COUNT)
  {
    motor_vals.ramp_release = RAMP_DEPLOY;
  }
  else
  {
    motor_vals.ramp_release = RAMP_NO_RELEASE;
  }

  /*
  printf("release ramp %d safe %d ctr %d ", motor_vals.ramp_release,
         ramp_btn_safe, ramp_press_ctr);
  */

  /* only allow the tower to deploy if button was not pressed while enabled &
     not in calibrate mode.  This safeguards against the switch being left in
     the deploy position when disabled */
  if ((disabled_mode == ROBOT_ENABLED) &&
      (Oi_calibrate > OI_CALIBRATE_ENCODERS))
  {
    if (Oi_sw_tower_release == 0)
    {
      tower_btn_safe = TRUE;
    }
  }
  else
  {
    tower_btn_safe = FALSE;
  }

  /* only deploy the tower if the button has been pressed for 10 loops */
  if ((Oi_sw_tower_release == 1) && (tower_btn_safe == TRUE))
  {
    if (motor_vals.tower_release == TOWER_NO_RELEASE)
    {
      tower_press_ctr++;
    }
  }
  else
  {
    tower_press_ctr = 0;
  }

  if (tower_press_ctr >= BTN_PRESS_COUNT)
  {
    motor_vals.tower_release = TOWER_RELEASE;
  }
  else
  {
    motor_vals.tower_release = TOWER_NO_RELEASE;
  }

  /*
  printf("tower %d safe %d ctr %d ", motor_vals.tower_release,
         tower_btn_safe, tower_press_ctr);
  */

  /* unlock the ramp if either the ramp or tower release buttons are pressed */
  if (((Oi_sw_ramp_release == 1) && (ramp_btn_safe == TRUE)) ||
      ((Oi_sw_tower_release == 1) && (tower_btn_safe == TRUE)))
  {
    motor_vals.ramp_lock = RAMP_UNLOCKED;
  }
  else
  {
    motor_vals.ramp_lock = RAMP_LOCKED;
  }

  /*
  printf("lock %d\r", motor_vals.ramp_lock);
  */
}

/*******************************************************************************
* FUNCTION NAME: acceleration_adjust
* PURPOSE:       Ramp up / down speed
******************************************************************************/
INT16 acceleration_adjust(INT16 desired_speed, INT16 curr_speed, UINT8 rate)
{
  INT16 new_speed = 0;

  if((desired_speed - curr_speed) > rate)
  {
    new_speed = curr_speed + rate;
  }
  else if((curr_speed - desired_speed) > rate)
  {
    new_speed = curr_speed - rate;
  }
  else
  {
    new_speed = desired_speed;
  }

  /*
  printf("D: %d C: %d N: %d\r", desired_speed, curr_speed,
         new_speed);
  */

  return new_speed;
}

