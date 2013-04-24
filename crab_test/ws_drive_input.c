/*******************************************************************************
* FILE NAME: ws_drive_input.c
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
#include "ws_io.h"
#include "ws_pid.h"
#include "ws_drive_input.h"
#include "ws_general.h"
#include "ws_feedback.h"
#include "ws_cc.h"
#include "ws_trig.h"

static INT16 sg_latched_theta = 0;
static UINT8 sg_hold_theta = FALSE;

static UINT8 is_crabbed;
static UINT8 car_drive;
CrabArcType do_crab_arc = CRAB_ARC_NONE;
PidValsType g_crab_front_pid_vals;
PidValsType g_crab_back_pid_vals;
extern CrabTgts g_crab_tgts;

/*******************************************************************************
* FUNCTION NAME: crab_stick_input
* PURPOSE:       initialize crab PID vals
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void crab_init()
{
#ifdef REAL_ROBOT
  g_crab_front_pid_vals.scale_factor = 80;
  g_crab_front_pid_vals.prop_gain = 3 * g_crab_front_pid_vals.scale_factor;
  g_crab_front_pid_vals.int_gain = 6;
  g_crab_front_pid_vals.deriv_gain = (-2 * g_crab_front_pid_vals.scale_factor);
  g_crab_front_pid_vals.integral = 0;
  g_crab_front_pid_vals.max_integral = (30 * g_crab_front_pid_vals.scale_factor);
  g_crab_front_pid_vals.min_val = -127;
  g_crab_front_pid_vals.max_val = 127;
  g_crab_front_pid_vals.int_reset_thresh = 4;
  g_crab_front_pid_vals.error_thresh = 1;
  pid_last_error_init(&g_crab_front_pid_vals);

  g_crab_back_pid_vals.scale_factor = 80;
  g_crab_back_pid_vals.prop_gain = 3 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.int_gain = 6;
  g_crab_back_pid_vals.deriv_gain = -2 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.integral = 0;
  g_crab_back_pid_vals.max_integral = 30 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.min_val = -127;
  g_crab_back_pid_vals.max_val = 127;
  g_crab_back_pid_vals.int_reset_thresh = 4;
  g_crab_back_pid_vals.error_thresh = 1;
  pid_last_error_init(&g_crab_back_pid_vals);

  g_crab_tgts.state = FEEDBACK_DISABLED;
  g_crab_tgts.front_pot_tgt = 127;
  g_crab_tgts.back_pot_tgt = 127;
#else
  g_crab_front_pid_vals.scale_factor = 80;
  g_crab_front_pid_vals.prop_gain = 3 * g_crab_front_pid_vals.scale_factor;
  g_crab_front_pid_vals.int_gain = 6;
  g_crab_front_pid_vals.deriv_gain = (-2 * g_crab_front_pid_vals.scale_factor);
  g_crab_front_pid_vals.integral = 0;
  g_crab_front_pid_vals.max_integral = (30 * g_crab_front_pid_vals.scale_factor);
  g_crab_front_pid_vals.min_val = -127;
  g_crab_front_pid_vals.max_val = 127;
  g_crab_front_pid_vals.int_reset_thresh = 4;
  g_crab_front_pid_vals.error_thresh = 1;
  pid_last_error_init(&g_crab_front_pid_vals);

  g_crab_back_pid_vals.scale_factor = 80;
  g_crab_back_pid_vals.prop_gain = 3 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.int_gain = 6;
  g_crab_back_pid_vals.deriv_gain = -2 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.integral = 0;
  g_crab_back_pid_vals.max_integral = 30 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.min_val = -127;
  g_crab_back_pid_vals.max_val = 127;
  g_crab_back_pid_vals.int_reset_thresh = 4;
  g_crab_back_pid_vals.error_thresh = 1;
  pid_last_error_init(&g_crab_back_pid_vals);

  g_crab_tgts.state = FEEDBACK_DISABLED;
  g_crab_tgts.front_pot_tgt = 127;
  g_crab_tgts.back_pot_tgt = 127;
#endif

}


/*******************************************************************************
* FUNCTION NAME: drive_preproc
* PURPOSE:       determine which mode the robot is in: crabbed, car drive,
*                or neither
*******************************************************************************/
void drive_preproc()
{
  /* check if wheels are crabbed to disable tank turning */
  /* in drive_stick_input() below */
  if ((Oi_crab_x < (127 - CRAB_TURN_DISABLE_VAL)) ||
      (Oi_crab_x > (127 + CRAB_TURN_DISABLE_VAL)))
  {
    is_crabbed = TRUE;
  }
  else
  {
    is_crabbed = FALSE;
  }

  if(Oi_sw_crab_arc_cw == 1)
  {
    do_crab_arc = CRAB_ARC_CW;
  }
  else if(Oi_sw_crab_arc_ccw == 1)
  {
    do_crab_arc = CRAB_ARC_CCW;
  }
  else
  {
    do_crab_arc = CRAB_ARC_NONE;
  }

  /* turn theta correction on if the driver isn't turning the robot and if
     the robot is supposed to be moving */
  if ((Oi_drive_x < (127 + 20)) && (Oi_drive_x > (127 - 20)) &&
      ((Oi_drive_y > (127 + 20)) || (Oi_drive_y < (127 - 20))))
  {
    /* only latch the orientation once */
    if (sg_hold_theta == FALSE)
    {
      sg_latched_theta = g_encoder_vals.orient;
      printf("theta lockin %3d\r", sg_latched_theta);
    }

    sg_hold_theta = TRUE;
  }
  else
  {
    sg_hold_theta = FALSE;
  }

  if (((g_encoder_vals.right_back > CAR_STEER_LIMIT) &&
       (g_encoder_vals.left_back > CAR_STEER_LIMIT)) ||
      ((g_encoder_vals.right_back < -CAR_STEER_LIMIT) &&
       (g_encoder_vals.left_back < -CAR_STEER_LIMIT)))
  {
    /* Checks to see if the robot is moving 'fast', is not crabbed, not in
       manual mode, and landing gear is up */
    if ((is_crabbed == FALSE) &&
        (Oi_sw_crab_manu_mode == 0) &&
        (Oi_sw_landing_gear == 0))
    {
      car_drive = TRUE;
    }
    else
    {
      car_drive = FALSE;
    }
  }
  else
  {
    car_drive = FALSE;
  }

  /*
  printf("dr x %3d y %3d; cr x %3d y %3d; Car %d Crab %d\r", Oi_drive_x,
         Oi_drive_y, Oi_crab_x, Oi_crab_y, car_drive, is_crabbed);
  */
}

/*******************************************************************************
* FUNCTION NAME: crab_stick_input
* PURPOSE:       process position of crab joystick
*******************************************************************************/
void crab_stick_input()
{
  INT16 crab_desired_pos_deg;
  INT8 front_diff = 0;
  INT8 back_diff = 0;

  //DEADZONE(Oi_crab_x, CRAB_DEADZONE);
  /*
   printf("OI-X %03d ", Oi_crab_x);
   */

  if (car_drive == FALSE)
  {
    if (Oi_sw_crab_manu_mode == 1)
    {
      /*
       * manual mode:
       * - disable feedback
       * - scale OI input to 1/2 speed crab motor output
       */
      g_crab_tgts.state = FEEDBACK_DISABLED;
      g_crab_tgts.front_pot_tgt = 127;
      g_crab_tgts.back_pot_tgt = 127;

      if(Oi_crab_y > (127 + CRAB_MANU_FB_RANGE))
      {
        motor_vals.front_crab = ((INT16)Oi_crab_x - 127) / 2;
        motor_vals.back_crab = 0;
      }
      else if(Oi_crab_y < (127 - CRAB_MANU_FB_RANGE))
      {
        motor_vals.front_crab = 0;
        motor_vals.back_crab = ((INT16)Oi_crab_x - 127) / 2;
      }
      else
      {
        motor_vals.front_crab = 0;
        motor_vals.back_crab = 0;
      }
    }
    else
    {
#if 0
      /* feedback mode - Super Crab Rev 3:
       * - scale joystick input to CRAB_LEFT_LIMIT to CRAB_RIGHT_LIMIT degree
       *   range
       * - enable feedback & set crab targets
       */
      if(Oi_crab_y <= (0 + CRAB_Y_LIVEZONE))
      {
        /* Y is in range for SuperCrab */
        if(Oi_crab_x <= (0 + CRAB_X_LIVEZONE))
        {
          /* SuperCrab right */
          crab_desired_pos_deg =
            (((CRAB_Y_LIVEZONE - (INT16)Oi_crab_y) *
              CRAB_PAST_90_DEG) / CRAB_Y_LIVEZONE) + 90;
        }
        else if(Oi_crab_x >= (254 - CRAB_X_LIVEZONE))
        {
          /* SuperCrab left */
          crab_desired_pos_deg =
            ((((INT16)Oi_crab_y - CRAB_Y_LIVEZONE) *
              CRAB_PAST_90_DEG) / CRAB_Y_LIVEZONE) - 90;
        }
        else
        {
          /* no SuperCrab */
          crab_desired_pos_deg = (((255 - (INT16)Oi_crab_x) * 12) / 17) - 90;
        }
      }
      else
      {
        /* no SuperCrab */
        crab_desired_pos_deg = (((255 - (INT16)Oi_crab_x) * 12) / 17) - 90;
      }

#else
      #define CRAB_DEAD 20
      if ((Oi_crab_x < (127 + CRAB_DEAD)) && (Oi_crab_x > (127 - CRAB_DEAD)) &&
          (Oi_crab_y < (127 + CRAB_DEAD)) && (Oi_crab_y > (127 - CRAB_DEAD)))
      {
        Oi_crab_x = 127;
        Oi_crab_y = 127;
      }

      if ((Oi_crab_y >= 127) && (Oi_crab_x >= 127))
      {
        /* upper left */
        crab_desired_pos_deg =
          ((17 * (UINT16)arctan(Oi_crab_x - 127, Oi_crab_y - 127)) / 12);
      }
      else if ((Oi_crab_y >= 127) && (Oi_crab_x < 127))
      {
        /* upper right */
        crab_desired_pos_deg =
          270 + ((17 * (UINT16)arctan(Oi_crab_y - 127, 127 - Oi_crab_x)) / 12);
      }
      else if ((Oi_crab_y < 127) && (Oi_crab_x >= 127))
      {
        /* lower left */
        crab_desired_pos_deg =
          90 + ((17 * (UINT16)arctan(127 - Oi_crab_y, Oi_crab_x - 127)) / 12);
      }
      else
      {
        /* lower right */
        crab_desired_pos_deg =
          180 + ((17 * (UINT16)arctan(127 - Oi_crab_x, 127 - Oi_crab_y)) / 12);
      }

      if (crab_desired_pos_deg >= 360)
      {
        crab_desired_pos_deg -= 360;
      }
      if (crab_desired_pos_deg >= 180)
      {
        crab_desired_pos_deg = (crab_desired_pos_deg - 360);
      }
      crab_desired_pos_deg = -crab_desired_pos_deg;
#endif

      printf("x %03d y %03d crab:pos %4d ", Oi_crab_x, Oi_crab_y, crab_desired_pos_deg);
      /*
      */

      //theta_correct(&front_diff, &back_diff);
#if DEBUGGING_THETA
      printf("cdiff f %d bd %d deg f %d b %d ", front_diff, back_diff,
             crab_desired_pos_deg + front_diff,
             crab_desired_pos_deg + back_diff);
#endif

      g_crab_tgts.state = FEEDBACK_ENABLED;
      /*
      g_crab_tgts.front_pot_tgt =
        get_crab_pot_tgt(crab_desired_pos_deg + front_diff,
                         &calibration_vals.front_crab);
                         */
      g_crab_tgts.back_pot_tgt =
        get_crab_pot_tgt(crab_desired_pos_deg + back_diff,
                         &calibration_vals.back_crab);

      /*
      printf("FT: %03d  BT: %03d ", g_crab_tgts.front_pot_tgt,
             g_crab_tgts.back_pot_tgt);
      */
      printf("BT: %03d ", g_crab_tgts.back_pot_tgt );
    }
  }
}


/*******************************************************************************
* FUNCTION NAME: theta_correct
* PURPOSE:
*******************************************************************************/
void theta_correct(INT8 *front_diff, INT8 *back_diff)
{
  INT16 signed_theta;

  *front_diff = 0;
  *back_diff = 0;

  /* theta correction */
  /* THETA CORRECT SWITCH OF 0 ON THE REAL OI IS ON */
  if ((sg_hold_theta == TRUE) && (Oi_sw_theta_correct == 1))
  {
    /* Calculate how far we are from the latched value
       we also need to scale the result to get it into
       a usable range */
    signed_theta = (g_encoder_vals.orient - sg_latched_theta) / 17;

    /* Limit the effect that theta correction can have */
    MIN_MAX(signed_theta, -15, 15);

    /*
     * theta increases clockwise, decreases counter-clockwise
     * when driving forward, adjust the front crab
     * when driving backward, adjust the back crab
     */

#if DEBUGGING_THETA
    printf("Tgt %d O %d st %d ", sg_latched_theta,
                                 g_encoder_vals.orient, signed_theta);
#endif

    if (Oi_drive_y > 127)
    {
      /* driving forward */
      *front_diff = -signed_theta;
    }
    else if (Oi_drive_y < 127)
    {
      /* driving backward */
      *back_diff = -signed_theta;
    }
  }

  /*
  printf("fd %3d bd %3d\r", *front_diff, *back_diff);
  */
}


/*******************************************************************************
* FUNCTION NAME: get_crab_pot_tgt
* PURPOSE:
*******************************************************************************/
UINT8 get_crab_pot_tgt(INT16 desired_pos_in_deg, CrabType *calibration)
{
  UINT16 desired_crab_pos_pot;

  /* range check desired position (must be between CRAB_LEFT_LIMIT &
     CRAB_RIGHT_LIMIT) */
  //MIN_MAX(desired_pos_in_deg, CRAB_LEFT_LIMIT, CRAB_RIGHT_LIMIT);

#if 0
  /* convert from -90<->90 to pot values (use left, mid, right calibration
     vals)*/
  if (desired_pos_in_deg > 0)
  {
    desired_crab_pos_pot =
      calibration->mid - (((INT16)desired_pos_in_deg *
                      (calibration->mid - calibration->right)) / 90);
  }
  else if (desired_pos_in_deg < 0)
  {
    desired_crab_pos_pot =
      calibration->mid - (((INT16)desired_pos_in_deg *
                      (calibration->left - calibration->mid)) / 90);
  }
  else
  {
    desired_crab_pos_pot = calibration->mid;
  }
#endif


#if 0
  /***********************************

    Pots increase in a counterclockwise direction

                    F
                    |
           Q2       |     Q1
                    |
                    |
      L-------------------------------R
                    |
                    |
           Q3       |     Q4
                    |
                    B

  ***********************************/

  if((desired_pos_in_deg > -180) && (desired_pos_in_deg <= -90))
  {
    /* Q3 */
    if(calibration->back < calibration->left)
    {
      /* Disjoint in this quadrant */
    }
    else
    {
      desired_crab_pos_pot =
        calibration->left + ((((INT16)desired_pos_in_deg + 90) *
                            (calibration->bot - calibration->left)) / -90);
    }
  }
  else if((desired_pos_in_deg > -90) && (desired_pos_in_deg <= 0))
  {
    /* Q2 */
    if(calibration->left < calibration->front)
    {
      /* Disjoint in this quadrant */
    }
    else
    {
      desired_crab_pos_pot =
        calibration->left + (((INT16)desired_pos_in_deg *
                            (calibration->front - calibration->left)) / 90);
    }
  }
  else if((desired_pos_in_deg > 0) && (desired_pos_in_deg <= 90))
  {
    /* Q1 */
    if(calibration->front < calibration->right)
    {
      /* Disjoint in this quadrant */
    }
    else
    {
      desired_crab_pos_pot =
        calibration->front + (((INT16)desired_pos_in_deg *
                            (calibration->right - calibration->front)) / 90);
    }
  }
  else
  {
    /* Q4 */
    if(calibration->right < calibration->back)
    {
      /* Disjoint in this quadrant */
    }
    else
    {
      desired_crab_pos_pot =
        calibration->bot + ((((INT16)desired_pos_in_deg - 180) *
                            (calibration->right - calibration->bot)) / -90);
    }
  }
#endif


  #define POT_RES        256
  /* NUM / DENOM == 256/360 */
  #define POT_RES_NUM    32
  #define POT_RES_DENOM  45

  desired_crab_pos_pot = ((POT_RES_NUM * (UINT16)(360 - desired_pos_in_deg)) / POT_RES_DENOM)
                           + calibration->mid;

  if(desired_crab_pos_pot >= POT_RES)
  {
    desired_crab_pos_pot -= POT_RES;
  }

  /*
  if(desired_crab_pos_pot > 255)
  {
    desired_crab_pos_pot -= 256;
  }
  else if(desired_crab_pos_pot < 0)
  {
    desired_crab_pos_pot += 256;
  }

  */

  return (UINT8)desired_crab_pos_pot;
}


/*******************************************************************************
* FUNCTION NAME: drive_stick_input
* PURPOSE:       Process position of drive joystick
*******************************************************************************/
void drive_stick_input(UINT8 use_deadzone)
{
  INT16  right_drive_speed, left_drive_speed;
  static INT16 left_drive_speed_prev = 0;
  static INT16 right_drive_speed_prev = 0;
  INT16  crab_desired_pos_deg;
  INT8   front_diff;
  INT8   back_diff;

  /*
  printf("x %3d y %3d ", (int)Oi_drive_x, (int)Oi_drive_y);
  */

  if (use_deadzone == TRUE)
  {
    DEADZONE(Oi_drive_x, DRIVE_DEADZONE);
    DEADZONE(Oi_drive_y, DRIVE_DEADZONE);
  }

  /* disable tank turning if wheels are crabbed */
  if(is_crabbed == TRUE)
  {
    Oi_drive_x = 127;
  }

  if(do_crab_arc != CRAB_ARC_NONE)
  {
    /* use crab arc mode
       - Drive Y stick controls throttle
       - Crab locked at 90 deg Right
       - ws_io will only run the back motors
    */
    left_drive_speed = (INT16)Oi_drive_y;
    right_drive_speed = (INT16)Oi_drive_y;

    g_crab_tgts.state = FEEDBACK_ENABLED;
    if(do_crab_arc == CRAB_ARC_CW)
    {
      /* Turn wheels 90 left */
      g_crab_tgts.front_pot_tgt =
        get_crab_pot_tgt(90, &calibration_vals.front_crab);
      g_crab_tgts.back_pot_tgt =
        get_crab_pot_tgt(90, &calibration_vals.back_crab);
    }
    else
    {
      /* Turn wheels 90 right */
      g_crab_tgts.front_pot_tgt =
        get_crab_pot_tgt(-90, &calibration_vals.front_crab);
      g_crab_tgts.back_pot_tgt =
        get_crab_pot_tgt(-90, &calibration_vals.back_crab);
    }
  }
  else if(car_drive == FALSE)
  {
    /* single stick drive calculation */
    left_drive_speed = ((INT16)Oi_drive_y - (INT16)Oi_drive_x + 127);
    right_drive_speed = ((INT16)Oi_drive_y + (INT16)Oi_drive_x - 127);
  }
  else
  {
    /* use car drive mode */
    left_drive_speed = (INT16)Oi_drive_y;
    right_drive_speed = (INT16)Oi_drive_y;

    crab_desired_pos_deg =
      (((255 - (INT16)Oi_drive_x) *
        CAR_DRIVE_DEG_RANGE) / 127) - CAR_DRIVE_DEG_RANGE;

    /*
    printf("X %03d ang %3d ", (int)Oi_drive_x, crab_desired_pos_deg);
    */

    theta_correct(&front_diff, &back_diff);
#if DEBUGGING_THETA
    printf("ddiff f %d bd %d deg f %d b %d ", front_diff, back_diff,
           crab_desired_pos_deg + front_diff,
           crab_desired_pos_deg + back_diff);
#endif

    if(Oi_drive_y > 127)
    {
      /* driving forward, steer front wheels and straighten back wheels */
      g_crab_tgts.state = FEEDBACK_ENABLED;
      g_crab_tgts.front_pot_tgt =
        get_crab_pot_tgt(crab_desired_pos_deg + front_diff,
                         &calibration_vals.front_crab);
      g_crab_tgts.back_pot_tgt =
        get_crab_pot_tgt(back_diff, &calibration_vals.back_crab);
    }
    else
    {
      /* driving backward, steer back wheels and straighten front wheels */
      g_crab_tgts.state = FEEDBACK_ENABLED;
      g_crab_tgts.front_pot_tgt =
        get_crab_pot_tgt(front_diff, &calibration_vals.front_crab);
      g_crab_tgts.back_pot_tgt =
        get_crab_pot_tgt(crab_desired_pos_deg + back_diff,
                         &calibration_vals.back_crab);
    }

#if DEBUGGING_THETA
    printf(" tgt F: %03d B: %03d ", g_crab_tgts.front_pot_tgt, g_crab_tgts.back_pot_tgt);
    /*
    */
#endif



    /*
    printf("car: front %3d   back %3d\r", motor_vals.front_crab,
                                          motor_vals.back_crab);
    */
  }

  MIN_MAX(left_drive_speed, 0, 254);
  MIN_MAX(right_drive_speed, 0, 254);

  /* center around 0 so anti-turbo calculation works */
  left_drive_speed -= 127;
  right_drive_speed -= 127;

  //printf("\r");

  /* anti-turbo */
  if((Oi_sw_turbo == 0) && (do_crab_arc == CRAB_ARC_NONE))
  {
    left_drive_speed *= DRIVE_SCALE_NUMERATOR;
    left_drive_speed /= DRIVE_SCALE_DENOMINATOR;
    right_drive_speed *= DRIVE_SCALE_NUMERATOR;
    right_drive_speed /= DRIVE_SCALE_DENOMINATOR;
  }

  /*
  printf("l %3d r %3d \r", left_drive_speed, right_drive_speed);
  */
  /* acceleration */
  left_drive_speed = acceleration_adjust(left_drive_speed,
                                         left_drive_speed_prev,
                                         DRIVE_ACCEL_RATE);
  right_drive_speed = acceleration_adjust(right_drive_speed,
                                          right_drive_speed_prev,
                                          DRIVE_ACCEL_RATE);

  left_drive_speed_prev = left_drive_speed;
  right_drive_speed_prev = right_drive_speed;

  /*
  printf("l %3d r %3d prev l %3d r %3d\r", left_drive_speed, right_drive_speed,
         left_drive_speed_prev, right_drive_speed_prev);
  */


  if(do_crab_arc == CRAB_ARC_NONE)
  {
    /* enable brake mode if drive wheels are turning slowly */
    if(((g_encoder_vals.right_back < BRAKE_MODE_LIMIT) &&
       (g_encoder_vals.right_back > -BRAKE_MODE_LIMIT)) &&
       ((g_encoder_vals.left_back < BRAKE_MODE_LIMIT) &&
       (g_encoder_vals.left_back > -BRAKE_MODE_LIMIT)))
    {
      motor_vals.brake_mode = BRAKE_MODE_ON;
    }
    else
    {
      motor_vals.brake_mode = BRAKE_MODE_OFF;
    }
  }
  else
  {
    motor_vals.brake_mode = BRAKE_MODE_ON;
  }

  /* fill in motor_vals structure */
  motor_vals.left_drive = left_drive_speed;
  motor_vals.right_drive = right_drive_speed;
  if(Oi_sw_landing_gear == 1)
  {
    motor_vals.landing_gear = LANDING_GEAR_DOWN;
  }
  else
  {
    motor_vals.landing_gear = LANDING_GEAR_UP;
  }
  return;
}
