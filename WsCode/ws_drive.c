/*******************************************************************************
* FILE NAME: ws_drive.c
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
#include "ws_trig.h"
#include "ws_pid.h"
#include "ws_drive.h"
#include "ws_autonomous.h"
INT16 apply_acceleration2(INT16 speed, INT16 speed_prev);
#define USE_TRACTION_ACCEL 1

#define MONSTER_LOOKUP_DIVISOR 100
UINT8 monster_diff_lookup[26] =
{100, 96, 92, 89, 85, 81, 77, 74, 70, 66, 62, 59, 55,
 51,  48, 44, 41, 37, 34, 30, 27, 24, 20, 17, 14, 11};

#define CROSSOVER_ZONE_MIN (128-50)
#define CROSSOVER_ZONE_MAX (128+50)

static INT16 sg_latched_theta = 0;
static UINT8 sg_hold_theta = FALSE;
static UINT8 toggle_tmp = 0;

UINT8 g_use_forced_theta = FALSE;
INT16 g_forced_theta = 0;

PotCheckVarsType back_pot_check_vars = {0, 0, 0};
PotCheckVarsType front_pot_check_vars = {0, 0, 0};

PidValsType g_crab_front_pid_vals;
PidValsType g_crab_back_pid_vals;
PidValsType g_diff_theta_pid_vals;

CrabTgts g_crab_tgts;
UINT8 g_current_spiral_pos_user = SPIRAL_POS_INIT;
UINT8 g_spiral_max = SPIRAL_POS_INIT;
UINT8 g_spiral_min = SPIRAL_POS_INIT;

/* Allows autonomous to use differential theta when set */
UINT8 g_use_differential_theta = 0;

/*******************************************************************************
* FUNCTION NAME: crab_init
* PURPOSE:       initialize crab PID vals
*******************************************************************************/
void crab_init()
{

#ifdef USE_DIGITAL_POT
  g_crab_back_pid_vals.scale_factor = 100;
  g_crab_back_pid_vals.prop_gain = 2 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.int_gain = 10;
  g_crab_back_pid_vals.deriv_gain = -0 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.integral = 0;
  g_crab_back_pid_vals.max_integral = 15 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.min_val = -127;
  g_crab_back_pid_vals.max_val = 127;
  g_crab_back_pid_vals.int_reset_thresh = 4;
  g_crab_back_pid_vals.error_thresh = 2;
  pid_last_error_init(&g_crab_back_pid_vals);

  g_crab_front_pid_vals.scale_factor = 200;
  g_crab_front_pid_vals.prop_gain = 1 * g_crab_front_pid_vals.scale_factor;
  g_crab_front_pid_vals.int_gain = 10;
  g_crab_front_pid_vals.deriv_gain = -0 * g_crab_front_pid_vals.scale_factor;
  g_crab_front_pid_vals.integral = 0;
  g_crab_front_pid_vals.max_integral = 15 * g_crab_front_pid_vals.scale_factor;
  g_crab_front_pid_vals.min_val = -127;
  g_crab_front_pid_vals.max_val = 127;
  g_crab_front_pid_vals.int_reset_thresh = 4;
  g_crab_front_pid_vals.error_thresh = 2;
  pid_last_error_init(&g_crab_front_pid_vals);

#else

  g_crab_back_pid_vals.scale_factor = 100;
  g_crab_back_pid_vals.prop_gain = 5 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.int_gain = 5;
  g_crab_back_pid_vals.deriv_gain = -0 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.integral = 0;
  g_crab_back_pid_vals.max_integral = 15 * g_crab_back_pid_vals.scale_factor;
  g_crab_back_pid_vals.min_val = -127;
  g_crab_back_pid_vals.max_val = 127;
  g_crab_back_pid_vals.int_reset_thresh = 12;
  g_crab_back_pid_vals.error_thresh = 2;
  pid_last_error_init(&g_crab_back_pid_vals);

  g_crab_front_pid_vals.scale_factor = 100;
  g_crab_front_pid_vals.prop_gain = 5 * g_crab_front_pid_vals.scale_factor;
  g_crab_front_pid_vals.int_gain = 10;
  g_crab_front_pid_vals.deriv_gain = -0 * g_crab_front_pid_vals.scale_factor;
  g_crab_front_pid_vals.integral = 0;
  g_crab_front_pid_vals.max_integral = 15 * g_crab_front_pid_vals.scale_factor;
  g_crab_front_pid_vals.min_val = -127;
  g_crab_front_pid_vals.max_val = 127;
  g_crab_front_pid_vals.int_reset_thresh = 12;
  g_crab_front_pid_vals.error_thresh = 2;
  pid_last_error_init(&g_crab_front_pid_vals);
#endif

  g_diff_theta_pid_vals.scale_factor = 2;
  g_diff_theta_pid_vals.prop_gain = 3 * g_diff_theta_pid_vals.scale_factor;
  g_diff_theta_pid_vals.int_gain = 0;
  g_diff_theta_pid_vals.deriv_gain = -0 * g_diff_theta_pid_vals.scale_factor;
  g_diff_theta_pid_vals.integral = 0;
  g_diff_theta_pid_vals.max_integral = 15 * g_diff_theta_pid_vals.scale_factor;
  g_diff_theta_pid_vals.min_val = -127;
  g_diff_theta_pid_vals.max_val = 127;
  g_diff_theta_pid_vals.int_reset_thresh = 12;
  g_diff_theta_pid_vals.error_thresh = 2;
  pid_last_error_init(&g_diff_theta_pid_vals);

  g_crab_tgts.state = FEEDBACK_DISABLED;
}


/*******************************************************************************
* FUNCTION NAME: drive_control
* PURPOSE:       single function that handles all the drive functionality
*******************************************************************************/
void drive_control(UINT8 use_deadzone)
{
  DriveModeType drive_mode;
  UINT8 first_loop;

  /* need this to turn manual mode off */
  motor_vals.front_crab = 0;
  motor_vals.back_crab = 0;

  drive_mode = drive_mode_select(&first_loop);

  if (drive_mode == DRIVE_MODE_MONSTER)
  {
#if DRIVE_PRINTS
    printf("dmm ");
#endif
    drive_mode_monster(TRUE);
  }
  else if (drive_mode == DRIVE_MODE_CRAB)
  {
#if DRIVE_PRINTS
    printf("dmc ");
#endif
    drive_mode_crab(first_loop);
  }
  else if (drive_mode == DRIVE_MODE_TANK)
  {
#if DRIVE_PRINTS
    printf("dmt ");
#endif
    drive_mode_tank(use_deadzone);
  }
  else if (drive_mode == DRIVE_MODE_CAR)
  {
#if DRIVE_PRINTS
    printf("dmc ");
#endif
    drive_mode_monster(FALSE);
  }
  else
  {
#if DRIVE_PRINTS
    printf("dm? ");
#endif
  }

  /* enable brake mode if the drive wheels are turning slowly */
  if ((cc_data.type == CC_REQ_ENCODER) &&
      (cc_data.data.encoder.left >= -BRAKE_MODE_THRESHOLD) &&
      (cc_data.data.encoder.left <= BRAKE_MODE_THRESHOLD) &&
      (cc_data.data.encoder.right >= -BRAKE_MODE_THRESHOLD) &&
      (cc_data.data.encoder.right <= BRAKE_MODE_THRESHOLD))
  {
    motor_vals.brake_mode = BRAKE_MODE_ON;
  }
  else
  {
    motor_vals.brake_mode = BRAKE_MODE_OFF;
  }

#if DRIVE_PRINTS
  printf("l %3d r %3d ft %4d bt %4d ", motor_vals.left_drive,
      motor_vals.right_drive, g_crab_tgts.front_pot_tgt,
      g_crab_tgts.back_pot_tgt);
#endif
}


/*******************************************************************************
* FUNCTION NAME: drive_mode_select
* PURPOSE:       determine which mode the robot is in: crabbed, car drive,
*                or tank
*******************************************************************************/
DriveModeType drive_mode_select(UINT8 *first_loop)
{
  static DriveModeType drive_mode_prev = DRIVE_MODE_UNINIT;
  DriveModeType drive_mode = DRIVE_MODE_UNINIT;


  /* only allow feedback if calibration is good */
  if (calibration_vals.state == CALIBRATION_GOOD)
  {
    /* determine if crab should be enabled or disabled */
    if (Oi_sw_crab_manu_mode == 0)
    {
#if 1
      /* switch says crab feedback should be enabled, but only enable it if the
         pot is not broken */
      if (g_crab_tgts.state != FEEDBACK_POT_BROKEN)
      {
        g_crab_tgts.state = FEEDBACK_ENABLED;
      }
      else
      {
        /* turn on red lights */
        Pwm1_red = 1;
        Pwm2_red = 1;
        Relay1_red = 1;
        Relay2_red = 1;
      }
#else
      g_crab_tgts.state = FEEDBACK_ENABLED;
#endif
    }
    else
    {
      g_crab_tgts.state = FEEDBACK_DISABLED;

      /* initialize dead pot check variables */
      front_pot_check_vars.crab_speed_prev = 0;
      front_pot_check_vars.pot_prev = pot_vals.front_crab.abs_val;
      front_pot_check_vars.bad_loop_counter = 0;
      back_pot_check_vars.crab_speed_prev = 0;
      back_pot_check_vars.pot_prev = pot_vals.back_crab.abs_val;
      back_pot_check_vars.bad_loop_counter = 0;
    }
  }
  else
  {
    g_crab_tgts.state = FEEDBACK_DISABLED;
    Pwm1_red = 1;
    Pwm2_red = 0;
    Relay1_red = 1;
    Relay2_red = 0;
  }


  if ((Oi_sw_car == 1))
  {
    drive_mode = DRIVE_MODE_CAR;
  }
  else if ((Oi_sw_landing_gear == 1))
  {
    drive_mode = DRIVE_MODE_TANK;
  }
  else if (Oi_sw_monster_mode == 1)
  {
    if (g_crab_tgts.state == FEEDBACK_ENABLED)
    {
      drive_mode = DRIVE_MODE_MONSTER;
    }
    else
    {
      drive_mode = DRIVE_MODE_TANK;
    }
  }
  else if ((Oi_crab_x < (127 - CRAB_TURN_DISABLE_VAL_X)) ||
           (Oi_crab_x > (127 + CRAB_TURN_DISABLE_VAL_X)) ||
           (Oi_crab_y < (127 - CRAB_TURN_DISABLE_VAL_Y)) ||
           (Oi_crab_y > (127 + CRAB_TURN_DISABLE_VAL_Y)))
  {
    /* if crab joystick is in crab range, crab drive is enabled */
    drive_mode = DRIVE_MODE_CRAB;
  }
  else
  {
    drive_mode = DRIVE_MODE_TANK;
  }

#if JOYSTICK_PRINTS
  printf("dx %3d dy %3d; cx %3d cy %3d M %d ", Oi_drive_x,
         Oi_drive_y, Oi_crab_x, Oi_crab_y, drive_mode);
#endif

  if (drive_mode == drive_mode_prev)
  {
    *first_loop = FALSE;
  }
  else
  {
    *first_loop = TRUE;
  }

  drive_mode_prev = drive_mode;

  return drive_mode;
}


/*******************************************************************************
* FUNCTION NAME: drive_mode_monster
* PURPOSE:       convert joystick inputs to drive motor speed and crab targets
*                when robot is in monster steering mode ("monster turn truck")
*******************************************************************************/
void drive_mode_monster(UINT8 steer_back)
{
  UINT8 crab_desired_pos_brad_front, crab_desired_pos_brad_back;
  INT8 monster_brad_offset;
  INT16 outer_speed;
  INT16 inner_speed;

  /* landing gear is always up in monster mode */
  motor_vals.landing_gear = LANDING_GEAR_UP;

  /* if the crabs are turned outside the sprial limits, bring them back */
  ENFORCE_SPIRAL_LIMITS(g_current_spiral_pos_user, g_spiral_min, g_spiral_max);

  /* clear the latched theta */
  sg_hold_theta = FALSE;

  /* Default to straight position, no monster turn */
  crab_desired_pos_brad_front = calibration_vals.front_crab.pot_mid;
  crab_desired_pos_brad_back  = calibration_vals.back_crab.pot_mid;

  monster_brad_offset = (MAX_MONSTER_OFFSET * ((INT16)Oi_drive_x - 127)) / 127;
  monster_brad_offset -=
    (monster_brad_offset * (MONSTER_SPEED_SCALE_NUM *
                          ABS((INT16)Oi_drive_y - 127) /
                          MONSTER_SPEED_SCALE_DENOM)) / 127;

  g_crab_tgts.front_pot_tgt =
    convert_brad_to_pot((UINT8)monster_brad_offset,
        &calibration_vals.front_crab, 0);

  if (steer_back == TRUE)
  {
    g_crab_tgts.back_pot_tgt =
      convert_brad_to_pot(0 - (UINT8)monster_brad_offset,
          &calibration_vals.back_crab, 0);
  }

  /* ADD DEADZONE */
  DEADZONE(Oi_drive_x, MONSTER_DRIVE_DEADZONE);
  DEADZONE(Oi_drive_y, MONSTER_DRIVE_DEADZONE);

  /* pass 127 as first argument so the drive speed calculation doesn't produce
     any differential steering */
  drive_speed_calc(127, Oi_drive_y);

  /*
   * drive the outer wheels faster than the inner wheels; use the inner wheels
   * as the base and add speed to the outer wheels.  We can use either the
   * right or left drive speed for the inner speed because they're both the
   * same right now.  If the outer wheels are maxed out, subtract speed from
   * the inner wheels.  This isn't exact, but it's close enough.
   *
   * Inner speed / Outer speed = monster_diff_lookup[monster_offset] / DIVISOR
   *
   */
  inner_speed = motor_vals.right_drive;
  outer_speed = (inner_speed * MONSTER_LOOKUP_DIVISOR) /
                monster_diff_lookup[ABS(monster_brad_offset)];

  //printf("in %3d out %3d ", inner_speed, outer_speed);

  /* if the outer wheel speed is outside the -127 to 127 range, subtract from
     the inner and set the outer to -127/127 */
  if (outer_speed > 127)
  {
    inner_speed -= (outer_speed - 127);
    outer_speed = 127;
  }
  else if (outer_speed < -127)
  {
    inner_speed -= (outer_speed + 127);
    outer_speed = -127;
  }

  //printf("in %3d out %3d ", inner_speed, outer_speed);

  MIN_MAX(inner_speed, -127, 127);
  MIN_MAX(outer_speed, -127, 127);

  if (Oi_drive_x > 127)
  {
    /* turning left: left is inner, right is outer */
    motor_vals.right_drive = outer_speed;
    motor_vals.left_drive = inner_speed;
  }
  else
  {
    /* turning right: right is inner, left is outer */
    motor_vals.right_drive = inner_speed;
    motor_vals.left_drive = outer_speed;
  }

#if DRIVE_PRINTS
  printf("BO: %4d FT: %4d BT: %4d  ", monster_brad_offset,
         g_crab_tgts.front_pot_tgt, g_crab_tgts.back_pot_tgt);
#endif
}


/*******************************************************************************
* FUNCTION NAME: drive_mode_crab
* PURPOSE:       convert joystick inputs to drive motor speed and crab targets
*                when robot is in monster steering mode ("monster turn truck")
*******************************************************************************/
void drive_mode_crab(UINT8 first_loop)
{
  static UINT8 crab_desired_pos_brad_prev = 0;
  UINT8 crab_desired_pos_brad;
  INT8 front_diff = 0;
  INT8 back_diff = 0;

  //DEADZONE(Oi_crab_x, CRAB_DEADZONE);

  /* initialize to 0 when we enter crab mode for the first time */
  if (first_loop == TRUE)
  {
    crab_desired_pos_brad_prev = 0;
  }

  /* landing gear is always up in crab mode */
  motor_vals.landing_gear = LANDING_GEAR_UP;

  if (g_crab_tgts.state == FEEDBACK_ENABLED)
  {
    /*
     * feedback mode:
     * - get angle of joystick relative to straight up (in brads)
     * - theta correct - use gyro data to offset front or back crabs to
     *   correct skew (still in brads)
     * - convert front & back targets from brads to pot ticks
     */

    /* add deadzone to the crab joystick */
    if ((Oi_crab_x < (127 + CRAB_DEAD_X)) &&
        (Oi_crab_x > (127 - CRAB_DEAD_X)) &&
        (Oi_crab_y < (127 + CRAB_DEAD_Y)) &&
        (Oi_crab_y > (127 - CRAB_DEAD_Y)))
    {
      Oi_crab_x = 127;
      Oi_crab_y = 127;
    }


    crab_desired_pos_brad =  convert_joystick_to_brads(Oi_crab_x, Oi_crab_y);

#if LIMIT_CRAB_90 == 0
    /* check whether the joystick crossed the spiral 0 point */
    if ((crab_desired_pos_brad >= CROSSOVER_ZONE_MIN) &&
        (crab_desired_pos_brad < 128) &&
        (crab_desired_pos_brad_prev <= CROSSOVER_ZONE_MAX) &&
        (crab_desired_pos_brad_prev >= 128))
    {
      if(g_current_spiral_pos_user >= g_spiral_min)
      {
        g_current_spiral_pos_user--;
      }
    }
    else if ((crab_desired_pos_brad <= CROSSOVER_ZONE_MAX) &&
             (crab_desired_pos_brad >= 128) &&
             (crab_desired_pos_brad_prev >= CROSSOVER_ZONE_MIN) &&
             (crab_desired_pos_brad_prev < 128))
    {
      if(g_current_spiral_pos_user <= g_spiral_max)
      {
        g_current_spiral_pos_user++;
      }
    }
    else
    {
    }

    /* allow the crab wheels to move 32 brads past straight back in both
       directions */
    if ((g_current_spiral_pos_user > g_spiral_max) &&
        ((crab_desired_pos_brad > 192) || (crab_desired_pos_brad < 128)))
    {
       crab_desired_pos_brad = 192;
    }
    else if ((g_current_spiral_pos_user < g_spiral_min) &&
             ((crab_desired_pos_brad > 128) || (crab_desired_pos_brad < 64)))
    {
       crab_desired_pos_brad = 64;
    }
#endif

#if DRIVE_PRINTS
    printf("x %03d y %03d cp %3d sp %d ", Oi_crab_x, Oi_crab_y,
           crab_desired_pos_brad, g_current_spiral_pos_user);
#if USE_DIGITAL_POT
    printf("fr %3d fs %3d fa %4d br %3d bs %3d ba %4d ",
           pot_vals.front_crab.raw_val, pot_vals.front_crab.spiral_count,
           pot_vals.front_crab.abs_val, pot_vals.back_crab.raw_val,
           pot_vals.back_crab.spiral_count, pot_vals.back_crab.abs_val);
#else
    printf("fa %4d ba %4d ", pot_vals.front_crab.abs_val,
           pot_vals.back_crab.abs_val);
#endif
#endif

    /* use theta correction in crab mode when the y-axis of the drive stick is
       deflected; latch the orientation if we have good CC data and it hasn't
       been latched already */
    if ((Oi_drive_y > (127 + 20)) || (Oi_drive_y < (127 - 20)))
    {
      if ((cc_data.type == CC_REQ_ENCODER) && (sg_hold_theta == FALSE))
      {
        sg_latched_theta = cc_data.data.encoder.orient;
        sg_hold_theta = TRUE;
      }
    }
    else
    {
      sg_hold_theta = FALSE;
    }

    theta_correct(&front_diff, &back_diff, crab_desired_pos_brad);

#if THETA_PRINTS
    printf("cdiff f %d b %d brad f %d b %d ", front_diff, back_diff,
        crab_desired_pos_brad + front_diff,
        crab_desired_pos_brad + back_diff);
#endif

    g_crab_tgts.front_pot_tgt =
      convert_brad_to_pot(crab_desired_pos_brad,
                          &calibration_vals.front_crab, front_diff);

    g_crab_tgts.back_pot_tgt =
      convert_brad_to_pot(crab_desired_pos_brad,
                          &calibration_vals.back_crab, back_diff);


#if DRIVE_PRINTS
    printf("FT: %04d  BT: %04d ", g_crab_tgts.front_pot_tgt,
           g_crab_tgts.back_pot_tgt);
#endif
  }
  else
  {
    /*
     * manual mode:
     * - scale OI input to 1/2 speed crab motor output
     */
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

  /* pass 127 as first argument so the drive speed calculation doesn't produce
     any differential steering */
  drive_speed_calc(127, Oi_drive_y);

  crab_desired_pos_brad_prev = crab_desired_pos_brad;
}


/*******************************************************************************
* FUNCTION NAME: drive_mode_tank
* PURPOSE:       convert joystick inputs to drive motor speed when robot is in
*                tank steering mode
*******************************************************************************/
void drive_mode_tank(UINT8 use_deadzone)
{
  UINT8 crab_desired_pos_brad_front, crab_desired_pos_brad_back;
  INT8 front_diff = 0;
  INT8 back_diff = 0;
  INT8 x_stick_diff = 0;

#if DRIVE_PRINTS
  printf("x %3d y %3d ", (int)Oi_drive_x, (int)Oi_drive_y);
#endif

  /* if the landing gear button is pressed, lower it, otherwise raise it */
  if (Oi_sw_landing_gear == 1)
  {
    motor_vals.landing_gear = LANDING_GEAR_DOWN;
  }
  else
  {
    motor_vals.landing_gear = LANDING_GEAR_UP;
  }

  /* if the crabs are turned outside the sprial limits, bring them back */
  ENFORCE_SPIRAL_LIMITS(g_current_spiral_pos_user, g_spiral_min, g_spiral_max);

  /* use theta correction in tank mode when the x-axis of the drive stick is
     close to center and the y-axis is not; latch the orientation if we have
     good CC data and it hasn't been latched already */
  if ((Oi_drive_x < (127 + 5)) && (Oi_drive_x > (127 - 5)) &&
      ((Oi_drive_y > (127 + 20)) || (Oi_drive_y < (127 - 20))))
  {
    if ((cc_data.type == CC_REQ_ENCODER) && (sg_hold_theta == FALSE))
    {
      sg_latched_theta = cc_data.data.encoder.orient;
      sg_hold_theta = TRUE;
    }
  }
  else
  {
    /* clear the latched theta */
    sg_hold_theta = FALSE;
  }

  if(!g_use_differential_theta)
  {
    theta_correct(&front_diff, &back_diff, 0);
  }
  else
  {
    /* Limit differential theta to only when we're driving fast.
       Note that for simplicity we're limiting this to driving forward.
       If we use this in the future, be sure to make it work in both 
       directions :) */
    if(Oi_drive_y > 240)
    {
      theta_correct_differential(&x_stick_diff);
      if(((INT16)Oi_drive_x + x_stick_diff) > 255)
      {
        Oi_drive_x = 255;
      }
      else if(((INT16)Oi_drive_x + x_stick_diff) < 0)
      {
        Oi_drive_x= 0;
      }
      else
      {
        Oi_drive_x += x_stick_diff;
      }
    }
    front_diff = 0;
    back_diff = 0;
  }

  /* set crab target to center */
  if (g_crab_tgts.state == FEEDBACK_ENABLED)
  {
    g_crab_tgts.front_pot_tgt =
      convert_brad_to_pot(0, &calibration_vals.front_crab, front_diff);
    g_crab_tgts.back_pot_tgt =
      convert_brad_to_pot(0, &calibration_vals.back_crab, back_diff);
  }

  if(use_deadzone == TRUE)
  {
    DEADZONE(Oi_drive_x, TANK_DRIVE_DEADZONE);
    DEADZONE(Oi_drive_y, TANK_DRIVE_DEADZONE);
  }

  drive_speed_calc(Oi_drive_x, Oi_drive_y);

  return;
}


/*******************************************************************************
* FUNCTION NAME: drive_speed_calc
* PURPOSE:       use X and Y axis values to calculate motor values for
*                differntial steering
*******************************************************************************/
void drive_speed_calc(UINT8 x_axis, UINT8 y_axis)
{
  INT16  right_drive_speed, left_drive_speed;
  static INT16 right_drive_speed_prev = 0;
  static INT16 left_drive_speed_prev = 0;

  /* single stick drive calculation */
  left_drive_speed = ((INT16)y_axis - (INT16)x_axis + 127);
  right_drive_speed = ((INT16)y_axis + (INT16)x_axis - 127);

  MIN_MAX(left_drive_speed, 0, 254);
  MIN_MAX(right_drive_speed, 0, 254);

  /* center around 0 so anti-turbo calculation works */
  left_drive_speed -= 127;
  right_drive_speed -= 127;

  if(Oi_sw_turbo == 1)
  {
    /* anti-turbo */
    left_drive_speed = (left_drive_speed * DRIVE_SCALE_NUM) /
                                           DRIVE_SCALE_DENOM;
    right_drive_speed = (right_drive_speed * DRIVE_SCALE_NUM) /
                         DRIVE_SCALE_DENOM;
  }

  /* acceleration */
#if USE_TRACTION_ACCEL
  left_drive_speed = apply_acceleration2(left_drive_speed,
                                        left_drive_speed_prev);
  right_drive_speed = apply_acceleration2(right_drive_speed,
                                         right_drive_speed_prev);
  if(toggle_tmp > 2)
  {
      toggle_tmp = 0;
  }
  else
  {
      toggle_tmp++;
  }
#else
  left_drive_speed = apply_acceleration(left_drive_speed,
                                        left_drive_speed_prev);
  right_drive_speed = apply_acceleration(right_drive_speed,
                                         right_drive_speed_prev);
#endif


  /* fill in motor_vals structure */
  motor_vals.left_drive = left_drive_speed;
  motor_vals.right_drive = right_drive_speed;
  right_drive_speed_prev = right_drive_speed;
  left_drive_speed_prev = left_drive_speed;
}


/*******************************************************************************
* FUNCTION NAME: theta_correct
* PURPOSE:
*******************************************************************************/
void theta_correct(INT8 *front_diff, INT8 *back_diff, UINT8 desired_brad)
{
  INT16 signed_theta;
  INT16 target_heading;

  *front_diff = 0;
  *back_diff = 0;

  if(g_use_forced_theta)
  {
    target_heading = g_forced_theta;
    sg_latched_theta = g_forced_theta;
    sg_hold_theta = TRUE;
  }
  else
  {
    target_heading = sg_latched_theta;
  }

  /* theta correction should be used when a theta is latched, the Oi switch is
     on, and the CC returned good data */
  if ((sg_hold_theta == TRUE) && (Oi_sw_theta_correct == 1) &&
      (cc_data.type == CC_REQ_ENCODER))
  {
    /* Calculate how far we are from the latched value
       we also need to scale the result to get it into
       a usable range */
    signed_theta = (cc_data.data.encoder.orient - target_heading) / 24;

    /* Limit the effect that theta correction can have */
    MIN_MAX(signed_theta, -11, 11);

    /*
     * theta increases clockwise, decreases counter-clockwise
     * when driving forward, adjust the front crab
     * when driving backward, adjust the back crab
     */

#if THETA_PRINTS
    printf("Tgt %d O %d st %d ", sg_latched_theta, cc_data.data.encoder.orient,
           signed_theta);
#endif

    if (((Oi_drive_y > 127) &&
         ((desired_brad <= 64) || (desired_brad >= 192))) ||
        ((Oi_drive_y < 127) &&
         ((desired_brad > 64) && (desired_brad < 192))))
    {
      /* robot is moving forward (kindof) */
      *back_diff = -signed_theta;
    }
    else
    {
      /* driving backward (kindof) */
      *front_diff = -signed_theta;
    }
  }
}

void theta_correct_differential(INT8 *x_stick_diff)
{
  INT16 signed_theta;

  *x_stick_diff = 0;

  /* theta correction should be used when a theta is latched, the Oi switch is
     on, and the CC returned good data */
  if ((sg_hold_theta == TRUE) && (Oi_sw_theta_correct == 1) &&
      (cc_data.type == CC_REQ_ENCODER))
  {
    /* Calculate how far we are from the latched value
       we also need to scale the result to get it into
       a usable range */
    /*
    show_pid = 1;
    */
    signed_theta = ws_pid(&g_diff_theta_pid_vals, cc_data.data.encoder.orient, sg_latched_theta);
    /*
    show_pid = 0;
    */

    /*
     * theta increases clockwise, decreases counter-clockwise
     * when driving forward, adjust the front crab
     * when driving backward, adjust the back crab
     */

#if THETA_PRINTS
    printf("Tgt %d O %d st %d ", sg_latched_theta, cc_data.data.encoder.orient,
           signed_theta);
#endif

    *x_stick_diff = -signed_theta;
  }
}

/*******************************************************************************
* FUNCTION NAME: convert_joystick_to_brads
* PURPOSE:
*******************************************************************************/
UINT8 convert_joystick_to_brads(UINT8 x, UINT8 y)
{
  UINT8 brads;

  /* convert joystick position into an angle from vertical in brads */
  if ((y >= 127) && (x >= 127))
  {
    /* upper left */
    brads = arctan(x - 127, y - 127);
  }
  else if ((y >= 127) && (x < 127))
  {
    /* upper right */
    brads = 192 + arctan(y - 127, 127 - x);
  }
  else if ((y < 127) && (x >= 127))
  {
    /* lower left */
    brads = 64 + arctan(127 - y, x - 127);
#if LIMIT_CRAB_90
    brads = 64;
#endif
  }
  else
  {
    /* lower right */
    brads = 128 + arctan(127 - x, 127 - y);
#if LIMIT_CRAB_90
    brads = 192;
#endif
  }
}


/*******************************************************************************
* FUNCTION NAME: convert_brads_to_joystick
* PURPOSE:
*******************************************************************************/
void convert_brads_to_joystick(UINT8 brads, UINT8 *x, UINT8 *y)
{
  INT16 l_x, l_y;

  /* the l_x and l_y values calculated will be in the range of
     -127 to 127 */
  if((brads >= 0) && (brads < 32))
  {
    l_x = (-BRAD_TO_JOY_MAX_X * (INT16)brads ) / 32;
    l_y = BRAD_TO_JOY_MAX_Y;
  }
  else if((brads >= 32) && (brads < 64))
  {
    l_x = -BRAD_TO_JOY_MAX_X;
    l_y = (BRAD_TO_JOY_MAX_Y * ((INT16)brads - 64)) / (32 - 64);
  }
  else if((brads >= 64) && (brads < 96))
  {
    l_x = -BRAD_TO_JOY_MAX_X;
    l_y = (-BRAD_TO_JOY_MAX_Y * ((INT16)brads - 64)) / (96 - 64);
  }
  else if((brads >= 96) && (brads < 128))
  {
    l_x = (-BRAD_TO_JOY_MAX_X * ((INT16)brads - 128)) / (96 - 128);
    l_y = -BRAD_TO_JOY_MAX_Y;
  }
  else if((brads >= 128) && (brads < 160))
  {
    l_x = (BRAD_TO_JOY_MAX_X * ((INT16)brads - 128)) / (160 - 128);
    l_y = -BRAD_TO_JOY_MAX_Y;
  }
  else if((brads >= 160) && (brads < 192))
  {
    l_x = BRAD_TO_JOY_MAX_X;
    l_y = (-BRAD_TO_JOY_MAX_Y * ((INT16)brads - 192)) / (160 - 192);
  }
  else if((brads >= 192) && (brads < 224))
  {
    l_x = BRAD_TO_JOY_MAX_X;
    l_y = (BRAD_TO_JOY_MAX_Y * ((INT16)brads - 192)) / (224 - 192);
  }
  else
  {
    l_x = (BRAD_TO_JOY_MAX_X * ((INT16)brads - 255)) / (224 - 255);
    l_y = BRAD_TO_JOY_MAX_Y;
  }

  /* In reality, if the calculations are correct above we don't need to do this,
     but it's better to be safe */
  MIN_MAX(l_x, -BRAD_TO_JOY_MAX_X, BRAD_TO_JOY_MAX_X);
  MIN_MAX(l_y, -BRAD_TO_JOY_MAX_Y, BRAD_TO_JOY_MAX_Y);

  /* Convert from -127-127 to 0-255.  The x axis needs to be inverted */
  *x = (UINT8) (127 - l_x);
  *y = (UINT8) (l_y + 127);
}


/*******************************************************************************
* FUNCTION NAME: convert_brad_to_pot
* PURPOSE:
*******************************************************************************/
UINT16 convert_brad_to_pot(UINT8 brad_val, CrabType *calibration,
                           INT8 brad_offset)
{
  UINT16 brad_val_abs;
  INT16 brad_diff;
  UINT16 pot_res;
  INT16 pot_diff;
  UINT16 pot_val;

#if USE_DIGITAL_POT
  /* The rollover point for brads is in the front so it first needs to be
     shifted to the back by adding 128.*/
  brad_val += 128;
  /* Put the brad value on the brad spiral */
  brad_val_abs = brad_val + (256 * (UINT16)g_current_spiral_pos_user) +
                 brad_offset;

  /* Convert the absolute brad position to an absolute pot position */
  pot_val = (brad_val_abs << CRAB_STICK_TO_POT_SHIFT) + calibration->pot_mid;

  /* Since we pre-rotated the brad value, un-rotate the pot value */
  pot_val -= (CRAB_POT_RES / 2);

#else

  /* the rollover point for brads is in the front so it first needs to be
     shifted to the back by adding 128 */
  brad_val += 128;
  /* put the brad value on the brad spiral */
  brad_val_abs = brad_val + (256 * (UINT16)g_current_spiral_pos_user) +
                 brad_offset;
  /* reorient so the middle is 256 * SPIRAL_POS_INIT */
  brad_val_abs -= 128;

  /* determine the desired distance from brad center */
  brad_diff = brad_val_abs - (256 * (UINT16)SPIRAL_POS_INIT);

  if (brad_diff > 0)
  {
    /* turning CCW, use (left - mid) for resolution */
    pot_res = calibration->pot_left - calibration->pot_mid;

    /* since the blue pots cannot turn as far, we'll limit travel to 110
       degrees (78 brads) in each direction */
    if (brad_diff > 78)
    {
      brad_diff = 78;
    }
  }
  else
  {
    /* turning CW, use (mid -> right) for resolution */
    pot_res = calibration->pot_mid - calibration->pot_right;

    /* since the blue pots cannot turn as far, we'll limit travel to 110
       degrees (78 brads) in each direction */
    if (brad_diff < -78)
    {
      brad_diff = -78;
    }
  }

  /* convert brad_diff to pot ticks & add it to the pot center */
  pot_diff = ((INT32)brad_diff * pot_res) / 64;

  pot_val = calibration->pot_mid + pot_diff;

  /*
  printf("bva %4d bd %4d pd %4d pv %4d ", brad_val_abs, brad_diff, pot_diff,
      pot_val);
  */
#endif

  return pot_val;
}

/*******************************************************************************
* FUNCTION NAME: apply_acceleration
* PURPOSE:       applies acceleration!
*******************************************************************************/
INT16 apply_acceleration(INT16 speed, INT16 speed_prev)
{

    if((speed - speed_prev) > ACCEL_RATE)
    {
        speed = speed_prev + ACCEL_RATE;
    }
    else if((speed - speed_prev) < -ACCEL_RATE)
    {
        speed = speed_prev - ACCEL_RATE;
    }

  return speed;
}

INT16 apply_acceleration2(INT16 speed, INT16 speed_prev)
{
    INT16 accel;
    if(toggle_tmp == 2)
    {
        accel = ACCEL_RATE_TRACTION;
    }
    else
    {
        accel = 0;
    }

    if(Oi_auto_color_select == 1)
    {
        if(speed_prev >= 0 && speed > speed_prev)
        {
            if(speed >= 25 && speed_prev < 25)
            {
                speed = 25;
            }
            else
            {
                speed = speed_prev + accel;
            }
        }
        else if(speed_prev > 0 && speed <= 0)
        {
            speed = 0;
        }
        else if(speed_prev <= 0 && speed < speed_prev)
        {
            if(speed <= -15 && speed_prev > -15)
            {
                speed = -15;
            }
            else
            {
                speed = speed_prev - accel;
            }
        }
        else if(speed_prev < 0 && speed >= 0)
        {
            speed = 0;
        }
        else
        {
            speed = speed;
        }
    }
    return speed;
}

/*******************************************************************************
* FUNCTION NAME: crab_pos_control
* PURPOSE:       runs feedback
*******************************************************************************/
void crab_pos_control()
{
#if CRAB_FEEDBACK_PRINTS
  printf("crab %d ", g_crab_tgts.state);
#endif

  if (g_crab_tgts.state == FEEDBACK_ENABLED)
  {
#if 1
#if CRAB_FEEDBACK_PRINTS
    printf(" B ");
#endif
    show_pid = 0;
    motor_vals.back_crab = crab_feedback(g_crab_tgts.back_pot_tgt,
                                         &pot_vals.back_crab,
                                         &calibration_vals.back_crab,
                                         &g_crab_back_pid_vals);
#endif
#if 1
#if CRAB_FEEDBACK_PRINTS
    printf(" F ");
#endif
    show_pid = 0;
    motor_vals.front_crab = crab_feedback(g_crab_tgts.front_pot_tgt,
                                          &pot_vals.front_crab,
                                          &calibration_vals.front_crab,
                                          &g_crab_front_pid_vals);
#endif

#if 1
    bad_pot_check(motor_vals.front_crab, pot_vals.front_crab.abs_val,
                  &front_pot_check_vars);
    bad_pot_check(motor_vals.back_crab, pot_vals.back_crab.abs_val,
                  &back_pot_check_vars);
#endif
  }
}

/*******************************************************************************
* FUNCTION NAME: bad_pot_check()
* PURPOSE: Check for bad pots :)
*******************************************************************************/
void bad_pot_check(INT8 crab_speed, UINT16 pot_curr,
                   PotCheckVarsType *check_vars)
{
  INT16 pot_delta;

  pot_delta = pot_curr - check_vars->pot_prev;
#if DEBUGGING_POT_CHECK
  printf(" PD: %5d  BLC1: %4d  ", pot_delta, check_vars->bad_loop_counter);
#endif

  if (crab_speed <= -POT_CHECK_MOTOR_FULL)
  {
    /* Motor ~full reverse */
#if DEBUGGING_POT_CHECK
    printf("REV  ");
#endif
    if ((pot_delta > POT_CHECK_REV_MIN_DELTA) &&
        (pot_delta < POT_CHECK_REV_MAX_DELTA))
    {
#if DEBUGGING_POT_CHECK
      printf("GOOD ");
#endif
      check_vars->bad_loop_counter -= POT_CHECK_GOOD_DECREMENT;
    }
    else
    {
#if DEBUGGING_POT_CHECK
      printf("BAD  ");
#endif
      check_vars->bad_loop_counter += POT_CHECK_BAD_INCREMENT;
    }
  }
  else if (crab_speed >= POT_CHECK_MOTOR_FULL)
  {
    /* Motor ~full forward */
#if DEBUGGING_POT_CHECK
    printf("FWD  ");
#endif
    if ((pot_delta > POT_CHECK_FWD_MIN_DELTA) && 
        (pot_delta < POT_CHECK_FWD_MAX_DELTA))
    {
#if DEBUGGING_POT_CHECK
      printf("GOOD ");
#endif
      check_vars->bad_loop_counter -= POT_CHECK_GOOD_DECREMENT;
    }
    else
    {
#if DEBUGGING_POT_CHECK
      printf("BAD  ");
#endif
      check_vars->bad_loop_counter += POT_CHECK_BAD_INCREMENT;
    }
  }
  else if ((crab_speed >= -POT_CHECK_MOTOR_STOP) &&
      (crab_speed <= POT_CHECK_MOTOR_STOP))
  {
    /* Motor ~stopped */
#if DEBUGGING_POT_CHECK
    printf("STOP ");
#endif
    if ((pot_delta > POT_CHECK_STOP_MIN_DELTA) && 
        (pot_delta < POT_CHECK_STOP_MAX_DELTA))
    {
#if DEBUGGING_POT_CHECK
      printf("GOOD ");
#endif
      check_vars->bad_loop_counter -= POT_CHECK_GOOD_DECREMENT;
    }
    else
    {
#if DEBUGGING_POT_CHECK
      printf("BAD  ");
#endif
      check_vars->bad_loop_counter += POT_CHECK_BAD_INCREMENT;
    }
  }
  else
  {
    /* Transition state; reset loop counter */
#if DEBUGGING_POT_CHECK
    printf("ELSE RST  ");
#endif
    check_vars->bad_loop_counter = 0;
  }

#if DEBUGGING_POT_CHECK
  printf("BLC2: %4d  ", check_vars->bad_loop_counter);
#endif

  /* Do not allow good decrements to accumulate */
  if (check_vars->bad_loop_counter < 0)
  {
    check_vars->bad_loop_counter = 0;
  }
  /* Do not allow bad increments to accumulate too much */
  if (check_vars->bad_loop_counter > POT_CHECK_BAD_LOOP_MAX)
  {
    check_vars->bad_loop_counter = POT_CHECK_BAD_LOOP_MAX;
  }

  if (check_vars->bad_loop_counter >= POT_CHECK_BAD_THRESHOLD)
  {
    g_crab_tgts.state = FEEDBACK_POT_BROKEN;
  }

  check_vars->crab_speed_prev = crab_speed;
  check_vars->pot_prev = pot_curr;
}

/*******************************************************************************
* FUNCTION NAME: crab_feedback
* PURPOSE:
*******************************************************************************/
INT8 crab_feedback(UINT16 desired_crab_pos_pot, PotDataType *cur_crab_pos,
                   CrabType *calibration, PidValsType *pid_vals)
{
  INT8 crab_speed = 0;


#if CRAB_FEEDBACK_PRINTS
  printf("des %3d curr %3d ", desired_crab_pos_pot, cur_crab_pos->abs_val);
#endif

  crab_speed = ws_pid(pid_vals, cur_crab_pos->abs_val, desired_crab_pos_pot);

#if CRAB_FEEDBACK_PRINTS
  printf("spd %03d", crab_speed);
#endif

  return crab_speed;
}

