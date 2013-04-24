/*******************************************************************************
* FILE NAME: ws_io.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_frc.h"
#include "ifi_code.h"
#include "pwm.h"
#include "ws_includes.h"
#include "ws_io.h"

/*
#define MOTOR_TEST
 */

/*******************************************************************************
* FUNCTION NAME: set_motor_vals_off()
* PURPOSE:       Turns all motors off in motor_vals struct
* ARGUMENTS:     none
* RETURNS:       none
*
*******************************************************************************/
void set_motor_vals_off(void)
{
  motor_vals.left_drive = 0;
  motor_vals.right_drive = 0;
  motor_vals.front_crab = 0;
  motor_vals.back_crab = 0;
  motor_vals.roller = 0;
  motor_vals.accum_tilt = ACCUM_TILT_IN;
  motor_vals.nest_tilt = NEST_TILT_IN;
  motor_vals.slapper = SLAPPER_DOWN;
  motor_vals.landing_gear = LANDING_GEAR_UP;
  motor_vals.lift_1 = LIFT_NONE;
  motor_vals.lift_2 = LIFT_NONE;
  motor_vals.nest_level = NEST_LEVEL_NONE;
  motor_vals.ladder = LADDER_NONE;
  motor_vals.pump = PUMP_OFF;
  motor_vals.brake_mode = BRAKE_MODE_OFF;
}


/*******************************************************************************
* FUNCTION NAME: assign_outputs_slow
* PURPOSE:       assign motor speeds to pwm outputs
* CALLED FROM:
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void assign_outputs_slow()
{
  /****************************************************************************
   *
   * This section assigns PWM outputs
   *
   ***************************************************************************/
  pwm01 = 127;
  pwm02 = 127;
  pwm03 = 127;
  pwm04 = 127;
  pwm05 = 127;
  pwm06 = 127;
  pwm07 = 127;
  pwm08 = 127;
  pwm09 = 127;
  pwm10 = 127;
  pwm11 = 127;
  pwm12 = 127;

#ifdef MOTOR_TEST
  if ((Oi_drive_y > (127 - SC_CALIB_STICK_DEADZONE)) &&
      (Oi_drive_y < (127 + SC_CALIB_STICK_DEADZONE)))
  {
    Oi_drive_y = 127;
  }

  /*
  Rc_analog_out_drive_lf = Oi_drive_y;
  Rc_analog_out_drive_lb = Oi_drive_y;
  Rc_analog_out_drive_rf = Oi_drive_y;
  Rc_analog_out_drive_rb = Oi_drive_y;
  */

  printf(" driveY=%d\r", (int)Oi_drive_y);

#else
  /*
  printf("left: %d right: %d\r", (int)motor_vals.left_drive,
                                 (int)motor_vals.right_drive);
  */
  /* drive motors */
  Rc_analog_out_drive_lf = motor_vals.left_drive + 127;
  Rc_analog_out_drive_lb = motor_vals.left_drive + 127;
  Rc_analog_out_drive_rf = motor_vals.right_drive + 127;
  Rc_analog_out_drive_rb = motor_vals.right_drive + 127;

  /*
  printf("%d %d %d %d ", Rc_analog_out_drive_lf, Rc_analog_out_drive_lb,
                          Rc_analog_out_drive_rf, Rc_analog_out_drive_rb);
  */

  Rc_analog_out_roller = motor_vals.roller + 127;
#endif

  /****************************************************************************
   *
   * This section assigns relay outputs
   *
   ***************************************************************************/
  relay1_fwd = 0;
  relay1_rev = 0;
  relay2_fwd = 0;
  relay2_rev = 0;
  relay3_fwd = 0;
  relay3_rev = 0;
  relay4_fwd = 0;
  relay4_rev = 0;
  relay5_fwd = 0;
  relay5_rev = 0;
  relay6_fwd = 0;
  relay6_rev = 0;
  relay7_fwd = 0;
  relay7_rev = 0;
  relay8_fwd = 0;
  relay8_rev = 0;

#ifdef MOTOR_TEST

#else

  Rc_relay_accum_tilt = motor_vals.accum_tilt;
  Rc_relay_nest_tilt = motor_vals.nest_tilt;
  Rc_relay_slapper = motor_vals.slapper;
  Rc_relay_landing_gear = motor_vals.landing_gear;
  Rc_relay_pump = motor_vals.pump;

  /* Control the two way solenoids */
  SOLENOID_CONTROL_2WAY(motor_vals.lift_1, Rc_relay_lift_1_down, Rc_relay_lift_1_up,
                        LIFT_NONE, LIFT_DOWN, LIFT_UP);
  SOLENOID_CONTROL_2WAY(motor_vals.lift_2, Rc_relay_lift_2_down, Rc_relay_lift_2_up,
                        LIFT_NONE, LIFT_DOWN, LIFT_UP);
  SOLENOID_CONTROL_2WAY(motor_vals.nest_level, Rc_relay_nest_level_up, Rc_relay_nest_level_down,
                        NEST_LEVEL_NONE, NEST_LEVEL_DOWN,  NEST_LEVEL_UP);
  SOLENOID_CONTROL_2WAY(motor_vals.ladder, Rc_relay_ladder_down, Rc_relay_ladder_up,
                        LADDER_NONE, LADDER_DOWN,  LADDER_UP);
#endif


  /****************************************************************************
   *
   * This section assigns digital outputs
   *
   ***************************************************************************/
  Dig_out_brake_mode = motor_vals.brake_mode;

  return;
}


/*******************************************************************************
* FUNCTION NAME: assign_outputs_fast
* PURPOSE:       assign motor speeds to pwm outputs
* CALLED FROM:
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void assign_outputs_fast()
{
  pwm13 = 127;
  pwm14 = 127;
  pwm15 = 127;
  pwm16 = 127;

#ifdef MOTOR_TEST
  if ((Oi_drive_y > (127 - SC_CALIB_STICK_DEADZONE)) &&
      (Oi_drive_y < (127 + SC_CALIB_STICK_DEADZONE)))
  {
    Oi_drive_y = 127;
  }

  /*
  Rc_analog_out_pwm13 = Oi_drive_y;
  Rc_analog_out_pwm14 = Oi_drive_y;
  Rc_analog_out_pwm15 = Oi_drive_y;
  Rc_analog_out_pwm16 = Oi_drive_y;

  printf(" driveY %d\r", (int)Oi_drive_y);
  */

#else
  /* crab motors */
  Rc_analog_out_back_crab = motor_vals.back_crab + 127;
  Rc_analog_out_front_crab = motor_vals.front_crab + 127;
  //printf("Back %d Front %d \r", Rc_analog_out_front_crab, Rc_analog_out_back_crab);


  PWM(pwm13, pwm14, pwm15, pwm16);  // generate the precision PWM pusles
#endif

  return;
}



/*******************************************************************************
* FUNCTION NAME: joystick_scaling
* PURPOSE:       Scale joystick so the x & y ranges are 0-254
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
UINT8 joystick_scaling(UINT8 joystick_val, UINT8 stick_min_val,
                       UINT8 stick_mid_val, UINT8 stick_max_val)
{
  int tmp_val;
  INT16 new_joystick_val = 127;

  /************************************************************
  Since no two joysticks are the same, we want to make sure that
  all joysticks give a consistent 0-254 input to the rest of
  the code.

  This is done by scaling the joystick so that the values map to
  the following range
        Input          Output
     stick_min_val ==>   0
     stick_mid_val ==>  127
     stick_max_val ==>  254

  The scaled joystick value is stored back into joystick_val

  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  It is important to make sure that the return value is never
  less than 0 or greater than 254
  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  ************************************************************/

  /*
  printf("in %d ", (int)joystick_val);
  */

  MIN_MAX(joystick_val, stick_min_val, stick_max_val);

  /* Scale the joystick */
  if (joystick_val < stick_mid_val)
  {
    new_joystick_val = ((((INT16)joystick_val - (INT16)stick_min_val) * 127) /
                        ((INT16)stick_mid_val - (INT16)stick_min_val));
  }
  else if (joystick_val > stick_mid_val)
  {
    new_joystick_val = ((((INT16)joystick_val - (INT16)stick_mid_val) * 127) /
                        ((INT16)stick_max_val - (INT16)stick_mid_val)) + 127;
  }
  else
  {
    new_joystick_val = 127;
  }

  /* Ensure return is not less than 0 or greater than 254 */
  MIN_MAX(new_joystick_val, 0, 254);

  /*
  printf("out %d\r", (int)new_joystick_val);
  */

  return new_joystick_val;
}



void io_print_oi_inputs()
{
  /*
  printf(" Drive x %d y %d ", (int)Oi_drive_x, (int)Oi_drive_y);
  printf("\r");
  */

  /*
  printf("p1 x %3d y %3d wheel %3d aux %3d top %d trig %d aux1 %d aux2 %d\r",
         (int)p1_x, (int)p1_y, (int)p1_wheel, (int)p1_aux,
         (int)p1_sw_top, (int)p1_sw_trig, (int)p1_sw_aux1, (int)p1_sw_aux2);
  printf("p2 x %3d y %3d wheel %3d aux %3d top %d trig %d aux1 %d aux2 %d\r",
         (int)p2_x, (int)p2_y, (int)p2_wheel, (int)p2_aux,
         (int)p2_sw_top, (int)p2_sw_trig, (int)p2_sw_aux1, (int)p2_sw_aux2);
  printf("p3 x %3d y %3d wheel %3d aux %3d top %d trig %d aux1 %d aux2 %d\r",
         (int)p3_x, (int)p3_y, (int)p3_wheel, (int)p3_aux,
         (int)p3_sw_top, (int)p3_sw_trig, (int)p3_sw_aux1, (int)p3_sw_aux2);
  printf("p4 x %3d y %3d wheel %3d aux %3d top %d trig %d aux1 %d aux2 %d\r",
         (int)p4_x, (int)p4_y, (int)p4_wheel, (int)p4_aux,
         (int)p4_sw_top, (int)p4_sw_trig, (int)p4_sw_aux1, (int)p4_sw_aux2);

  printf("X %d S %d T %d C %d XL %d SL %d TL %d CL %d XR %d SR %d TR %d CR %d ",
          Oi_ps2_x_no_shift, Oi_ps2_square_no_shift, Oi_ps2_triangle_no_shift, 
          Oi_ps2_circle_no_shift, Oi_ps2_x_shift_l, Oi_ps2_square_shift_l, 
          Oi_ps2_triangle_shift_l, Oi_ps2_circle_shift_l, Oi_ps2_x_shift_r, 
          Oi_ps2_square_shift_r, Oi_ps2_triangle_shift_r, Oi_ps2_circle_shift_r);

  printf("IS %d ISL %d ISR %d ", Oi_ps2_is_shifted, Oi_ps2_is_shifted_l, Oi_ps2_is_shifted_r);
  */

  return;
}


void io_print_rc_inputs()
{
  /*
  printf("01 %d 02 %d 03 %d 04 %d 05 %d 06 %d 07 %d 08 %d 09 %d\r",
          rc_dig_in01, rc_dig_in02, rc_dig_in03, rc_dig_in04,
          rc_dig_in05, rc_dig_in06, rc_dig_in07, rc_dig_in08,
          rc_dig_in09);
  printf("10 %d 11 %d 12 %d 13 %d 14 %d 15 %d 16 %d 17 %d 18 %d\r",
          rc_dig_in10, rc_dig_in11, rc_dig_in12,
          rc_dig_in13, rc_dig_in14, rc_dig_in15, rc_dig_in16,
          rc_dig_in17, rc_dig_in18);
  printf("\r");
  */
  return;
}


/*******************************************************************************
* FUNCTION NAME: display_oi_data
* PURPOSE:       Displays data on OI user display when in user mode
* ARGUMENTS:     print_data - data to print
*                type
* RETURNS:       none
*******************************************************************************/
void display_oi_data(UINT8 print_data, DisplayDataType type)
{

  User_Mode_byte = print_data;

  return;
}


