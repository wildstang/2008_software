/*******************************************************************************
* FILE NAME: ws_io.c
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

extern UINT8 do_crab_arc;
/*
#define MOTOR_TEST
 */

/*******************************************************************************
* FUNCTION NAME: set_arm_motor_vals_off()
* PURPOSE:       Turns all motors in the arm off in motor_vals struct
* ARGUMENTS:     none
* RETURNS:       none
*
*******************************************************************************/
void set_arm_motor_vals_off(void)
{
  motor_vals.shoulder = 0;
  motor_vals.elbow = 0;
  motor_vals.rotate = 0;

}
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
  motor_vals.enable_crab_arc = 0;
  //motor_vals.pump = PUMP_OFF;
  motor_vals.landing_gear = LANDING_GEAR_UP;

  set_arm_motor_vals_off();

  motor_vals.gripper_top = GRIPPER_CLOSED;
  motor_vals.gripper_bottom = GRIPPER_CLOSED;
  motor_vals.telescope = TELESCOPE_IN;

  motor_vals.brake_mode = BRAKE_MODE_OFF;

  motor_vals.ramp_lock = RAMP_LOCKED;
  motor_vals.ramp_release = RAMP_NO_RELEASE;
  motor_vals.tower_release = TOWER_NO_RELEASE;
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

  if(do_crab_arc != CRAB_ARC_NONE)
  {
    Rc_analog_out_drive_lf = 127;
    Rc_analog_out_drive_rf =  127;
  }

  /*
  printf("%d %d %d %d\r", Rc_analog_out_drive_lf, Rc_analog_out_drive_lb,
                          Rc_analog_out_drive_rf, Rc_analog_out_drive_rb);
                          */

  /* crab motors */
  Rc_analog_out_back_crab = motor_vals.back_crab + 127;
  Rc_analog_out_front_crab = motor_vals.front_crab + 127;

  /* arm motors */
  Rc_analog_out_shoulder = motor_vals.shoulder + 127;
  Rc_analog_out_elbow = motor_vals.elbow + 127;
  Rc_analog_out_rotate = motor_vals.rotate + 127;

  /* ramp lock servo */
  if (motor_vals.ramp_lock == RAMP_UNLOCKED)
  {
    Rc_analog_out_ramp_lock = RAMP_UNLOCKED_PWM;
  }
  else
  {
    Rc_analog_out_ramp_lock = RAMP_LOCKED_PWM;
  }

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
  if(motor_vals.pump == PUMP_ON)
  {
    Rc_relay_pump_on = 1;
  }
  else
  {
    Rc_relay_pump_on = 0;
  }

  if(motor_vals.landing_gear == LANDING_GEAR_DOWN)
  {
    Rc_relay_landing_gear = 1;
  }
  else
  {
    Rc_relay_landing_gear = 0;
  }

  if(motor_vals.gripper_top == GRIPPER_OPEN)
  {
    Rc_relay_gripper_top = 1;
  }
  else
  {
    Rc_relay_gripper_top = 0;
  }

  if(motor_vals.gripper_bottom == GRIPPER_OPEN)
  {
    Rc_relay_gripper_bottom = 1;
  }
  else
  {
    Rc_relay_gripper_bottom = 0;
  }

  if(motor_vals.telescope == TELESCOPE_OUT)
  {
    Rc_relay_telescope = 1;
  }
  else
  {
    Rc_relay_telescope = 0;
  }

  if (motor_vals.ramp_release == RAMP_DEPLOY)
  {
    Rc_relay_ramp_release = 1;
  }
  else
  {
    Rc_relay_ramp_release = 0;
  }

  if (motor_vals.tower_release == TOWER_RELEASE)
  {
    Rc_relay_tower_release = 1;
  }
  else
  {
    Rc_relay_tower_release = 0;
  }


#endif

  /*
  printf("LAND - MV: %d OUT: %d\r", (int)motor_vals.landing_gear,
                                    (int) Rc_relay_landing_gear);
  */

  /*
  printf("MV S: %3d E: %3d R: %3d GT: %d GB: %d T: %d PC: %d\r",
          (int) motor_vals.shoulder, 
          (int) motor_vals.elbow, 
          (int) motor_vals.rotate,
          (int) motor_vals.gripper_top, 
          (int) motor_vals.gripper_bottom,
          (int) motor_vals.telescope);
  */

  /*
  printf("OUT S: %03d E: %03d R: %03d GT: %d GB: %d T: %d PC: %03d\r", 
          (int) Rc_analog_out_shoulder, 
          (int) Rc_analog_out_elbow, 
          (int) Rc_analog_out_rotate, 
          (int) Rc_relay_gripper_top, 
          (int) Rc_relay_gripper_bottom,
          (int) Rc_relay_telescope);
   */

  /****************************************************************************
   *
   * This section assigns digital outputs
   *
   ***************************************************************************/
  if(motor_vals.brake_mode == BRAKE_MODE_ON)
  {
    Dig_out_brake_mode = 0;
  }
  else
  {
    Dig_out_brake_mode = 1;
  }
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
  */


  /*
  printf("PRESET  0-%d 1-%d 2-%d 3-%d 4-%d 5-%d\r", 
          Oi_sw_arm_preset_0, Oi_sw_arm_preset_1,
          Oi_sw_arm_preset_2, Oi_sw_arm_preset_3,
          Oi_sw_arm_preset_4, Oi_sw_arm_preset_5);
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


