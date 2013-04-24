/*******************************************************************************
* FILE NAME: ws_arm.c
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
#include "ws_arm.h"
#include "ws_general.h"
#include "ws_trig.h"

#define DEBUG_SHOULDER_CONSTRAINT  0
#define DEBUG_ELBOW_CONSTRAINT     0
#define DEBUG_ROTATE_CONSTRAINT    0

PidValsType shoulder_pid_vals;
PidValsType elbow_pid_vals;
PidValsType elbow_speed_pid_vals;
PidValsType rotate_pid_vals;
ArmPosType feedback_tgt_pos;

extern UINT16 g_cur_elbow;
UINT8 telescope_prev = TELESCOPE_IN;
#if ANALOG_GRIPPER
UINT8 gripper_prev = GRIPPER_CLOSED;
#endif

/*******************************************************************************
* FUNCTION NAME: arm_init
* PURPOSE:       initialize arm PID vals
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void arm_init()
{
  /* Initialize shoulder PID values */
  shoulder_pid_vals.scale_factor = 10;
  shoulder_pid_vals.prop_gain = 4 * shoulder_pid_vals.scale_factor;
  shoulder_pid_vals.int_gain = 1;
  shoulder_pid_vals.deriv_gain = -1 * shoulder_pid_vals.scale_factor;
  shoulder_pid_vals.integral = 0;
  shoulder_pid_vals.max_integral = 63 * shoulder_pid_vals.scale_factor;
  shoulder_pid_vals.min_val = -127;
  shoulder_pid_vals.max_val = 127;
  shoulder_pid_vals.int_reset_thresh = NO_INT_RESET_THRESH;
  shoulder_pid_vals.error_thresh = 3;
  pid_last_error_init(&shoulder_pid_vals);

  /* Initialize elbow PID values */
  elbow_pid_vals.scale_factor = 100;
  elbow_pid_vals.prop_gain = (10 * elbow_pid_vals.scale_factor) / 6;
  elbow_pid_vals.int_gain = 22;
  elbow_pid_vals.deriv_gain = (-11 * elbow_pid_vals.scale_factor) / 5;
  elbow_pid_vals.integral = 0;
  elbow_pid_vals.max_integral = 20 * elbow_pid_vals.scale_factor; elbow_pid_vals.min_val = -63;
  elbow_pid_vals.max_val = 63;
  elbow_pid_vals.int_reset_thresh = 2;
  elbow_pid_vals.error_thresh = 4;
  pid_last_error_init(&elbow_pid_vals);

  /* Initialize elbow speed PID values */
  elbow_speed_pid_vals.scale_factor = 100;
  elbow_speed_pid_vals.prop_gain = 50;
  elbow_speed_pid_vals.int_gain = 0;
  elbow_speed_pid_vals.deriv_gain = 10;
  //elbow_speed_pid_vals.int_gain = 7;
  //elbow_speed_pid_vals.deriv_gain = 60;
  elbow_speed_pid_vals.integral = 0;
  elbow_speed_pid_vals.max_integral = 127 * elbow_speed_pid_vals.scale_factor;
  elbow_speed_pid_vals.min_val = -127;
  elbow_speed_pid_vals.max_val = 127;
  elbow_speed_pid_vals.int_reset_thresh = NO_INT_RESET_THRESH;
  elbow_speed_pid_vals.error_thresh = NO_ERROR_THRESH;
  pid_last_error_init(&elbow_speed_pid_vals);

  /* Initialize rotation PID values */
#ifdef REAL_ROBOT
  rotate_pid_vals.scale_factor = 10;
  rotate_pid_vals.prop_gain = 4 * rotate_pid_vals.scale_factor;
  rotate_pid_vals.int_gain = 0;
  rotate_pid_vals.deriv_gain = -2 * rotate_pid_vals.scale_factor;
  rotate_pid_vals.integral = 0;
  rotate_pid_vals.max_integral = 63 * rotate_pid_vals.scale_factor;
  rotate_pid_vals.min_val = -90;
  rotate_pid_vals.max_val = 90;
  rotate_pid_vals.int_reset_thresh = 2;
  rotate_pid_vals.error_thresh = 2;
  pid_last_error_init(&rotate_pid_vals);
#else
  rotate_pid_vals.scale_factor = 10;
  rotate_pid_vals.prop_gain = 8 * rotate_pid_vals.scale_factor;
  rotate_pid_vals.int_gain = 0;
  rotate_pid_vals.deriv_gain = -2 * rotate_pid_vals.scale_factor;
  rotate_pid_vals.integral = 0;
  rotate_pid_vals.max_integral = 63 * rotate_pid_vals.scale_factor;
  rotate_pid_vals.min_val = -127;
  rotate_pid_vals.max_val = 127;
  rotate_pid_vals.int_reset_thresh = 2;
  rotate_pid_vals.error_thresh = 2;
  pid_last_error_init(&rotate_pid_vals);
#endif

  feedback_tgt_pos.feedback_enabled = 0;
}


/*******************************************************************************
* FUNCTION NAME: arm_control
* PURPOSE:       main point of entry for controlling the arm
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void arm_control(UINT8 force_feedback)
{
  static ArmStateType current_state = ARM_CTRL_STATE_MANUAL;
  ArmStateType new_state = ARM_CTRL_STATE_MANUAL;

  /* Arm control state machine
   *
   */
  /*
  printf("cs %d ", current_state);
  */
  switch(current_state)
  {
    case ARM_CTRL_STATE_MANUAL:
      /*
       *  State: ARM_CTRL_STATE_MANUAL (INITIAL STATE)
       *
       *         Input:       Preprog button press
       *         Action:      Call arm_control_feedback
       *         Next State:  ARM_CTRL_STATE_FEEDBACK
       *
       *         Input:       Manual override engaged
       *         Action:      Call arm_control_manual without constraints
       *         Next State:  ARM_CTRL_STATE_MANUAL_OVERRIDE
       *
       *         Input:       *
       *         Action:      Call arm_control_manual with constraints
       *         Next State:  ARM_CTRL_STATE_MANUAL
       *
       */

      if ((Oi_sw_arm_preset_0 == 1 && Oi_sw_arm_preset_0_prev == 0) ||
          (Oi_sw_arm_preset_1 == 1 && Oi_sw_arm_preset_1_prev == 0) ||
          (Oi_sw_arm_preset_2 == 1 && Oi_sw_arm_preset_2_prev == 0) ||
          (Oi_sw_arm_preset_3 == 1 && Oi_sw_arm_preset_3_prev == 0) ||
          (Oi_sw_arm_preset_4 == 1 && Oi_sw_arm_preset_4_prev == 0) ||
          (Oi_sw_arm_preset_5 == 1 && Oi_sw_arm_preset_5_prev == 0) ||
          (Oi_sw_arm_preset_6 == 1 && Oi_sw_arm_preset_6_prev == 0) ||
          (Oi_sw_arm_preset_7 == 1 && Oi_sw_arm_preset_7_prev == 0) ||
          (Oi_sw_arm_preset_8 == 1 && Oi_sw_arm_preset_8_prev == 0) ||
          (force_feedback == 1))
      {
        arm_control_feedback();
        new_state = ARM_CTRL_STATE_FEEDBACK;
      }
      else if (Oi_sw_arm_manu_override == 1)
      {
        arm_control_manual(0);
        new_state = ARM_CTRL_STATE_MANUAL_OVERRIDE;
      }
      else
      {
        arm_control_manual(1);
        new_state = ARM_CTRL_STATE_MANUAL;
      }
      break;

    case ARM_CTRL_STATE_MANUAL_OVERRIDE:
      /*
       *  State: ARM_CTRL_STATE_MANUAL_OVERRIDE
       *
       *         Input:       Manual override disengaged
       *         Action:      Enable constraints and call arm_control_manual
       *         Next State:  ARM_CTRL_STATE_MANUAL
       *
       *         Input:       *
       *         Action:      Disable constraints and call arm_control_manual
       *         Next State:  ARM_CTRL_STATE_MANUAL_OVERRIDE
       *
       */
      if (Oi_sw_arm_manu_override == 0)
      {
        arm_control_manual(1);
        new_state = ARM_CTRL_STATE_MANUAL;
      }
      else
      {
        arm_control_manual(0);
        new_state = ARM_CTRL_STATE_MANUAL_OVERRIDE;
      }
      break;

    case ARM_CTRL_STATE_FEEDBACK:
    /*
     *  State: ARM_CTRL_STATE_FEEDBACK
     *
     *         Input:       Manual movement
     *         Action:      Terminate current goal
     *         Next State:  ARM_CTRL_STATE_MANUAL
     *
     *         Input:       Manual override engaged
     *         Action:      Terminate current goal
     *         Next State:  ARM_CTRL_STATE_MANUAL_OVERRIDE
     *
     *         Input:       *
     *         Action:      Call arm_control_feedback
     *         Next State:  ARM_CTRL_STATE_FEEDBACK
     *
     */

      if(OUTSIDE_DEADZONE(Oi_arm_shoulder, SHOULDER_MANU_DEADZONE) ||
         OUTSIDE_DEADZONE(Oi_arm_elbow, ELBOW_MANU_DEADZONE) ||
         OUTSIDE_DEADZONE(Oi_arm_rotate, ROTATE_MANU_DEADZONE) ||
#if TWO_BUTTON_GRIPPER
         (Oi_sw_gripper_open == 1) ||
         (Oi_sw_gripper_close == 1) ||
#elif ANALOG_GRIPPER
         (Oi_sw_gripper >= ANALOG_GRIPPER_OPEN) ||
         (Oi_sw_gripper <= ANALOG_GRIPPER_CLOSED) ||
#else
         (Oi_sw_gripper == 1) ||
#endif
         (Oi_sw_arm_telescope == 1))
      {
        set_arm_motor_vals_off();
        arm_control_manual(1);
        new_state = ARM_CTRL_STATE_MANUAL;
      }
      else if (Oi_sw_arm_manu_override == 1)
      {
        set_arm_motor_vals_off();
        arm_control_manual(0);
        new_state = ARM_CTRL_STATE_MANUAL_OVERRIDE;
      }
      else
      {
        arm_control_feedback();
        new_state = ARM_CTRL_STATE_FEEDBACK;
      }

    break;


    default:
      /* Default to manual */
      new_state = ARM_CTRL_STATE_MANUAL;
      break;

  }

  /* If we're disabled, drop back to manual */
  if(disabled_mode == ROBOT_DISABLED)
  {
    new_state = ARM_CTRL_STATE_MANUAL;
  }

  /*
  printf("SwP: %d  SwC : %d  TP: %d TC: %d\r", Oi_sw_arm_telescope, Oi_sw_arm_telescope_prev,
                                           telescope_prev, motor_vals.telescope);
   */
  gripper_prev = motor_vals.gripper_top;
  telescope_prev = motor_vals.telescope;

  current_state = new_state;
}

/*******************************************************************************
* FUNCTION NAME: get_cur_arm_position
* PURPOSE:       Reads the RC inputs to determine the position of the arm
* ARGUMENTS:     none
* RETURNS:       ArmPosType position
*******************************************************************************/
ArmPosType get_cur_arm_position(void)
{
  ArmPosType position;

  /* Read the RC inputs and fill in the position structure */
  position.gripper_top = motor_vals.gripper_top;
  position.gripper_bottom = motor_vals.gripper_top;
  position.telescope = motor_vals.telescope;
  position.shoulder = GET_ANALOG_VALUE_SHIFT(Analog_in_shoulder);
#ifndef USING_SPEED_PID
  g_cur_elbow =  Get_Analog_Value(Analog_in_elbow);
#endif
  position.elbow = g_cur_elbow >> 2;
  position.rotate = GET_ANALOG_VALUE_SHIFT(Analog_in_rotate);

  /*
  printf("ARM: S: %03d E: %3d R %3d GT: %d  GB: %d  T: %d\r",
         (int) position.shoulder,
         (int) position.elbow,
         (int) position.rotate,
         (int) position.gripper_top,
         (int) position.gripper_bottom,
         (int) position.telescope);
  */
  return position;
}


/*******************************************************************************
* FUNCTION NAME: arm_control_manual
* PURPOSE:       Use the OI Inputs to move the arm manually.
* ARGUMENTS:     None
* RETURNS:       None
*******************************************************************************/
void arm_control_manual(UINT8 use_constraints)
{
  static int rotate_counter = 0;
  ArmPosType pos_cur;
  ArmConstraintType constraints;
  INT16 elbow_speed_tgt;
  INT32 elbow_speed_base;
  INT16 elbow_speed_cur;

  INT16  rotate_speed;
  static INT16 rotate_speed_prev = 0;

  /* Disallow arm feedback */
  feedback_tgt_pos.feedback_enabled = 0;

  /*
  printf("ARM CALIB: %d %d %d\r", (calibration_vals.shoulder_horizontal >> 2), 
      (calibration_vals.elbow_horizontal >> 2),
      (calibration_vals.rotate_horizontal >> 2));
      */

  /* Get the current position of the arm */
  pos_cur = get_cur_arm_position();

  /* Get the current constraints */
  constraints = calc_constraints(pos_cur, ARM_CTRL_STATE_MANUAL);

  /* Deadzone manipulator joystick */
  DEADZONE(Oi_arm_shoulder, SHOULDER_DEADZONE);
  DEADZONE(Oi_arm_elbow, ELBOW_DEADZONE);
  DEADZONE(Oi_arm_rotate, ROTATE_DEADZONE);

  /* Map OI Inputs to motor_vals outputs */
  motor_vals.shoulder = 127 - Oi_arm_shoulder;


#if 0
  elbow_speed_cur = g_elbow_speed;
  elbow_speed_tgt = ((INT16) 127 - (INT16) Oi_arm_elbow) * 2 / 3;

  if(Oi_arm_elbow >= 127)
  {
    //printf("+ ");
    //elbow_speed_base = ((elbow_speed_tgt * 96 ) / 100);
    /* .0002 * x^3 + .024 x^2 + 1.2134x */
    elbow_speed_base = (((INT32)2 * elbow_speed_tgt * elbow_speed_tgt * elbow_speed_tgt) / 10000);
    elbow_speed_base += (((INT32)24 * elbow_speed_tgt * elbow_speed_tgt) / 1000);
    elbow_speed_base += (((INT32)1213 * elbow_speed_tgt) / 1000);
  }
  else
  {
    //printf("- ");
    //elbow_speed_base = ((elbow_speed_tgt * 71 ) / 100);
    /* .0003 * x^3 - .0423 x^2 + 2.0651x */
    elbow_speed_base = (((INT32)3 * elbow_speed_tgt * elbow_speed_tgt * elbow_speed_tgt) / 10000);
    elbow_speed_base -= (((INT32)42 * elbow_speed_tgt * elbow_speed_tgt) / 1000);
    elbow_speed_base += (((INT32)2065 * elbow_speed_tgt) / 1000);
  }
  MIN_MAX(elbow_speed_base, -127, 127);


  /*
  show_pid = 0;
  */
  motor_vals.elbow = (INT16) elbow_speed_base + ws_pid(&elbow_speed_pid_vals, elbow_speed_cur, elbow_speed_tgt);
  if((elbow_speed_tgt > 0) && (motor_vals.elbow < 0))
  {
    motor_vals.elbow = 5;
  }
  else if((elbow_speed_tgt < 0) && (motor_vals.elbow > 0))
  {
    motor_vals.elbow = -5;
  }
  else if(elbow_speed_tgt == 0)
  {
    motor_vals.elbow = 0;
  }
  /*
  show_pid = 0;
  */
  //printf("T: %4d  S: %4d  MV: %3d  B: %3d\r", elbow_speed_tgt, g_elbow_speed, motor_vals.elbow, elbow_speed_base);
#else
  motor_vals.elbow = ((127 - (INT16) Oi_arm_elbow) * 2) / 3;
  /*
  printf("%d %d\r", (int) Oi_arm_elbow, (int)motor_vals.elbow);
  */

#endif

  /* ramp up speed of rotation based on how long button is pressed */
  if ((Oi_arm_rotate < OI_ROTATE_RIGHT) || (Oi_arm_rotate > OI_ROTATE_LEFT))
  {
#ifdef REAL_ROBOT
    rotate_speed = 75;
#else
    rotate_speed = 127;
#endif


    if (Oi_arm_rotate > OI_ROTATE_LEFT)
    {
      /* need to reverse motor */
      rotate_speed = -rotate_speed;
    }

    rotate_speed = acceleration_adjust(rotate_speed,
                                       rotate_speed_prev,
                                       ROTATE_ACCEL_RATE);
  }
  else
  {
    rotate_speed = 0;
    rotate_counter = 0;
  }

  //printf("speed %4d  prev %4d\r ", rotate_speed, rotate_speed_prev);
  motor_vals.rotate = rotate_speed;
  rotate_speed_prev = rotate_speed;

  if(disabled_mode == ROBOT_DISABLED)
  {
    motor_vals.telescope = TELESCOPE_IN;
    motor_vals.gripper_top = GRIPPER_CLOSED;
    motor_vals.gripper_bottom = GRIPPER_CLOSED;
  }
  else
  {
#if ANALOG_GRIPPER
    if((Oi_sw_gripper >= ANALOG_GRIPPER_OPEN) &&
       (gripper_prev == GRIPPER_CLOSED))
#else
    if((Oi_sw_gripper_open == 1) && (Oi_sw_gripper_open_prev == 0))
#endif
    {
      motor_vals.gripper_top = GRIPPER_OPEN;
      motor_vals.gripper_bottom = GRIPPER_OPEN;
    }

#if ANALOG_GRIPPER
    else if((Oi_sw_gripper <= ANALOG_GRIPPER_CLOSED) &&
            (gripper_prev == GRIPPER_OPEN))
#else
    if((Oi_sw_gripper_close == 1) && (Oi_sw_gripper_close_prev == 0))
#endif
    {
      motor_vals.gripper_top = GRIPPER_CLOSED;
      motor_vals.gripper_bottom = GRIPPER_CLOSED;
    }




    if((Oi_sw_arm_telescope == 1) && (Oi_sw_arm_telescope_prev == 0))
    {
      if (telescope_prev == TELESCOPE_IN)
      {
        motor_vals.telescope = TELESCOPE_OUT;
      }
      else if (telescope_prev == TELESCOPE_OUT)
      {
        motor_vals.telescope = TELESCOPE_IN;
      }
    }
  }


  /*
  printf(" scope %d \r", motor_vals.telescope);
  */

  /* If the manual override switch is not set, use the constraints and the
     desired movement to disable the outputs that aren't allowed */

  if(use_constraints == 1)
  {
    apply_constraints(constraints, ARM_CTRL_STATE_MANUAL);
  }

  /*
  printf("\r");
  */

}

/*******************************************************************************
* FUNCTION NAME: apply_constraints
* PURPOSE:       
* ARGUMENTS:     ArmConstraintType constraint
* RETURNS:       None
*******************************************************************************/
void apply_constraints(ArmConstraintType constraints, ArmStateType mode)
{
  //printf(" constraints ");
    APPLY_CONSTRAINTS(mode, constraints.shoulder, 
                          motor_vals.shoulder, shoulder_pid_vals)
#if DEBUG_SHOULDER_CONSTRAINT
    printf(" MV %d\r", motor_vals.shoulder);
#endif

    APPLY_CONSTRAINTS(mode, constraints.elbow, 
                          motor_vals.elbow, elbow_pid_vals)
    if(motor_vals.elbow == 0)
    {
      clear_pid_vals_history(&elbow_speed_pid_vals);
    }
#if DEBUG_ELBOW_CONSTRAINT
    printf(" MV %d\r", motor_vals.elbow);
#endif

    APPLY_CONSTRAINTS(mode, constraints.rotate, 
                          motor_vals.rotate, rotate_pid_vals)
#if DEBUG_ROTATE_CONSTRAINT
    printf(" MV %d\r", motor_vals.rotate);
#endif
}




/*******************************************************************************
* FUNCTION NAME: calc_arm_constraints
* PURPOSE:       Determine which arm joints are allowed to move based on the
*                current position
* ARGUMENTS:     ArmPosType arm_pos - Current arm position
* RETURNS:       ArmConstraintType constraint
*******************************************************************************/
ArmConstraintType calc_constraints(ArmPosType pos_cur, ArmStateType mode)
{
  ArmConstraintType constraints;

  constraints.elbow = CONSTRAINT_ALLOW_ALL;
  constraints.rotate = CONSTRAINT_ALLOW_ALL;
  constraints.shoulder = CONSTRAINT_ALLOW_ALL;

  //printf("T: %03d B: %03d Cur: %03d", (int)(calibration_vals.elbow_top >> 2), (int)(calibration_vals.elbow_bottom >> 2), (int)pos_cur.elbow);

  /* Elbow endpoints */
  if((pos_cur.elbow >= (calibration_vals.elbow_top) >> 2 ))
  {
#if DEBUG_ELBOW_CONSTRAINT
    printf("ET! - ");
#endif
    constraints.elbow = CONSTRAINT_ALLOW_FWD;
  }
  else if((pos_cur.elbow <= (calibration_vals.elbow_bottom >> 2)))
  {
#if DEBUG_ELBOW_CONSTRAINT
    printf("EB! - ");
#endif
    constraints.elbow = CONSTRAINT_ALLOW_REV;
  }
  else
  {
#if DEBUG_ELBOW_CONSTRAINT
    printf("E*  - ");
#endif
  }

#if DEBUG_ELBOW_CONSTRAINT
  printf("EC: %d ET: %d EB: %d ", pos_cur.elbow, 
                                  (calibration_vals.elbow_top) >> 2,
                                  (calibration_vals.elbow_bottom) >> 2);
#endif

  /* Shoulder endpoints */
  if((pos_cur.shoulder <= (calibration_vals.shoulder_front) >> 2 ))
  {
#if DEBUG_SHOULDER_CONSTRAINT
    printf("SF! - ");
#endif
    constraints.shoulder = CONSTRAINT_ALLOW_REV;
  }
  else if((pos_cur.shoulder >= (calibration_vals.shoulder_back >> 2)))
  {
#if DEBUG_SHOULDER_CONSTRAINT
    printf("SB! - ");
#endif
    constraints.shoulder = CONSTRAINT_ALLOW_FWD;
  }
  else
  {
#if DEBUG_SHOULDER_CONSTRAINT
    printf("S  - ");
#endif
  }
#if DEBUG_SHOULDER_CONSTRAINT
  printf("SC: %d SF: %d SB: %d ", pos_cur.shoulder, 
                                  (calibration_vals.shoulder_front) >> 2,
                                  (calibration_vals.shoulder_back) >> 2);
#endif

  /* Rotate endpoints */
  if((pos_cur.rotate >= (calibration_vals.rotate_right) >> 2 ))
  {
#if DEBUG_ROTATE_CONSTRAINT
    printf("RR! - ");
#endif
    constraints.rotate = CONSTRAINT_ALLOW_FWD;
  }
  else if((pos_cur.rotate <= (calibration_vals.rotate_left >> 2)))
  {
#if DEBUG_ROTATE_CONSTRAINT
    printf("RL! - ");
#endif
    constraints.rotate = CONSTRAINT_ALLOW_REV;
  }
  else
  {
#if DEBUG_ROTATE_CONSTRAINT
    printf("R* - ");
#endif
  }

#if DEBUG_ROTATE_CONSTRAINT
  printf("RC: %d RR: %d RL: %d ", pos_cur.rotate, 
                                  (calibration_vals.rotate_right) >> 2,
                                  (calibration_vals.rotate_left) >> 2);
#endif



#if 0
  /* Use the current position to fill in the constraints structure */
  if((pos_cur.shoulder > 140) && (pos_cur.elbow < 100))
  {
    constraints.elbow = CONSTRAINT_ALLOW_FWD;
  }
  else if((pos_cur.shoulder < 75) && (pos_cur.elbow > 100))
  {
    constraints.elbow = CONSTRAINT_ALLOW_REV;
  }


  if((pos_cur.shoulder > 100) && (pos_cur.shoulder < 130))
  {
    constraints.shoulder = CONSTRAINT_ALLOW_NONE;
  }

  if(pos_cur.shoulder > 150)
  {
    constraints.shoulder = CONSTRAINT_ALLOW_REV;
  }
  else if(pos_cur.shoulder < 50)
  {
    constraints.shoulder = CONSTRAINT_ALLOW_FWD;
  }
#endif

  return constraints;
}

/*******************************************************************************
* FUNCTION NAME: arm_control_feedback
* PURPOSE:       Use the OI Inputs and globals to determine the arm position
*                goal and move it there.
* ARGUMENTS:     None
* RETURNS:       None
*******************************************************************************/
void arm_control_feedback(void)
{
  ArmPosType pos_cur;
  ArmConstraintType constraints;

  /*
  printf(" %d %d %d\r",
        (calibration_vals.shoulder_horizontal >> 2),
        (calibration_vals.elbow_horizontal >> 2),
        (calibration_vals.rotate_horizontal >> 2));
        */

  /* Get the current position of the arm */
  pos_cur = get_cur_arm_position();


  /* Set the global target position based on the current goal */
  set_tgt_arm_position();

  /* Get the constraints based on the current and target positions
     Since we're passing in FEEDBACK, the target will be adjusted
     if necessary to comply with the constraints */
  constraints = calc_constraints(pos_cur, ARM_CTRL_STATE_FEEDBACK);

  /* Process control of arm components that are allowed by the constraints
   * For those that aren't allowed, clear the PID Vals history */

  if(feedback_tgt_pos.feedback_enabled == 1 && (disabled_mode != ROBOT_DISABLED))
  {
/*
    printf("ARM MASK %d %d %d %d %d %d %d %d\r",
            (feedback_tgt_pos.mask & ARM_MASK_SHOULDER) >> ARM_MASK_SHOULDER_SHIFT,
            (feedback_tgt_pos.mask & ARM_MASK_ELBOW) >> ARM_MASK_ELBOW_SHIFT,
            (feedback_tgt_pos.mask & ARM_MASK_ROTATE) >> ARM_MASK_ROTATE_SHIFT,
            (feedback_tgt_pos.mask & ARM_MASK_GRIPPER_TOP) >> ARM_MASK_GRIPPER_TOP_SHIFT,
            (feedback_tgt_pos.mask & ARM_MASK_GRIPPER_BOTTOM) >> ARM_MASK_GRIPPER_BOTTOM_SHIFT,
            (feedback_tgt_pos.mask & ARM_MASK_TELESCOPE) >> ARM_MASK_TELESCOPE_SHIFT,
            (feedback_tgt_pos.mask & ARM_MASK_NONE) >> ARM_MASK_NONE_SHIFT,
            (feedback_tgt_pos.mask & ARM_MASK_ALL) >> ARM_MASK_ALL_SHIFT
        );
*/
    if(feedback_tgt_pos.mask & ARM_MASK_SHOULDER)
    {
#if 1
    /*
      show_pid = 1;
      */
      motor_vals.shoulder = ws_pid(&shoulder_pid_vals, pos_cur.shoulder, feedback_tgt_pos.shoulder);
      show_pid = 0;
#else
      motor_vals.shoulder = 0;
#endif
    }
    else
    {
      motor_vals.shoulder = 0;
    }

    if(feedback_tgt_pos.mask & ARM_MASK_ELBOW)
    {
#if 1
    /*
      show_pid = 1;
      */
      motor_vals.elbow = ws_pid(&elbow_pid_vals, pos_cur.elbow, feedback_tgt_pos.elbow);
    /*
      show_pid = 0;
    */
#else
      motor_vals.elbow = 0;
#endif
    }
    else
    {
    motor_vals.elbow = 0;
    }

    if(feedback_tgt_pos.mask & ARM_MASK_ROTATE)
    {
#if 1
     /*
      show_pid = 1;
      */

      motor_vals.rotate = ws_pid(&rotate_pid_vals, pos_cur.rotate, feedback_tgt_pos.rotate);
     /*
      show_pid = 0;
      */
#else
      motor_vals.rotate = 0;
#endif
    }
    else
    {
      motor_vals.rotate = 0;
    }

    if(feedback_tgt_pos.mask & ARM_MASK_GRIPPER_TOP)
    {
      motor_vals.gripper_top = feedback_tgt_pos.gripper_top;
    }

    if(feedback_tgt_pos.mask & ARM_MASK_GRIPPER_BOTTOM)
    {
      motor_vals.gripper_bottom = feedback_tgt_pos.gripper_bottom;
    }

    if(feedback_tgt_pos.mask & ARM_MASK_TELESCOPE)
    {
      motor_vals.telescope = feedback_tgt_pos.telescope;
    }

    apply_constraints(constraints, ARM_CTRL_STATE_FEEDBACK);
  }
  else
  {
    set_arm_motor_vals_off();
  }

  /*
  printf("S %03d E %03d Ts %03d Te %03d MVs %03d MVe %03d", 
         (int)pos_cur.shoulder,
         (int)pos_cur.elbow,
         (int)feedback_tgt_pos.shoulder,
         (int)feedback_tgt_pos.elbow,
         (int)motor_vals.shoulder,
         (int)motor_vals.elbow);


  printf("\r");
  */



}

/*******************************************************************************
* FUNCTION NAME: set_tgt_arm_position
* PURPOSE:       Reads the RC inputs to determine the target position of the arm
* ARGUMENTS:     none
* RETURNS:       ArmPosType position
*******************************************************************************/
void set_tgt_arm_position(void)
{
  /* The latch will be set any time a pre-program switch is pressed
     It will be cleared when none of the buttons are pressed.  This
     debounces the case where the position is a Shift-Button and the
     Shift is released first.
  */
  static UINT8 arm_sw_latch = 0;

  /* Read the RC inputs and fill in the target position structure */
/*

    Proto:
    Calib Shoulder - 66
    Calib Elbow - 139
    Calib Rotate - 86

   */

  if(Oi_sw_arm_preset_grab_floor == 1)
  {
/*
    Shoulder - 28
    Elbow - 160
    Rotate - Center
   */
    if(arm_sw_latch == 0)
    {
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.shoulder =
        (calibration_vals.shoulder_horizontal >> 2) - 36;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) + 19;
      feedback_tgt_pos.rotate = (calibration_vals.rotate_horizontal >> 2);
      feedback_tgt_pos.telescope = TELESCOPE_OUT;
      feedback_tgt_pos.gripper_top = GRIPPER_OPEN;
      feedback_tgt_pos.gripper_bottom = GRIPPER_OPEN;




      feedback_tgt_pos.mask = ARM_MASK_SHOULDER |
                              ARM_MASK_ELBOW |
                              ARM_MASK_TELESCOPE |
                              ARM_MASK_GRIPPER_TOP |
                              ARM_MASK_GRIPPER_BOTTOM |
                              ARM_MASK_ROTATE;
    }

    arm_sw_latch = 1;

    /*
    printf("GRAB_FLOOR - ");
    */
  }
  else if(Oi_sw_arm_preset_storage == 1)
  {
/*
    Shoulder - 39
    Elbow - 234
    Rotate - Center
   */
    if(arm_sw_latch == 0)
    {
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.shoulder = (calibration_vals.shoulder_horizontal >> 2) - 25;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) + 93;
      feedback_tgt_pos.rotate = (calibration_vals.rotate_horizontal >> 2);
      feedback_tgt_pos.telescope = TELESCOPE_IN;
      feedback_tgt_pos.gripper_top = GRIPPER_CLOSED;
      feedback_tgt_pos.gripper_bottom = GRIPPER_CLOSED;

      feedback_tgt_pos.mask = ARM_MASK_SHOULDER |
                              ARM_MASK_ELBOW |
                              ARM_MASK_TELESCOPE |
                              ARM_MASK_GRIPPER_TOP |
                              ARM_MASK_GRIPPER_BOTTOM |
                              ARM_MASK_ROTATE;
    }

    arm_sw_latch = 1;

    /*
    printf("STORAGE - ");
    */
  }
  else if(Oi_sw_arm_preset_grab_hp == 1)
  {
/*
    Shoulder - 105
    Elbow - 196
    Rotate - Center
   */
    if(arm_sw_latch == 0)
    {
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.shoulder =
        (calibration_vals.shoulder_horizontal >> 2) + 41;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) + 55;
      feedback_tgt_pos.rotate = (calibration_vals.rotate_horizontal >> 2);
      feedback_tgt_pos.telescope = TELESCOPE_IN;
      feedback_tgt_pos.gripper_top = GRIPPER_OPEN;
      feedback_tgt_pos.gripper_bottom = GRIPPER_OPEN;

      feedback_tgt_pos.mask = ARM_MASK_SHOULDER |
                              ARM_MASK_ELBOW |
                              ARM_MASK_TELESCOPE |
                              ARM_MASK_GRIPPER_TOP |
                              ARM_MASK_GRIPPER_BOTTOM |
                              ARM_MASK_ROTATE;
    }

    arm_sw_latch = 1;

    /*
    printf("HP AUTO- ");
    */
  }
  else if(Oi_sw_arm_preset_score_low == 1)
  {
/*
    Shoulder - 39
    Elbow - 196
    Rotate - Center
   */
    if(arm_sw_latch == 0)
    {
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.shoulder = (calibration_vals.shoulder_horizontal >> 2) - 25;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) + 55;
      feedback_tgt_pos.rotate = (calibration_vals.rotate_horizontal >> 2);
      feedback_tgt_pos.telescope = TELESCOPE_IN;
      feedback_tgt_pos.gripper_top = GRIPPER_CLOSED;
      feedback_tgt_pos.gripper_bottom = GRIPPER_CLOSED;

      feedback_tgt_pos.mask = ARM_MASK_SHOULDER |
                              ARM_MASK_ELBOW |
                              ARM_MASK_TELESCOPE |
                              ARM_MASK_GRIPPER_TOP |
                              ARM_MASK_GRIPPER_BOTTOM |
                              ARM_MASK_ROTATE;
    }

    arm_sw_latch = 1;

    /*
    printf("SCORE LOW - ");
    */
  }
  else if(Oi_sw_arm_preset_score_mid == 1)
  {
/*
    Shoulder - 72
    Elbow - 172
    Rotate - Center
   */
    if(arm_sw_latch == 0)
    {
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.shoulder =
        (calibration_vals.shoulder_horizontal >> 2) + 6;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) + 31;
      feedback_tgt_pos.rotate = (calibration_vals.rotate_horizontal >> 2);
      feedback_tgt_pos.telescope = TELESCOPE_IN;
      feedback_tgt_pos.gripper_top = GRIPPER_CLOSED;
      feedback_tgt_pos.gripper_bottom = GRIPPER_CLOSED;

      feedback_tgt_pos.mask = ARM_MASK_SHOULDER |
                              ARM_MASK_ELBOW |
                              ARM_MASK_TELESCOPE |
                              ARM_MASK_GRIPPER_TOP |
                              ARM_MASK_GRIPPER_BOTTOM |
                              ARM_MASK_ROTATE;
    }

    arm_sw_latch = 1;

    /*
    printf("SCORE_LOW - ");
    */
  }
  else if(Oi_sw_arm_preset_score_high == 1)
  {
/*
    Shoulder - 111
    Elbow - 134
    Rotate - Center
   */
    if(arm_sw_latch == 0)
    {
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.shoulder =
        (calibration_vals.shoulder_horizontal >> 2) + 47;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) + 7;
      feedback_tgt_pos.rotate = (calibration_vals.rotate_horizontal >> 2);
      feedback_tgt_pos.telescope = TELESCOPE_OUT;
      feedback_tgt_pos.gripper_top = GRIPPER_CLOSED;
      feedback_tgt_pos.gripper_bottom = GRIPPER_CLOSED;

      feedback_tgt_pos.mask = ARM_MASK_SHOULDER |
                              ARM_MASK_ELBOW |
                              ARM_MASK_TELESCOPE |
                              ARM_MASK_GRIPPER_TOP |
                              ARM_MASK_GRIPPER_BOTTOM |
                              ARM_MASK_ROTATE;
    }

    arm_sw_latch = 1;

    /*
    printf("SCORE_LOW - ");
    */
  }
  else if(Oi_sw_arm_preset_driveby_left == 1)
  {
/*
    Shoulder - 67
    Elbow - 175
    Rotate - 68
   */
    if(arm_sw_latch == 0)
    {
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.shoulder =
        (calibration_vals.shoulder_horizontal >> 2) + 3;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) + 36;
      feedback_tgt_pos.rotate = (calibration_vals.rotate_horizontal >> 2) - 18;
      feedback_tgt_pos.telescope = TELESCOPE_IN;
      feedback_tgt_pos.gripper_top = GRIPPER_CLOSED;
      feedback_tgt_pos.gripper_bottom = GRIPPER_CLOSED;

      feedback_tgt_pos.mask = ARM_MASK_SHOULDER |
                              ARM_MASK_ELBOW |
                              ARM_MASK_TELESCOPE |
                              ARM_MASK_GRIPPER_TOP |
                              ARM_MASK_GRIPPER_BOTTOM |
                              ARM_MASK_ROTATE;
    }

    arm_sw_latch = 1;

    /*
    printf("DBY_LEFT - ");
    */
  }
  else if(Oi_sw_arm_preset_rotate_center == 1)
  {
/*
    Rotate - Center
   */
    if(arm_sw_latch == 0)
    {
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.rotate = (calibration_vals.rotate_horizontal >> 2);
      feedback_tgt_pos.mask = ARM_MASK_ROTATE;
    }

    arm_sw_latch = 1;

    /*
    printf("ROT_CENTER - ");
    */
  }
  else if(Oi_sw_arm_preset_driveby_right == 1)
  {
/*
    Shoulder - 67
    Elbow - 175
    Rotate - 112
   */
    if(arm_sw_latch == 0)
    {
      feedback_tgt_pos.feedback_enabled = 1;

      feedback_tgt_pos.shoulder =
        (calibration_vals.shoulder_horizontal >> 2) + 3;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) + 36;
      feedback_tgt_pos.rotate = (calibration_vals.rotate_horizontal >> 2) + 26;

      feedback_tgt_pos.telescope = TELESCOPE_IN;
      feedback_tgt_pos.gripper_top = GRIPPER_CLOSED;
      feedback_tgt_pos.gripper_bottom = GRIPPER_CLOSED;

      feedback_tgt_pos.mask = ARM_MASK_SHOULDER |
                              ARM_MASK_ELBOW |
                              ARM_MASK_TELESCOPE |
                              ARM_MASK_GRIPPER_TOP |
                              ARM_MASK_GRIPPER_BOTTOM |
                              ARM_MASK_ROTATE;
    }

    arm_sw_latch = 1;

    /*
    printf("DBY RIGHT - ");
    */
  }
  else
  {
    arm_sw_latch = 0;
    /*
    printf("* - ");
    */
  }



  /*
 printf("tgt s %d  e %d  r %d  GT %d\r", (int) feedback_tgt_pos.shoulder,
                                         (int) feedback_tgt_pos.elbow,
                                         (int) feedback_tgt_pos.rotate,
                                         (int) feedback_tgt_pos.gripper_top,
                                         (int) feedback_tgt_pos.gripper_bottom,
                                         (int) feedback_tgt_pos.telescope);
     */
}
