#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"
#include "ws_arm.h"

#include "ws_autonomous.h"
#include "ws_autonomous_prog.h"
//#include "ws_auto_drive.h"
#include "ws_general.h"

/* GRABBER */
UINT8 ap_grabber(void)
{
  static UINT8 prog_state = AP_GRBR_STATE_LOWER_ARM;
  UINT8 prog_ret = AUTO_PROGRAM_DONE;

  static UINT16 loop_count = 0;

  printf("GRABBER\r");
  switch(prog_state)
  {
    case AP_GRBR_STATE_LOWER_ARM:
      if(loop_count >= 100)
      {
        loop_count = 0;
        prog_state = AP_GRBR_STATE_RELEASE_TUBE;
      }
      else
      {
      /* Shoulder - 48
         Elbow - 144
         Rotate - Center */
        feedback_tgt_pos.feedback_enabled = 1;
        feedback_tgt_pos.shoulder = (calibration_vals.shoulder_horizontal >> 2) - 25;
        feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2);
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
        arm_control(1);
        loop_count++;
      }
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_GRBR_STATE_RELEASE_TUBE:
      if(loop_count >= 40)
      {
        loop_count = 0;
        prog_state = AP_GRBR_STATE_RAISE_ARM;
      }
      else
      {
      /* Shoulder - 48
         Elbow - 144
         Rotate - Center */
        feedback_tgt_pos.feedback_enabled = 1;
        feedback_tgt_pos.shoulder = (calibration_vals.shoulder_horizontal >> 2) - 25;
        feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2);
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
        arm_control(1);
        loop_count++;
      }
      prog_ret = AUTO_PROGRAM_NOT_DONE;

      break;
    case AP_GRBR_STATE_RAISE_ARM:
      /* Give the arm 4 seconds to raise */
      if(loop_count >= 160)
      {
        prog_state = AP_GRBR_STATE_DONE;
        /*
        printf("SAFE TIMER EXPIRED\r");
        */
      }
      else
      {
        SET_Oi_sw_arm_preset_grab_hp();
        arm_control(1);
        loop_count++;
      }
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_GRBR_STATE_DONE:
      prog_ret = AUTO_PROGRAM_DONE;
      printf("GRABBER DONE\r");
      break;
  }
  return prog_ret;
}

