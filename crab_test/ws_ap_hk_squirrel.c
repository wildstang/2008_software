#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"
#include "ws_cc.h"
#include "ws_drive_input.h"
#include "ws_arm.h"

#include "ws_autonomous.h"
#include "ws_autonomous_prog.h"

/* HK SQUIRREL 
 */
#define AP_HKSQR_DRIVE_TO_RACK_COUNT          (800)
#define AP_HKSQR_PAUSE_AT_RACK_COUNT          (60)
#define AP_HKSQR_LOWER_ARM_COUNT              (60)
#define AP_HKSQR_RELEASE_TUBE_COUNT           (20)
#define AP_HKSQR_DRIVE_HOME_RELEASING_COUNT   (-250)
#define AP_HKSQR_DRIVE_HOME_IN_STORAGE_COUNT  (-750)
#define AP_HKSQR_GRAB_HP_COUNT                (200)

UINT8 ap_hk_squirrel(void)
{
  static UINT8 prog_state = AP_HKSQR_STATE_DRIVE_TO_RACK;
  UINT8 prog_ret = AUTO_PROGRAM_DONE;
  static UINT16 loop_count = 0;
  static INT16 drive_count = 0;

  printf("HKSQ %d %d %d\r", prog_state, loop_count, drive_count);
  Oi_drive_y = 127;

  switch(prog_state)
  {
    case AP_HKSQR_STATE_DRIVE_TO_RACK:
      //feedback_tgt_pos.feedback_enabled = 1;
      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;

        if(drive_count >= AP_HKSQR_DRIVE_TO_RACK_COUNT)
        {
          feedback_tgt_pos.feedback_enabled = 0;
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_HKSQR_STATE_PAUSE_AT_RACK;
        }
        else if(loop_count >= 800)
        {
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_HKSQR_STATE_DONE;
        }
        else
        {
          Oi_drive_y = (127 + 40);
          Oi_sw_turbo = 1;
          SET_Oi_sw_arm_preset_score_low();
          loop_count++;
        }
        prog_ret = AUTO_PROGRAM_NOT_DONE;
      }
      else
      {
        prog_state = AP_HKSQR_STATE_DONE;
      }

      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_HKSQR_STATE_PAUSE_AT_RACK:
        feedback_tgt_pos.feedback_enabled = 0;
        if(loop_count >= AP_HKSQR_PAUSE_AT_RACK_COUNT)
        {
          loop_count = 0;
          prog_state = AP_HKSQR_STATE_LOWER_ARM;
        }
        arm_control(1);
        prog_ret = AUTO_PROGRAM_NOT_DONE;
        loop_count++;
      break;
    case AP_HKSQR_STATE_LOWER_ARM:
      /* Shoulder - 48
         Elbow - 128
         Rotate - Center */
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.shoulder = (calibration_vals.shoulder_horizontal >> 2) - 25;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) - 12;
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

      if(loop_count >= AP_HKSQR_LOWER_ARM_COUNT)
      {
        loop_count = 0;
        feedback_tgt_pos.feedback_enabled = 0;
        prog_state = AP_HKSQR_STATE_RELEASE_TUBE;
      }
      else
      {
        loop_count++;
      }
      
      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_HKSQR_STATE_RELEASE_TUBE:
      if(loop_count >= AP_HKSQR_RELEASE_TUBE_COUNT)
      {
        loop_count = 0;
        drive_count = 0;
        prog_state = AP_HKSQR_STATE_GO_HOME_RELEASING;
      }
      else
      {
        loop_count++;
      }
#if ANALOG_GRIPPER
      Oi_sw_gripper = ANALOG_GRIPPER_OPEN;
#else
      Oi_sw_gripper = 1;
#endif

      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_HKSQR_STATE_GO_HOME_RELEASING:
      feedback_tgt_pos.feedback_enabled = 0;
      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;

        /* Going backwards, so count will be negative */
        if(drive_count <= AP_HKSQR_DRIVE_HOME_RELEASING_COUNT)
        {
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_HKSQR_STATE_GO_HOME_IN_STORAGE;
        }
        else if(loop_count >= 800)
        {
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_HKSQR_STATE_DONE;
        }
        else
        {
          Oi_drive_y = (127 - 70);
          Oi_sw_turbo = 1;
#if ANALOG_GRIPPER
      Oi_sw_gripper = ANALOG_GRIPPER_OPEN;
#else
      Oi_sw_gripper = 1;
#endif
          loop_count++;
        }

        prog_ret = AUTO_PROGRAM_NOT_DONE;
      }
      else
      {
        prog_state = AP_HKSQR_STATE_DONE;
      }

      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_HKSQR_STATE_GO_HOME_IN_STORAGE:
      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;

        /* Going backwards, so count will be negative */
        if(drive_count <= AP_HKSQR_DRIVE_HOME_IN_STORAGE_COUNT)
        {
          feedback_tgt_pos.feedback_enabled = 0;
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_HKSQR_STATE_GRAB_HP;
        }
        else if(loop_count >= 800)
        {
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_HKSQR_STATE_DONE;
        }
        else
        {
          Oi_drive_y = (127 - 40);
          Oi_sw_turbo = 1;
          SET_Oi_sw_arm_preset_storage();
          loop_count++;
        }

        prog_ret = AUTO_PROGRAM_NOT_DONE;
      }
      else
      {
        prog_state = AP_HKSQR_STATE_DONE;
      }

      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_HKSQR_STATE_GRAB_HP:
      if(loop_count >= AP_HKSQR_GRAB_HP_COUNT)
      {
        loop_count = 0;
        feedback_tgt_pos.feedback_enabled = 0;
        prog_state = AP_HKSQR_STATE_DONE;
      }
      else
      {
        SET_Oi_sw_arm_preset_grab_hp();
        loop_count++;
      }
      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_HKSQR_STATE_DONE:
      prog_ret = AUTO_PROGRAM_DONE;
      printf("HKSQR DONE\r");
      break;
  }

  drive_stick_input(0);

  return prog_ret;
}
