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


/*
   R-L Sweeper
 */

#define AP_SWPRR_DRIVE_SPEED               (40)
#define AP_SWPRR_PAUSE_TURN_CRAB_90_COUNT  (25)
#define AP_SWPRR_PAUSE_AFTER_RIGHT_COUNT   (40)
#define AP_SWPRR_PAUSE_AFTER_LEFT_COUNT    (20)
#define AP_SWPRR_RAISE_ARM_COUNT           (160)

#define AP_SWPRR_DRIVE_DISTANCE_RIGHT      (-600)
#define AP_SWPRR_DRIVE_DISTANCE_LEFT       (250)

UINT8 ap_sweeper_right(void)
{
  static UINT8 prog_state = AP_SWPRR_STATE_TURN_CRAB_90_LEFT;
  UINT8 prog_ret = AUTO_PROGRAM_DONE;
  static UINT16 loop_count = 0;
  static INT16 drive_count = 0;

  printf("SWEEPER RIGHT %d %d %d\r", prog_state, loop_count, drive_count);
  Oi_drive_y = 127;
  feedback_tgt_pos.feedback_enabled = 0;

  switch(prog_state)
  {
    case AP_SWPRR_STATE_TURN_CRAB_90_LEFT:
      printf("STATE 1 %d ", loop_count);
      Oi_crab_x  = 254;
      Oi_drive_x  = 127;
      Oi_drive_y  = 127;

      if(loop_count >= AP_SWPRR_PAUSE_TURN_CRAB_90_COUNT)
      {
        loop_count = 0;
        prog_state = AP_SWPRR_STATE_DRIVE_RIGHT;
      }
      else
      {
        loop_count++;
      }
      printf("\r");
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_SWPRR_STATE_DRIVE_RIGHT:
      printf("STATE 2 %d %d\r", loop_count, drive_count);
      Oi_crab_x  = 254;
      Oi_drive_x  = 127;
      Oi_drive_y  = (127 - AP_SWPRR_DRIVE_SPEED);
      Oi_sw_turbo = 1;

      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;

        /* Drive right is a negative count */
        if(drive_count <= AP_SWPRR_DRIVE_DISTANCE_RIGHT)
        {
          drive_count = 0;
          loop_count = 0;
          prog_state = AP_SWPRR_STATE_PAUSE_AFTER_RIGHT;
        }
        else if(loop_count > 120)
        {
          drive_count = 0;
          loop_count = 0;
          prog_state = AP_SWPRR_STATE_RAISE_ARM;
        }
        else
        {
          loop_count++;
        }
      }
      else
      {
        prog_state = AP_SWPRR_STATE_RAISE_ARM;
      }

      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_SWPRR_STATE_PAUSE_AFTER_RIGHT:
      printf("STATE 3 %d\r", loop_count);
      Oi_crab_x  = 254;
      Oi_drive_x  = 127;
      Oi_drive_y  = 127;
      Oi_sw_turbo = 1;

      if(loop_count >= AP_SWPRR_PAUSE_AFTER_RIGHT_COUNT)
      {
        loop_count = 0;
        prog_state = AP_SWPRR_STATE_DRIVE_LEFT;
      }
      else
      {
        loop_count++;
      }
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_SWPRR_STATE_DRIVE_LEFT:
      printf("STATE 4 %d %d\r", loop_count, drive_count);
      Oi_crab_x  = 254;
      Oi_drive_x  = 127;
      Oi_drive_y  = (127 + AP_SWPRR_DRIVE_SPEED);
      Oi_sw_turbo = 1;

      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;

        if(drive_count >= AP_SWPRR_DRIVE_DISTANCE_LEFT)
        {
          drive_count = 0;
          loop_count = 0;
          prog_state = AP_SWPRR_STATE_PAUSE_AFTER_LEFT;
        }
        else if(loop_count > 120)
        {
          drive_count = 0;
          loop_count = 0;
          prog_state = AP_SWPRR_STATE_RAISE_ARM;
        }
        else
        {
          loop_count++;
        }
      }
      else
      {
        prog_state = AP_SWPRR_STATE_RAISE_ARM;
      }

      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_SWPRR_STATE_PAUSE_AFTER_LEFT:
      printf("STATE 5 %d\r", loop_count);
        Oi_crab_x  = 254;
        Oi_drive_x  = 127;
        Oi_drive_y  = 127;
        Oi_sw_turbo = 1;

        if(loop_count >= AP_SWPRR_PAUSE_AFTER_LEFT_COUNT)
        {
          loop_count = 0;
          prog_state = AP_SWPRR_STATE_RAISE_ARM;
        }
        else
        {
          loop_count++;
        }
        prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_SWPRR_STATE_RAISE_ARM:
      printf("STATE 6 %d\r", loop_count);
      Oi_crab_x  = 127;
      Oi_drive_x  = 127;
      Oi_drive_y  = 127;
      Oi_sw_turbo = 1;
      feedback_tgt_pos.feedback_enabled = 1;

      /* Give the arm 4 seconds to raise */
      if(loop_count >= AP_SWPRR_RAISE_ARM_COUNT)
      {
        feedback_tgt_pos.feedback_enabled = 0;
        prog_state = AP_SWPRR_STATE_DONE;
        /*
        printf("SAFE TIMER EXPIRED\r");
        */
      }
      else
      {
        feedback_tgt_pos.feedback_enabled = 1;
        SET_Oi_sw_arm_preset_grab_hp();
        loop_count++;
      }
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_SWPRR_STATE_DONE:
      Oi_crab_x  = 127;
      Oi_drive_x  = 127;
      Oi_drive_y  = 127;
      Oi_sw_turbo = 1;
      prog_ret = AUTO_PROGRAM_DONE;
      printf("SWEEPER RIGHT DONE\r");
      break;
    default:
      Oi_crab_x  = 127;
      Oi_drive_x  = 127;
      Oi_drive_y  = 127;
      Oi_sw_turbo = 1;
      prog_state = AP_SWPRR_STATE_DONE;
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
  }

  drive_preproc();
  crab_stick_input();
  drive_stick_input(FALSE);
  arm_control(1);

  return prog_ret;
}
