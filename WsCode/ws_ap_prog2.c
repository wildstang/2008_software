#include <stdio.h>
#include "ifi_frc.h"
#include "ws_io.h"
#include "ws_includes.h"
#include "ws_autonomous.h"
#include "ws_autoprog.h"
#include "ws_pid.h"
#include "ws_drive.h"
#include "ws_lift.h"

#define TO_LPR_STATE_DRIVE_OUT         200
#define TO_LPR_STATE_START_TURN        200
#define TO_LPR_STATE_DRIVE_SHORT_SIDE  200
#define TO_LPR_STATE_FINISH_TURN       200
#define TO_LPR_STATE_DRIVE_AFTER_TURN  200

/* Auto Program 2
 * Drive 3 lines
 */
UINT8 ap_prog2(void)
{
  static UINT8 prog_state = AP_LPR_STATE_DELAY;
  static UINT16 timeout_thresh = TO_NONE;
  static UINT16 timeout_loop = 0;
  static UINT16 loop_count = 0;
  UINT8 prog_ret = AUTO_PROGRAM_NOT_DONE;
  UINT8 delay_loops = 0;
  Oi_sw_theta_correct = 1;

  switch(prog_state)
  {
    case AP_LPR_STATE_DELAY:
      timeout_thresh = TO_NONE;

      /* UNFOLD DURING DELAY!!! */
      if(g_auto_delay == DELAY_1)
      {
        delay_loops = 40;
      }
      else if(g_auto_delay == DELAY_2)
      {
        delay_loops = 80;
      }
      else
      {
        delay_loops = 5;
      }

      Oi_sw_accum_unfold = 1;

      /* Slapper will raise the lift to unfold the wheelie bar */
      Oi_slapper = 255;

      if(loop_count >= delay_loops)
      {
        loop_count = 0;
        prog_state = AP_LPR_STATE_DRIVE_OUT;
      }
      else
      {
        loop_count++;
      }
      break;

    case AP_LPR_STATE_DRIVE_OUT:
      timeout_thresh = TO_NONE;
      printf("AP_LPR_STATE_DRIVE_OUT ");
      if(drive_by_ticks(127, 127 + 40, 3000) == AUTON_DRIVE_DONE)
      //if(drive_by_ticks(127, 127 + 63, 3000) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        prog_state = AP_LPR_STATE_START_TURN;
      }
      break;

    case AP_LPR_STATE_START_TURN:
      timeout_thresh = TO_NONE;
      printf("AP_LPR_STATE_START_TURN ");
      Oi_sw_monster_mode = 1;
      if(drive_by_gyro(127 + 63, 127 + 63, -1000) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_LPR_STATE_DRIVE_SHORT_SIDE;
      }
      break;

    case AP_LPR_STATE_DRIVE_SHORT_SIDE:
      timeout_thresh = TO_NONE;
      printf("AP_LPR_STATE_DRIVE_OUT ");
      if(drive_by_ticks(127, 127 + 40, 200) == AUTON_DRIVE_DONE)
      //if(drive_by_ticks(127, 127 + 63, 200) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        prog_state = AP_LPR_STATE_FINISH_TURN;
      }
      break;

    case AP_LPR_STATE_FINISH_TURN:
      timeout_thresh = TO_NONE;
      printf("AP_LPR_STATE_FINISH_TURN ");
      Oi_sw_monster_mode = 1;
      /* Turn a little extra to pull back in */
      if(drive_by_gyro(127 + 63, 127 + 63, -1200) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_LPR_STATE_DRIVE_AFTER_TURN;
        /*** STOP AFTER TURN FOR NOW ***/
        prog_state = AP_LPR_STATE_DONE;
      }
      break;

    case AP_LPR_STATE_DRIVE_AFTER_TURN:
      timeout_thresh = TO_NONE;
      printf("AP_LPR_STATE_DRIVE_AFTER_TURN ");
      if(drive_by_ticks(127, 127 + 40, 700) == AUTON_DRIVE_DONE)
      //if(drive_by_ticks(127, 127 + 63, 700) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        prog_state = AP_LPR_STATE_DONE;
      }
      break;

    case AP_LPR_STATE_DONE:
      timeout_thresh = TO_NONE;
      printf("AP_LPR_STATE_DONE ");
      prog_state = AP_LPR_STATE_DONE;
      prog_ret = AUTO_PROGRAM_DONE;
      break;

    default:
      timeout_thresh = TO_NONE;
      prog_state = AP_LPR_STATE_DONE;
      break;
  }

  if(timeout_thresh != TO_NONE)
  {
    if(timeout_loop < timeout_thresh)
    {
      if(prog_ret == AUTO_PROGRAM_NOT_DONE)
      {
        timeout_loop++;
      }
    }
    else
    {
      prog_state = AP_LPR_STATE_DONE;
    }
  }

  return prog_ret;
}

