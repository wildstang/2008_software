#include <stdio.h>
#include "ifi_frc.h"
#include "ws_io.h"
#include "ws_includes.h"
#include "ws_autonomous.h"
#include "ws_autoprog.h"
#include "ws_lift.h"

#define AP4_INIT_WAIT              (80)
#define AP4_STRAIGHTAWAY_1_Y        (127 + 64)
#define AP4_STRAIGHTAWAY_1_TICKS    (700)
#define AP4_TURN_CORNER_X          (127 + 90)
#define AP4_TURN_CORNER_Y          (127 + 64)
#define AP4_TURN_CORNER_GYRO_DIFF  (100)
#define AP4_STRAIGHTAWAY_2_Y        (127 + 64)
#define AP4_STRAIGHTAWAY_2_TICKS    (700)
#define AP4_UNFOLD_WAIT            (ACCUM_UNFOLD_LOOPS)

#define TO_AP4_STATE_STRAIGHTAWAY_1  200
#define TO_AP4_STATE_TURN_CORNER     100
#define TO_AP4_STATE_STRAIGHTAWAY_2  200

/* Auto Program 4
 * Drive 3 lines
 */
UINT8 ap_prog4(void)
{
  static UINT8 prog_state = AP_LPR3_STATE_INIT;
  static UINT16 timeout_thresh = TO_NONE;
  static UINT16 timeout_loop = 0;
  static UINT16 loop_count = 0;
  static INT16  des_gyro = 0;
  UINT8 prog_ret = AUTO_PROGRAM_NOT_DONE;

  switch(prog_state)
  {
    case AP_LPR3_STATE_INIT:
      timeout_thresh = TO_NONE;
      printf("AP_LPR3_STATE_INIT ");
      /* Wait for other bots to clear out */
      if (loop_count >= AP4_INIT_WAIT)
      {
        prog_state = AP_LPR3_STATE_STRAIGHTAWAY_1;
        loop_count = 0;
        timeout_loop = 0;
      }
      else
      {
        loop_count++;
      }
      break;
    case AP_LPR3_STATE_STRAIGHTAWAY_1:
      timeout_thresh = TO_AP4_STATE_STRAIGHTAWAY_1;
      printf("AP_LPR3_STATE_STRAIGHTAWAY_1 ");
      if(drive_by_ticks(127, AP4_STRAIGHTAWAY_1_Y,
            AP4_STRAIGHTAWAY_1_TICKS) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        des_gyro = cc_data.data.encoder.orient + AP4_TURN_CORNER_GYRO_DIFF;
        prog_state = AP_LPR3_STATE_TURN_CORNER;
      }
      break;
    case AP_LPR3_STATE_TURN_CORNER:
      timeout_thresh = TO_AP4_STATE_TURN_CORNER;

      printf("AP_LPR3_STATE_TURN_CORNER ");
      if(drive_by_ticks(AP4_TURN_CORNER_X, AP4_TURN_CORNER_Y, des_gyro) ==
         AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        prog_state = AP_LPR3_STATE_STRAIGHTAWAY_2;
      }
      break;
    case AP_LPR3_STATE_STRAIGHTAWAY_2:
      timeout_thresh = TO_AP4_STATE_STRAIGHTAWAY_2;
      printf("AP_LPR3_STATE_STRAIGHTAWAY_2 ");
      if(drive_by_ticks(127, AP4_STRAIGHTAWAY_2_Y,
            AP4_STRAIGHTAWAY_2_TICKS) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        prog_state = AP_LPR3_STATE_UNFOLD;
      }
      break;
    case AP_LPR3_STATE_UNFOLD:
      timeout_thresh = TO_NONE;
      printf("AP_LPR3_STATE_UNFOLD ");
      if (loop_count <= AP4_UNFOLD_WAIT)
      {
        if(loop_count < 5)
        {
          Oi_sw_accum_unfold = 1;
        }
        loop_count++;
      }
      else
      {
        loop_count = 0;
        prog_state = AP_LPR3_STATE_DONE;
      }
      break;
    case AP_LPR3_STATE_DONE:
      timeout_thresh = TO_NONE;
      printf("AP_LPR3_STATE_DONE ");
      prog_state = AP_LPR3_STATE_DONE;
      prog_ret = AUTO_PROGRAM_DONE;
      break;
    default:
      timeout_thresh = TO_NONE;
      prog_state = AP_LPR3_STATE_DONE;
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
      prog_state = AP_LPR3_STATE_DONE;
    }
  }

  return prog_ret;
}

