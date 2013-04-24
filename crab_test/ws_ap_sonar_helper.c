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

#define AP_SONR_LOWER_ARM_COUNT         (20)
#define AP_SONR_RELEASE_TUBE_COUNT      (10)
#define AP_SONR_DRIVE_RELEASE_COUNT     (-100)
#define AP_SONR_DRIVE_RELEASE_SPEED     (-35)
#define AP_SONR_DRIVE_RELEASE_TIMEOUT   (120)

#define SONAR_FOUND_OBJECT              (19500)
#define SONAR_SCORE_MID_DIST            (11000)
#define SONAR_CONFIDENCE_COUNT_1        (6)
#define SONAR_CONFIDENCE_COUNT_2        (6)

#define PRESS_ARC_CRAB_SW(direction) \
        do \
        { \
          if((direction) == CRAB_ARC_CCW) \
          { \
            Oi_sw_crab_arc_ccw = 1; \
          } \
          else if((direction) == CRAB_ARC_CW) \
          { \
            Oi_sw_crab_arc_cw = 1; \
          } \
          else \
          { \
            prog_state = AP_SONR_STATE_DONE_FAIL; \
          } \
        } while(0)

UINT8 ap_sonar_stm(SonarStmParamType param)
{
  static UINT8 prog_state = AP_SONR_STATE_RAISE_ARM_INIT;
  SonarStmRetValType prog_ret = SONAR_STM_RET_DONE_FAIL;
  static UINT16 loop_count = 0;
  static INT16 drive_count = 0;
  static UINT8 found_spider_count = 0;
  static UINT8 end_after_release = 0;

  //printf("SONAR %d %d %d\r", prog_state, loop_count, drive_count);
  Oi_drive_y = 127;
  feedback_tgt_pos.feedback_enabled = 0;
  Oi_sw_theta_correct = param.use_theta_correct;
  Oi_sw_crab_arc_ccw = 0;
  Oi_sw_crab_arc_cw = 0;

  switch(prog_state)
  {
    case AP_SONR_STATE_RAISE_ARM_INIT:
      /* Raise arm prior to driving */
      printf("STATE 0: %03d %05d\r", loop_count, drive_count);
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.shoulder =
        (calibration_vals.shoulder_horizontal >> 2) + 63;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) - 52;
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

      if(loop_count >= param.raise_arm_count_init)
      {
        /* Reached the raise count, move on */
        loop_count = 0;
        feedback_tgt_pos.feedback_enabled = 0;
        prog_state = AP_SONR_STATE_DRIVE_TO_RACK;
      }
      else
      {
        loop_count++;
      }

      Oi_crab_x = param.drive_to_rack_crab_joystick;

      arm_control(1);
      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;
    case AP_SONR_STATE_DRIVE_TO_RACK:
      /* Drive to the rack.  Bail out if the CC isn't talking to us */
      printf("STATE 1: %03d %05d %05u %d\r", loop_count, drive_count,
          g_encoder_vals.sonar, found_spider_count);
      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;




        if( (((INT16)param.drive_to_rack_dist_count >= (INT16)0) &&
             ((INT16)drive_count >= (INT16)param.drive_to_rack_dist_count)) ||
            (((INT16)param.drive_to_rack_dist_count < (INT16)0) &&
             ((INT16)drive_count <= (INT16)param.drive_to_rack_dist_count)))
        {
          /* We're at the correct distance, move on */
          feedback_tgt_pos.feedback_enabled = 0;
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_SONR_STATE_PAUSE_AT_RACK;
        }
        else if( (((INT16)param.drive_to_rack_dist_count >= (INT16)0) &&
             ((INT16)drive_count >= (INT16)param.drive_to_rack_no_sonar_dist_count)) ||
            (((INT16)param.drive_to_rack_dist_count < (INT16)0) &&
             ((INT16)drive_count <= (INT16)param.drive_to_rack_no_sonar_dist_count)))
        {
          printf("LISTEN %d %d\r", drive_count, param.drive_to_rack_no_sonar_dist_count);
          /* We're now allowed to use sonar.  If we find something,
             we want to see it for multiple loops before we consider
             it found.
           */
          if(g_encoder_vals.sonar <= SONAR_FOUND_OBJECT)
          {
            if(found_spider_count >= SONAR_CONFIDENCE_COUNT_1)
            {
              /* We've detected something.  Go score on it */
              feedback_tgt_pos.feedback_enabled = 0;
              loop_count = 0;
              drive_count = 0;
              found_spider_count = 0;
              prog_state = AP_SONR_STATE_PAUSE_AFTER_FIND;
            }
            else
            {
              /* Found something.  Count it towards the confidence */
              found_spider_count++;
              Oi_drive_y = (127 + param.drive_to_rack_speed);
              Oi_sw_turbo = 1;
              feedback_tgt_pos.feedback_enabled = 1;
            }
          }
          else
          {
            /* Nothing found....drive */
            found_spider_count = 0;
            Oi_drive_y = (127 + param.drive_to_rack_speed);
            Oi_sw_turbo = 1;
            feedback_tgt_pos.feedback_enabled = 1;
          }
          loop_count++;
        }
        else if(loop_count >= param.drive_to_rack_timeout_loop_count)
        {
          /* Timed out....fail */
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_SONR_STATE_DONE_FAIL;
        }
        else
        {
          /* Drive */
          Oi_drive_y = (127 + param.drive_to_rack_speed);
          Oi_sw_turbo = 1;
          feedback_tgt_pos.feedback_enabled = 1;
          loop_count++;
        }
      }
      else
      {
        /* The CC isn't talking to us */
        prog_state = AP_SONR_STATE_DONE_FAIL;
      }

      Oi_crab_x = param.drive_to_rack_crab_joystick;

      arm_control(1);
      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;

    case AP_SONR_STATE_PAUSE_AT_RACK:
      printf("STATE 2: %03d %05d %05u\r", loop_count, drive_count,
          g_encoder_vals.sonar);
      /* Drive sticks are centered above, so we won't move */
      feedback_tgt_pos.feedback_enabled = 1;
      if(loop_count >= param.pause_at_rack_loop_count)
      {
        /* Move on */
        loop_count = 0;
        prog_state = AP_SONR_STATE_CRAB_AT_RACK;
      }
      else
      {
        loop_count++;
      }

      Oi_crab_x = param.drive_to_rack_crab_joystick;

      arm_control(1);
      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;

    case AP_SONR_STATE_CRAB_AT_RACK:
      /* Turn the wheels without driving */
      printf("STATE 2a: %03d %05d\r", loop_count, drive_count);

      if(loop_count >= param.crab_at_rack_loop_count)
      {
        loop_count = 0;
        prog_state = AP_SONR_STATE_FIND_SPIDER;
      }
      else
      {
        loop_count++;
      }

      PRESS_ARC_CRAB_SW(param.arc_direction);

      feedback_tgt_pos.feedback_enabled = 1;
      arm_control(1);

      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;

    case AP_SONR_STATE_FIND_SPIDER:
      /* Drive on an arc until we've found something */

      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        printf("STATE 3: %03d %05d %d %05u\r", loop_count, drive_count,
                                               found_spider_count, g_encoder_vals.sonar);
        if(g_encoder_vals.sonar <= SONAR_FOUND_OBJECT)
        {
          /* We've detected something with sonar. */

          if(found_spider_count >= SONAR_CONFIDENCE_COUNT_2)
          {
            /* We have enough confidence, so move on */
            /* Disable all arc crabbing */
            Oi_sw_crab_arc_ccw = 0;
            Oi_sw_crab_arc_cw = 0;
            loop_count = 0;
            prog_state = AP_SONR_STATE_PAUSE_AFTER_FIND;
          }
          else
          {
            /* We're not confident yet, so keep going */
            found_spider_count++;
            PRESS_ARC_CRAB_SW(param.arc_direction);
            Oi_drive_y = (127 + param.find_spider_speed);
            loop_count++;
          }
        }
        else if(loop_count >= param.find_spider_timeout_loop_count)
        {
          /* Timeout....drop the tube and stop*/
          loop_count = 0;
          end_after_release = 1;
          //prog_state = AP_SONR_STATE_LOWER_ARM;
          prog_state = AP_SONR_STATE_PAUSE_AFTER_FIND;
        }
        else
        {
          /* Drive */
          found_spider_count = 0;
          PRESS_ARC_CRAB_SW(param.arc_direction);
          Oi_drive_y = (127 + param.find_spider_speed);
          loop_count++;
        }
      }
      else
      {
        /* The CC isn't talking to us....fail */
        prog_state = AP_SONR_STATE_DONE_FAIL;
      }

      feedback_tgt_pos.feedback_enabled = 0;
      arm_control(1);
      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;

    case AP_SONR_STATE_PAUSE_AFTER_FIND:
      printf("STATE 4: %03d %05d\r", loop_count, drive_count);
      if(loop_count >= param.found_pause_loop_count)
      {
        /* Move on */
        loop_count = 0;
        drive_count = 0;

        if(end_after_release == 0)
        {
          prog_state = AP_SONR_STATE_DRIVE_TO_SPIDER;
        }
        else
        {
          prog_state = AP_SONR_STATE_BLIND_DRIVE_TO_SPIDER;
        }
      }
      else
      {
       loop_count++;
      }

      feedback_tgt_pos.feedback_enabled = 0;
      arm_control(1);
      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;

    case AP_SONR_STATE_DRIVE_TO_SPIDER:
      printf("STATE 5: %03d %05d %05u\r", loop_count, drive_count, g_encoder_vals.sonar);

      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        if(g_encoder_vals.sonar <= SONAR_SCORE_MID_DIST)
        {
          /* We're close enough to something to score
             move on */
          loop_count = 0;
          prog_state = AP_SONR_STATE_PAUSE_AT_SPIDER;
        }
        else if(loop_count >= param.drive_to_spider_timeout_loop_count)
        {
          /* We timed out.  Release the tube */
          loop_count = 0;
          end_after_release = 1;
          prog_state = AP_SONR_STATE_LOWER_ARM;
        }
        else
        {
          /* Drive forward */
          Oi_sw_turbo = 1;
          Oi_drive_y = (127 + param.drive_to_spider_speed);
          loop_count++;
        }
      }
      else
      {
        /* The CC isn't talking to us....fail */
        prog_state = AP_SONR_STATE_DONE_FAIL;
      }

      feedback_tgt_pos.feedback_enabled = 0;
      arm_control(1);
      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;

    case AP_SONR_STATE_PAUSE_AT_SPIDER:
      printf("STATE 6: %03d %05d\r", loop_count, drive_count);

      if(loop_count >= param.pause_at_spider_loop_count)
      {
        loop_count = 0;
        prog_state = AP_SONR_STATE_LOWER_ARM;
      }
      else
      {
       loop_count++;
      }

      feedback_tgt_pos.feedback_enabled = 0;
      arm_control(1);

      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;

    case AP_SONR_STATE_LOWER_ARM:
      printf("STATE 7: %03d %05d\r", loop_count, drive_count);

      /* Set the arm position */
      feedback_tgt_pos.feedback_enabled = 1;
      feedback_tgt_pos.shoulder =
           (calibration_vals.shoulder_horizontal >> 2) + 63;
      feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) - 75;
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

      if(loop_count >= AP_SONR_LOWER_ARM_COUNT)
      {
        /* We've moved long enough.  Move on */
        loop_count = 0;
        feedback_tgt_pos.feedback_enabled = 0;
        prog_state = AP_SONR_STATE_RELEASE_TUBE;
      }
      else
      {
        loop_count++;
      }
      arm_control(1);
      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;

    case AP_SONR_STATE_RELEASE_TUBE:
      printf("STATE 8: %03d %05d\r", loop_count, drive_count);
      if(loop_count >= AP_SONR_RELEASE_TUBE_COUNT)
      {
        /* We've released long enough.  Move on */
        loop_count = 0;
        drive_count = 0;
        prog_state = AP_SONR_STATE_DRIVE_RELEASE;
      }
      else
      {
        loop_count++;
      }

      /* Open the grippers */
#if ANALOG_GRIPPER
      Oi_sw_gripper = ANALOG_GRIPPER_OPEN;
#else
      Oi_sw_gripper = 1;
#endif

      arm_control(1);
      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;

    case AP_SONR_STATE_DRIVE_RELEASE:
      printf("STATE 9: %03d %05d\r", loop_count, drive_count);
      feedback_tgt_pos.feedback_enabled = 0;
      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;

        /* Going backwards, so count will be negative */
        if(drive_count <= AP_SONR_DRIVE_RELEASE_COUNT)
        {
          /* We've driven long enough */
          loop_count = 0;
          drive_count = 0;
          if(end_after_release == 1)
          {
            prog_state = AP_SONR_STATE_DONE_FAIL;
          }
          else
          {
            prog_state = AP_SONR_STATE_DONE;
          }
        }
        else if(loop_count >= AP_SONR_DRIVE_RELEASE_TIMEOUT)
        {
          /* Timeout....fail */
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_SONR_STATE_DONE_FAIL;
        }
        else
        {
          /* Drive */
          Oi_drive_y = (127 + AP_SONR_DRIVE_RELEASE_SPEED);
          Oi_sw_turbo = 1;
          feedback_tgt_pos.feedback_enabled = 1;
          feedback_tgt_pos.shoulder =
               (calibration_vals.shoulder_horizontal >> 2) + 63;
          feedback_tgt_pos.elbow = (calibration_vals.elbow_horizontal >> 2) - 85;
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
          loop_count++;
        }

      }
      else
      {
        /* The CC isn't talking to us....fail */
        prog_state = AP_SONR_STATE_DONE_FAIL;
      }

      arm_control(1);
      prog_ret = SONAR_STM_RET_NOT_DONE;
      break;

    case AP_SONR_STATE_BLIND_DRIVE_TO_SPIDER:
      printf("STATE 5a: %03d %05d\r", loop_count, drive_count);
      feedback_tgt_pos.feedback_enabled = 0;

      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;
        if(drive_count >= param.blind_drive_dist)
        {
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_SONR_STATE_PAUSE_AT_SPIDER;
        }
        else
        {
          Oi_sw_turbo = 1;
          Oi_drive_y = (127 + param.blind_drive_speed);
          loop_count++;
        }
        prog_ret = AUTO_PROGRAM_NOT_DONE;
      }
      else
      {
        prog_state = AP_SONR_STATE_DONE_FAIL;
      }

      arm_control(1);
      break;

    case AP_SONR_STATE_DONE_FAIL:
      /* Something failed to get here.  Tell the caller so */
      printf("STATE 12: %03d %05d\r", loop_count, drive_count);
      prog_ret = SONAR_STM_RET_DONE_FAIL;
      printf("SONAR-DONE-FAIL\r");
      break;
    case AP_SONR_STATE_DONE:
      /* We think we've successfuly scored.  Tell the caller so */
      printf("STATE 13: %03d %05d\r", loop_count, drive_count);
      prog_ret = SONAR_STM_RET_DONE_SUCCESS;
      printf("SONAR-DONE-SUCCESS\r");
      break;

    default:
      prog_state = AP_SONR_STATE_DONE_FAIL;
      break;
  }

  drive_preproc();
  crab_stick_input();
  drive_stick_input(FALSE);

  return prog_ret;
}
