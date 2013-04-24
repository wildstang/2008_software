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


#define AP_SONR_CCW_RAISE_ARM_COUNT        (1)
#define AP_SONR_CCW_DRIVE_TO_RACK_COUNT    (890)
#define AP_SONR_CCW_PAUSE_AT_RACK_COUNT    (30)
#define AP_SONR_CCW_CRAB_AT_RACK_COUNT     (20)
#define AP_SONR_CCW_PAUSE_AFTER_FIND_COUNT (15)
#define AP_SONR_CCW_PAUSE_AT_SPIDER        (20)
#define AP_SONR_CCW_LOWER_ARM_COUNT        (15)
#define AP_SONR_CCW_RELEASE_TUBE_COUNT     (10)
#define AP_SONR_CCW_RELEASE_TUBE_BAIL_COUNT  (40)
#define AP_SONR_CCW_DRIVE_HOME_RELEASE_COUNT (-100)
#define AP_SONR_CCW_DRIVE_HOME_STORAGE_COUNT (-600)

#define AP_SONR_CCW_DRIVE_TO_RACK_IGNORE_SONAR_DIST  \
          ((AP_SONR_CCW_DRIVE_TO_RACK_COUNT * 3) / 4)
#define AP_SONR_CCW_DRIVE_TO_RACK_TIMEOUT  (500)
#define AP_SONR_CCW_FIND_SPIDER_TIMEOUT    (200)
#define AP_SONR_CCW_DRIVE_TO_SPIDER_TO     (120)

#define AP_SONR_CCW_DRIVE_TO_RACK_SPEED    (35)
#define AP_SONR_CCW_FIND_SPIDER_SPEED      (-60)
#define AP_SONR_CCW_DRIVE_TO_SPIDER_SPEED  (35)
#define AP_SONR_CCW_DRIVE_HOME_RELEASE_SPEED (-35)
#define AP_SONR_CCW_DRIVE_HOME_STORAGE_SPEED (-40)

#define AP_SONR_CCW_BLIND_DRIVE_DIST       (250)
#define AP_SONR_CCW_BLIND_DRIVE_SPEED      (35)

#define SONAR_FOUND_OBJECT       (19500)
#define SONAR_SCORE_MID_DIST     (11000)
#define SONAR_CONFIDENCE_COUNT_1   (6)
#define SONAR_CONFIDENCE_COUNT_2   (6)

/* Sonar Squirrel - CCW
 */

UINT8 ap_sonar_ccw(void)
{
  static UINT8 prog_state = AP_SONR_CCW_STATE_INIT;
  static SonarStmParamType helper_params;
  UINT8 prog_ret = AUTO_PROGRAM_DONE;
  SonarStmRetValType sonar_stm_ret = SONAR_STM_RET_DONE_FAIL;
  /*printf("SONAR-CCW %d %d %d\r", prog_state);
    */

  switch(prog_state)
  {
    case AP_SONR_CCW_STATE_INIT:
      printf("SONAR-CCW INIT\r");
      helper_params.use_theta_correct = 0;
      helper_params.raise_arm_count_init = AP_SONR_CCW_RAISE_ARM_COUNT;
      helper_params.drive_to_rack_dist_count = AP_SONR_CCW_DRIVE_TO_RACK_COUNT;
      helper_params.drive_to_rack_crab_joystick = 127;
      helper_params.drive_to_rack_speed = AP_SONR_CCW_DRIVE_TO_RACK_SPEED;
      helper_params.drive_to_rack_no_sonar_dist_count = AP_SONR_CCW_DRIVE_TO_RACK_IGNORE_SONAR_DIST;
      helper_params.drive_to_rack_timeout_loop_count = AP_SONR_CCW_DRIVE_TO_RACK_TIMEOUT;
      helper_params.pause_at_rack_loop_count = AP_SONR_CCW_PAUSE_AT_RACK_COUNT;
      helper_params.crab_at_rack_loop_count = AP_SONR_CCW_CRAB_AT_RACK_COUNT;
      helper_params.arc_direction = CRAB_ARC_CCW;
      helper_params.find_spider_speed = AP_SONR_CCW_FIND_SPIDER_SPEED;
      helper_params.find_spider_timeout_loop_count = AP_SONR_CCW_FIND_SPIDER_TIMEOUT;
      helper_params.found_pause_loop_count = AP_SONR_CCW_PAUSE_AFTER_FIND_COUNT;
      helper_params.drive_to_spider_speed = AP_SONR_CCW_DRIVE_TO_SPIDER_SPEED;
      helper_params.drive_to_spider_timeout_loop_count = AP_SONR_CCW_DRIVE_TO_SPIDER_TO;
      helper_params.pause_at_spider_loop_count = AP_SONR_CCW_PAUSE_AT_SPIDER;
      helper_params.blind_drive_dist = AP_SONR_CCW_BLIND_DRIVE_DIST;
      helper_params.blind_drive_speed = AP_SONR_CCW_BLIND_DRIVE_SPEED;

      prog_state = AP_SONR_CCW_STATE_PLACE_TUBE;
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_PLACE_TUBE:
      sonar_stm_ret = ap_sonar_stm(helper_params);
      printf("PLACE TUBE: %d\r", sonar_stm_ret);
      if(sonar_stm_ret == SONAR_STM_RET_DONE_SUCCESS)
      {
        prog_state = AP_SONR_CCW_STATE_DONE;
      }
      else if(sonar_stm_ret == SONAR_STM_RET_DONE_FAIL)
      {
        prog_state = AP_SONR_CCW_STATE_DONE;
      }
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_SONR_CCW_STATE_DONE:
      prog_ret = AUTO_PROGRAM_DONE;
      printf("SONAR-CCW DONE\r");
      break;
    default:
      prog_ret = AUTO_PROGRAM_DONE;

  }


  return prog_ret;
#if 0
  static UINT8 prog_state = AP_SONR_CCW_STATE_RAISE_ARM_INIT;
  UINT8 prog_ret = AUTO_PROGRAM_DONE;
  static UINT16 loop_count = 0;
  static INT16 drive_count = 0;
  static UINT8 found_spider_count = 0;
  static UINT8 end_after_release = 0;

  //printf("SONAR-CCW %d %d %d\r", prog_state, loop_count, drive_count);
  Oi_drive_y = 127;
  feedback_tgt_pos.feedback_enabled = 0;
  Oi_sw_theta_correct = 1;

  switch(prog_state)
  {
    case AP_SONR_CCW_STATE_RAISE_ARM_INIT:
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

      if(loop_count >= AP_SONR_CCW_RAISE_ARM_COUNT)
      {
        loop_count = 0;
        feedback_tgt_pos.feedback_enabled = 0;
        prog_state = AP_SONR_CCW_STATE_DRIVE_TO_RACK;
      }
      else
      {
        loop_count++;
      }
      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_SONR_CCW_STATE_DRIVE_TO_RACK:
      printf("STATE 1: %03d %05d %05u %d\r", loop_count, drive_count,
          g_encoder_vals.sonar, found_spider_count);
      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;

        if(drive_count >= AP_SONR_CCW_DRIVE_TO_RACK_COUNT)
        {
          feedback_tgt_pos.feedback_enabled = 0;
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_SONR_CCW_STATE_PAUSE_AT_RACK;
        }
        else if(drive_count > AP_SONR_CCW_DRIVE_TO_RACK_IGNORE_SONAR_DIST)
        {
          if(g_encoder_vals.sonar <= SONAR_FOUND_OBJECT)
          {
            if(found_spider_count >= SONAR_CONFIDENCE_COUNT_1)
            {
              feedback_tgt_pos.feedback_enabled = 0;
              loop_count = 0;
              drive_count = 0;
              found_spider_count = 0;
              prog_state = AP_SONR_CCW_STATE_PAUSE_AFTER_FIND;
            }
            else
            {
              found_spider_count++;
              Oi_drive_y = (127 + AP_SONR_CCW_DRIVE_TO_RACK_SPEED);
              Oi_sw_turbo = 1;
              feedback_tgt_pos.feedback_enabled = 1;
            }
          }
          else
          {
            found_spider_count = 0;
            Oi_drive_y = (127 + AP_SONR_CCW_DRIVE_TO_RACK_SPEED);
            Oi_sw_turbo = 1;
            feedback_tgt_pos.feedback_enabled = 1;
          }
          loop_count++;
        }
        else if(loop_count >= AP_SONR_CCW_DRIVE_TO_RACK_TIMEOUT)
        {
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_SONR_CCW_STATE_DONE;
        }
        else
        {
          Oi_drive_y = (127 + AP_SONR_CCW_DRIVE_TO_RACK_SPEED);
          Oi_sw_turbo = 1;
          feedback_tgt_pos.feedback_enabled = 1;
          loop_count++;
        }
        prog_ret = AUTO_PROGRAM_NOT_DONE;
      }
      else
      {
        prog_state = AP_SONR_CCW_STATE_DONE;
      }

      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_PAUSE_AT_RACK:
      printf("STATE 2: %03d %05d %05u\r", loop_count, drive_count,
          g_encoder_vals.sonar);
      feedback_tgt_pos.feedback_enabled = 1;
      if(loop_count >= AP_SONR_CCW_PAUSE_AT_RACK_COUNT)
      {
        loop_count = 0;
        prog_state = AP_SONR_CCW_STATE_CRAB_AT_RACK;
      }
      else
      {
        loop_count++;
      }
      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_CRAB_AT_RACK:
      printf("STATE 2a: %03d %05d\r", loop_count, drive_count);
      feedback_tgt_pos.feedback_enabled = 1;
      if(loop_count >= AP_SONR_CCW_CRAB_AT_RACK_COUNT)
      {
        loop_count = 0;
        prog_state = AP_SONR_CCW_STATE_FIND_SPIDER;
      }
      else
      {
        Oi_sw_crab_arc_ccw = 1;
        loop_count++;
      }
      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_FIND_SPIDER:
      feedback_tgt_pos.feedback_enabled = 0;

      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        printf("STATE 3: %03d %05d %d %05u\r", loop_count, drive_count,
                                               found_spider_count, g_encoder_vals.sonar);
        if(g_encoder_vals.sonar <= SONAR_FOUND_OBJECT)
        {
          if(found_spider_count >= SONAR_CONFIDENCE_COUNT_2)
          {
            Oi_sw_crab_arc_ccw = 0;
            loop_count = 0;
            prog_state = AP_SONR_CCW_STATE_PAUSE_AFTER_FIND;
          }
          else
          {
            found_spider_count++;
            Oi_sw_crab_arc_ccw = 1;
            Oi_drive_y = (127 + AP_SONR_CCW_FIND_SPIDER_SPEED);
            loop_count++;
          }
        }
        else if(loop_count >= AP_SONR_CCW_FIND_SPIDER_TIMEOUT)
        {
          loop_count = 0;
          end_after_release = 1;
          prog_state = AP_SONR_CCW_STATE_LOWER_ARM;
        }
        else
        {
          found_spider_count = 0;
          Oi_sw_crab_arc_ccw = 1;
          Oi_drive_y = (127 + AP_SONR_CCW_FIND_SPIDER_SPEED);
          loop_count++;
        }
        prog_ret = AUTO_PROGRAM_NOT_DONE;
      }
      else
      {
        prog_state = AP_SONR_CCW_STATE_DONE;
      }

      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_PAUSE_AFTER_FIND:
      printf("STATE 4: %03d %05d\r", loop_count, drive_count);
      feedback_tgt_pos.feedback_enabled = 0;
      if(loop_count >= AP_SONR_CCW_PAUSE_AFTER_FIND_COUNT)
      {
        loop_count = 0;
        prog_state = AP_SONR_CCW_STATE_DRIVE_TO_SPIDER;
      }
      else
      {
       loop_count++;
      }
      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_DRIVE_TO_SPIDER:
      printf("STATE 5: %03d %05d %05u\r", loop_count, drive_count, g_encoder_vals.sonar);
      feedback_tgt_pos.feedback_enabled = 0;

      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        if(g_encoder_vals.sonar <= SONAR_SCORE_MID_DIST)
        {
          loop_count = 0;
          prog_state = AP_SONR_CCW_STATE_PAUSE_AT_SPIDER;
        }
        else if(loop_count >= 120)
        {
          loop_count = 0;
          end_after_release = 1;
          prog_state = AP_SONR_CCW_STATE_LOWER_ARM;
        }
        else
        {
          Oi_sw_turbo = 1;
          Oi_drive_y = (127 + AP_SONR_CCW_DRIVE_TO_SPIDER_SPEED);
          loop_count++;
        }
        prog_ret = AUTO_PROGRAM_NOT_DONE;
      }
      else
      {
        prog_state = AP_SONR_CCW_STATE_DONE;
      }

      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_PAUSE_AT_SPIDER:
      printf("STATE 6: %03d %05d\r", loop_count, drive_count);
      feedback_tgt_pos.feedback_enabled = 0;
      if(loop_count >= AP_SONR_CCW_PAUSE_AT_SPIDER)
      {
        loop_count = 0;
        prog_state = AP_SONR_CCW_STATE_LOWER_ARM;
      }
      else
      {
       loop_count++;
      }
      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_LOWER_ARM:
      printf("STATE 7: %03d %05d\r", loop_count, drive_count);
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

      if(loop_count >= AP_SONR_CCW_LOWER_ARM_COUNT)
      {
        loop_count = 0;
        feedback_tgt_pos.feedback_enabled = 0;
        prog_state = AP_SONR_CCW_STATE_RELEASE_TUBE;
      }
      else
      {
        loop_count++;
      }
      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_RELEASE_TUBE:
      printf("STATE 8: %03d %05d\r", loop_count, drive_count);
      if(loop_count >= AP_SONR_CCW_RELEASE_TUBE_COUNT)
      {
        loop_count = 0;
        drive_count = 0;
        prog_state = AP_SONR_CCW_STATE_GO_HOME_RELEASE;
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

    case AP_SONR_CCW_STATE_GO_HOME_RELEASE:
      printf("STATE 9: %03d %05d\r", loop_count, drive_count);
      feedback_tgt_pos.feedback_enabled = 0;
      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;

        /* Going backwards, so count will be negative */
        if(drive_count <= AP_SONR_CCW_DRIVE_HOME_RELEASE_COUNT)
        {
          loop_count = 0;
          drive_count = 0;
          if(end_after_release == 1)
          {
            prog_state = AP_SONR_CCW_STATE_DONE;
          }
          else
          {
          //prog_state = AP_SONR_CCW_STATE_GO_HOME_STORAGE;
            prog_state = AP_SONR_CCW_STATE_DONE;
          }
        }
        else if(loop_count >= 120)
        {
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_SONR_CCW_STATE_DONE;
        }
        else
        {
          Oi_drive_y = (127 + AP_SONR_CCW_DRIVE_HOME_RELEASE_SPEED);
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

        prog_ret = AUTO_PROGRAM_NOT_DONE;
      }
      else
      {
        prog_state = AP_SONR_CCW_STATE_DONE;
      }

      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_GO_HOME_STORAGE:
      printf("STATE 10: %03d %05d\r", loop_count, drive_count);

      if(g_cc_encoder_ret_val == CC_SUCCESS)
      {
        drive_count += g_encoder_vals.right_back;

        /* Count will be negative */
        if(drive_count <= AP_SONR_CCW_DRIVE_HOME_STORAGE_COUNT)
        {
          feedback_tgt_pos.feedback_enabled = 0;
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_SONR_CCW_STATE_RAISE_ARM;
        }
        else if(loop_count >= 250)
        {
          loop_count = 0;
          drive_count = 0;
          prog_state = AP_SONR_CCW_STATE_DONE;
        }
        else
        {
          Oi_drive_y = (127 + AP_SONR_CCW_DRIVE_HOME_STORAGE_SPEED);
          Oi_sw_turbo = 1;
          SET_Oi_sw_arm_preset_storage();
          loop_count++;
        }
        prog_ret = AUTO_PROGRAM_NOT_DONE;
      }
      else
      {
        prog_state = AP_SONR_CCW_STATE_DONE;
      }

      arm_control(1);
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_RAISE_ARM:
      printf("STATE 11: %03d %05d\r", loop_count, drive_count);
      prog_state = AP_SONR_CCW_STATE_DONE;
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_CCW_STATE_DONE:
      printf("STATE 12: %03d %05d\r", loop_count, drive_count);
      prog_ret = AUTO_PROGRAM_DONE;
      printf("SONAR-CCW DONE\r");
      break;

    default:
      prog_state = AP_SONR_CCW_STATE_DONE;
      break;
  }

  drive_preproc();
  crab_stick_input();
  drive_stick_input(FALSE);

  return prog_ret;
#endif
}
