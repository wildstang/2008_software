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


#define AP_SONR_SIDE_CW_RAISE_ARM_COUNT        (20)
#define AP_SONR_SIDE_CW_DRIVE_TO_RACK_COUNT    (-1220)
#define AP_SONR_SIDE_CW_PAUSE_AT_RACK_COUNT    (20)
#define AP_SONR_SIDE_CW_CRAB_AT_RACK_COUNT     (30)
#define AP_SONR_SIDE_CW_PAUSE_AFTER_FIND_COUNT (20)
#define AP_SONR_SIDE_CW_PAUSE_AT_SPIDER        (30)
#define AP_SONR_SIDE_CW_LOWER_ARM_COUNT        (20)
#define AP_SONR_SIDE_CW_RELEASE_TUBE_COUNT     (10)
#define AP_SONR_SIDE_CW_RELEASE_TUBE_BAIL_COUNT  (40)
#define AP_SONR_SIDE_CW_DRIVE_HOME_RELEASE_COUNT (-100)
#define AP_SONR_SIDE_CW_DRIVE_HOME_STORAGE_COUNT (-600)

#define AP_SONR_SIDE_CW_DRIVE_TO_RACK_IGNORE_SONAR_DIST  \
             (AP_SONR_SIDE_CW_DRIVE_TO_RACK_COUNT)
#define AP_SONR_SIDE_CW_DRIVE_TO_RACK_TIMEOUT  (500)
#define AP_SONR_SIDE_CW_FIND_SPIDER_TIMEOUT    (200)
#define AP_SONR_SIDE_CW_DRIVE_TO_SPIDER_TO     (120)

#define AP_SONR_SIDE_CW_DRIVE_TO_RACK_SPEED    (-85)
#define AP_SONR_SIDE_CW_FIND_SPIDER_SPEED      (-60)
#define AP_SONR_SIDE_CW_DRIVE_TO_SPIDER_SPEED  (35)
#define AP_SONR_SIDE_CW_DRIVE_HOME_RELEASE_SPEED (-35)
#define AP_SONR_SIDE_CW_DRIVE_HOME_STORAGE_SPEED (-40)

#define AP_SONR_SIDE_CW_BLIND_DRIVE_DIST       (250)
#define AP_SONR_SIDE_CW_BLIND_DRIVE_SPEED      (35)

#define SONAR_FOUND_OBJECT       (19500)
#define SONAR_SCORE_MID_DIST     (11000)
#define SONAR_CONFIDENCE_COUNT_1   (6)
#define SONAR_CONFIDENCE_COUNT_2   (6)

/* Sonar Squirrel Side- CW
 */

UINT8 ap_sonar_side_cw(void)
{
  static UINT8 prog_state = AP_SONR_SIDE_CW_STATE_INIT;
  static SonarStmParamType helper_params;
  UINT8 prog_ret = AUTO_PROGRAM_DONE;
  SonarStmRetValType sonar_stm_ret = SONAR_STM_RET_DONE_FAIL;

  switch(prog_state)
  {
    case AP_SONR_SIDE_CW_STATE_INIT:
      helper_params.use_theta_correct = 0;
      helper_params.raise_arm_count_init = AP_SONR_SIDE_CW_RAISE_ARM_COUNT;
      helper_params.drive_to_rack_dist_count = AP_SONR_SIDE_CW_DRIVE_TO_RACK_COUNT;
      helper_params.drive_to_rack_crab_joystick = 0;
      helper_params.drive_to_rack_speed = AP_SONR_SIDE_CW_DRIVE_TO_RACK_SPEED;
      helper_params.drive_to_rack_no_sonar_dist_count = AP_SONR_SIDE_CW_DRIVE_TO_RACK_IGNORE_SONAR_DIST;
      helper_params.drive_to_rack_timeout_loop_count = AP_SONR_SIDE_CW_DRIVE_TO_RACK_TIMEOUT;
      helper_params.pause_at_rack_loop_count = AP_SONR_SIDE_CW_PAUSE_AT_RACK_COUNT;
      helper_params.crab_at_rack_loop_count = AP_SONR_SIDE_CW_CRAB_AT_RACK_COUNT;
      helper_params.arc_direction = CRAB_ARC_CW;
      helper_params.find_spider_speed = AP_SONR_SIDE_CW_FIND_SPIDER_SPEED;
      helper_params.find_spider_timeout_loop_count = AP_SONR_SIDE_CW_FIND_SPIDER_TIMEOUT;
      helper_params.found_pause_loop_count = AP_SONR_SIDE_CW_PAUSE_AFTER_FIND_COUNT;
      helper_params.drive_to_spider_speed = AP_SONR_SIDE_CW_DRIVE_TO_SPIDER_SPEED;
      helper_params.drive_to_spider_timeout_loop_count = AP_SONR_SIDE_CW_DRIVE_TO_SPIDER_TO;
      helper_params.pause_at_spider_loop_count = AP_SONR_SIDE_CW_PAUSE_AT_SPIDER;
      helper_params.blind_drive_dist = AP_SONR_SIDE_CW_BLIND_DRIVE_DIST;
      helper_params.blind_drive_speed = AP_SONR_SIDE_CW_BLIND_DRIVE_SPEED;


      prog_state = AP_SONR_SIDE_CW_STATE_PLACE_TUBE;
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;

    case AP_SONR_SIDE_CW_STATE_PLACE_TUBE:
      sonar_stm_ret = ap_sonar_stm(helper_params);
      printf("PLACE TUBE: %d\r", sonar_stm_ret);
      if(sonar_stm_ret == SONAR_STM_RET_DONE_SUCCESS)
      {
        prog_state = AP_SONR_SIDE_CW_STATE_DONE;
      }
      else if(sonar_stm_ret == SONAR_STM_RET_DONE_FAIL)
      {
        prog_state = AP_SONR_SIDE_CW_STATE_DONE;
      }
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      break;
    case AP_SONR_SIDE_CW_STATE_DONE:
      prog_ret = AUTO_PROGRAM_DONE;
      printf("SONAR-SIDE CW DONE\r");
      break;
    default:
      prog_ret = AUTO_PROGRAM_DONE;

  }


  return prog_ret;
}
