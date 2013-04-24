#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"
#include "ws_cc.h"

#include "ws_autonomous.h"
#include "ws_autonomous_prog.h"

/* Auto Program 1 
 * Sleeper - DO NOTHING
 */
UINT8 ap_sleeper(void)
{
  static UINT8 prog_state = AP_SWPR_STATE_DONE;
  static UINT16 drive_count = 0;
  UINT8 prog_ret = AUTO_PROGRAM_DONE;
  printf("SLEEPER\r");

  switch(prog_state)
  {

    case AP_SWPR_STATE_DONE:
     if(g_cc_encoder_ret_val == CC_SUCCESS)
     {
       drive_count += g_encoder_vals.right_back;
     }
     printf("%d\r", drive_count);


      prog_ret = AUTO_PROGRAM_DONE;
      prog_ret = AUTO_PROGRAM_NOT_DONE;
      //printf("SLEEPER DONE\r");
      break;
  }
  return prog_ret;
}
