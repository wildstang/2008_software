#include <stdio.h>
#include "ifi_frc.h"
#include "ws_autonomous.h"
#include "ws_autoprog.h"
#include "ws_includes.h"
#include "ws_cc.h"

/* Auto Program 1 
 * Sleeper - DO NOTHING
 */
UINT8 ap_sleeper(void)
{
  static UINT8 prog_state = AP_SLPR_STATE_DONE;
  static UINT16 l_drive_count = 0, r_drive_count = 0;
  UINT8 prog_ret = AUTO_PROGRAM_DONE;
  printf("SLEEPER ");

  switch(prog_state)
  {
    case AP_SLPR_STATE_DONE:
#if 1
     if(cc_data.type == CC_REQ_ENCODER)
     {
       l_drive_count += cc_data.data.encoder.left;
       r_drive_count += cc_data.data.encoder.right;
     }
     printf("o %4d l %4d r %4d ", cc_data.data.encoder.orient, l_drive_count,
            r_drive_count);
     prog_ret = AUTO_PROGRAM_NOT_DONE;
#else
     prog_ret = AUTO_PROGRAM_DONE;
#endif

      //printf("SLEEPER DONE\r");
      break;
  }
  return prog_ret;
}
