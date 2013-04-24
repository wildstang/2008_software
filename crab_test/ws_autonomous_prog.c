#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_io.h"

#include "ws_autonomous.h"
//#include "ws_autonomous_tasks.h"
#include "ws_autonomous_prog.h"

UINT8 auto_run_program(UINT8 auto_lock, UINT8 prog)
{
  UINT8 ret = AUTO_PROGRAM_DONE;

  /* If we're not locked in, go to the default case
   * of do nothing */
  if(auto_lock == 0)
  {
    printf("NOT LOCKED IN - DO NOTHING\r");
    prog = 0;
  }

  switch(prog)
  {
    case 1:
      ret = ap_sleeper();
      break;
    case 2:
      ret = ap_grabber();
      break;
    case 3:
      ret = ap_sweeper_left();
      break;
    case 4:
      ret = ap_sweeper_right();
      break;
    case 5:
      ret = ap_sonar_cw();
      break;
    case 6:
      ret = ap_sonar_ccw();
      break;
    case 7:
      ret = ap_sonar_side_cw();
      break;
    case 8:
      ret = ap_sonar_side_ccw();
      break;
    default:
      ret = ap_sleeper();
      break;
  }
  return ret;
}


