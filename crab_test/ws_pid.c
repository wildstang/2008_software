/*******************************************************************************
* FILE NAME: ws_pid.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"

#include "ws_pid.h"
#include "ws_io.h"
#include "ws_general.h"

UINT16 g_cur_elbow = 0;

INT16 arm_speed_cur;
INT16 g_elbow_speed;

UINT8 show_pid = 0;
/*******************************************************************************
* FUNCTION NAME: pid
* PURPOSE:
* ARGUMENTS:
* RETURNS:
*
*******************************************************************************/
INT16 ws_pid(PidValsType *pid_vals, INT32 current, INT32 target)
{
  INT32 pid_val;
  INT32 error;
  INT32 prop_term;
  INT32 int_term;
  INT32 deriv_term;
  int i;
  int error_diff;

  /*
  printf("c %d t %d  ", (int)current, (int)target);
  */

  /* calculate the current error */
  error = target - current;
  error_diff = error - pid_vals->last_error[0];

  if( (pid_vals->error_thresh == NO_ERROR_THRESH) ||
      (error > pid_vals->error_thresh) || (error < -pid_vals->error_thresh))
  {
    /* calculate the proportional term of the PID equation */
    prop_term = pid_vals->prop_gain * error;

    if(error == 0)
    {
      pid_vals->integral = (pid_vals->integral * 9) / 10;
    }
    else
    {
      if(disabled_mode != ROBOT_DISABLED)
      {
        /* add the current error to the running integrated value */
        pid_vals->integral += error;
      }
    }


    if((pid_vals->int_reset_thresh != NO_INT_RESET_THRESH) &&
      ((error_diff > pid_vals->int_reset_thresh) ||
       (error_diff < (-pid_vals->int_reset_thresh))))
    {
      pid_vals->integral = 0;
    }


    /* prevent integral wind-up */
    if (pid_vals->max_integral < pid_vals->integral)
    {
      pid_vals->integral = pid_vals->max_integral;
    }
    else if (-(pid_vals->max_integral) > pid_vals->integral)
    {
      pid_vals->integral = -(pid_vals->max_integral);
    }


    /* calculate the integral term using the integrated value & gain */
    int_term = pid_vals->int_gain * pid_vals->integral;

    /* calculate the differential term */
#if 0
    if (p4_wheel > 200)
    {
      printf("n");
#endif
      deriv_term = pid_vals->deriv_gain *
                   (error - pid_vals->last_error[LAST_ERROR_SIZE - 1]);
#if 0
    }
    else
    {
      printf("y");
      deriv_term = (pid_vals->deriv_gain *
          (error - pid_vals->last_error[LAST_ERROR_SIZE - 1])) / error;
    }
#endif

    /* calculate the PID value using the previously calculate P, I, and D terms */
    pid_val = prop_term + int_term - deriv_term;

    pid_val = pid_val / pid_vals->scale_factor;

    /* limit PID value to max & min values */
    MIN(pid_val, pid_vals->min_val);
    MAX(pid_val, pid_vals->max_val);

    if (show_pid == 1)
    {
      printf("e %ld c %ld p %ld i %ld d %ld s %ld ", error,
             current, prop_term, int_term, deriv_term,
             pid_val);

      printf(" le ( ");
      for(i = 0; i < LAST_ERROR_SIZE; i++)
      {
        printf("%d ", pid_vals->last_error[i]);
      }
      printf(") ");

      printf("\r");
    }
  }
  else
  {
    pid_val = 0;
    if(show_pid == 1)
    {
      pid_vals->integral = 0;
      printf("e %ld thresh %d\r", error, (int) pid_vals->error_thresh);
    }
  }

  for(i = (LAST_ERROR_SIZE - 1); i > 0; i--)
  {
    pid_vals->last_error[i] = pid_vals->last_error[i-1];
  }
  pid_vals->last_error[0] = error;

  return (INT16)pid_val;
}



void speed_pid_interrupt(void)
{
  static UINT16 cur_elbow_prev = 0;
  static UINT8  do_read = 0;

  do_read ^= 1;

  if(do_read)
  {
    g_cur_elbow = Get_Analog_Value(Analog_in_elbow);

    g_elbow_speed = g_cur_elbow - cur_elbow_prev;

  }
  cur_elbow_prev = g_cur_elbow;
}


void pid_last_error_init( PidValsType *pid_vals)
{
  int i;
  for(i = 0; i < LAST_ERROR_SIZE; i++)
  {
    pid_vals->last_error[i] = 0;
  }
}

void clear_pid_vals_history(PidValsType *pid_vals)
{
  pid_vals->integral = 0;
  pid_last_error_init(pid_vals);
}
