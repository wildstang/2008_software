/*******************************************************************************
* FILE NAME: ws_lift.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_frc.h"
#include "ws_includes.h"
#include "ws_io.h"
#include "ws_lift.h"

static LiftStateType g_lift_state = LIFT_STATE_UNKNOWN;
static ManuLiftStateType g_man_lift_state = MANU_LIFT_STATE_DOWN;

static UINT8 g_reinit_auto_lift = 1;
static UINT8 g_reinit_roller = 1;


/*******************************************************************************
* FUNCTION NAME: lift_init
* PURPOSE:       initialize lift state machine
*******************************************************************************/
void lift_init()
{
}

/*******************************************************************************
* FUNCTION NAME: lift_reset
* PURPOSE:       reset the solenoid positions
*******************************************************************************/
void lift_reset()
{
  motor_vals.accum_tilt = ACCUM_TILT_IN;
  motor_vals.nest_tilt = NEST_TILT_IN;
  motor_vals.lift_1 = LIFT_NONE;
  motor_vals.lift_2 = LIFT_NONE;
  motor_vals.nest_level = NEST_LEVEL_NONE;
  motor_vals.ladder = LADDER_NONE;
  /* Need to reset the auto state machine */
  g_reinit_auto_lift = 1;
  g_reinit_roller = 1;
}

/*******************************************************************************
* FUNCTION NAME: lift_control
* PURPOSE:       handles the overall lift state machine
*******************************************************************************/
void lift_control()
{
  if(Oi_sw_lift_disabled == 0)
  {
    if(Oi_sw_lift_manu_mode == 1)
    {
      lift_control_manual();
      g_reinit_auto_lift = 1;
    }
    else
    {
      lift_control_auto();
      g_reinit_auto_lift = 0;
    }

    roller_control();
    slapper_control();

    if((Oi_sw_accum_unfold == 1) && (Oi_sw_accum_unfold_prev == 0))
    {
      unfold_accumulator(TRUE);
    }
    else
    {
      unfold_accumulator(FALSE);
    }
  }
  else
  {
    Oi_roller = ROLLER_OFF;
    g_reinit_roller = 1;
    roller_control();

    Oi_slapper = SLAPPER_THRESH;
    slapper_control();
  }
}

/*******************************************************************************
* FUNCTION NAME: lift_control_manual
* PURPOSE:       provides manual lift control
*******************************************************************************/
void lift_control_manual()
{
  if(Oi_sw_lift_manu_accum_out == 1)
  {
    motor_vals.accum_tilt = ACCUM_TILT_OUT;
  }
  else if(Oi_sw_lift_manu_accum_in == 1)
  {
    motor_vals.accum_tilt = ACCUM_TILT_IN;
  }
  else if((Oi_sw_lift_manu_lift_down == 1) &&
          (Oi_sw_lift_manu_lift_down_prev == 0))
  {
    if(g_man_lift_state == MANU_LIFT_STATE_UP)
    {
      motor_vals.lift_2 = LIFT_DOWN;
      motor_vals.lift_1 = LIFT_UP;
      motor_vals.ladder = LADDER_UP;
      g_man_lift_state = MANU_LIFT_STATE_MID2;
    }
    else if(g_man_lift_state == MANU_LIFT_STATE_MID2)
    {
      motor_vals.lift_2 = LIFT_DOWN;
      motor_vals.lift_1 = LIFT_DOWN;
      motor_vals.ladder = LADDER_UP;
      g_man_lift_state = MANU_LIFT_STATE_MID1;
    }
    else if(g_man_lift_state == MANU_LIFT_STATE_MID1)
    {
      motor_vals.lift_2 = LIFT_DOWN;
      motor_vals.lift_1 = LIFT_DOWN;
      motor_vals.ladder = LADDER_DOWN;
      g_man_lift_state = MANU_LIFT_STATE_DOWN;
    }
    else
    {
      motor_vals.lift_2 = LIFT_DOWN;
      motor_vals.lift_1 = LIFT_DOWN;
      motor_vals.ladder = LADDER_DOWN;
      g_man_lift_state = MANU_LIFT_STATE_DOWN;
    }
  }
  else if((Oi_sw_lift_manu_lift_up == 1) &&
          (Oi_sw_lift_manu_lift_up_prev == 0))
  {
    if(g_man_lift_state == MANU_LIFT_STATE_DOWN)
    {
      motor_vals.lift_2 = LIFT_DOWN;
      motor_vals.lift_1 = LIFT_DOWN;
      motor_vals.ladder = LADDER_UP;
      g_man_lift_state = MANU_LIFT_STATE_MID1;
    }
    else if(g_man_lift_state == MANU_LIFT_STATE_MID1)
    {
      motor_vals.lift_2 = LIFT_DOWN;
      motor_vals.lift_1 = LIFT_UP;
      motor_vals.ladder = LADDER_UP;
      g_man_lift_state = MANU_LIFT_STATE_MID2;
    }
    else if(g_man_lift_state == MANU_LIFT_STATE_MID2)
    {
      motor_vals.lift_2 = LIFT_UP;
      motor_vals.lift_1 = LIFT_UP;
      motor_vals.ladder = LADDER_UP;
      g_man_lift_state = MANU_LIFT_STATE_UP;
    }
    else
    {
      motor_vals.lift_2 = LIFT_UP;
      motor_vals.lift_1 = LIFT_UP;
      motor_vals.ladder = LADDER_UP;
      g_man_lift_state = MANU_LIFT_STATE_UP;
    }
  }
  else if(Oi_sw_lift_manu_nest_in == 1)
  {
    motor_vals.nest_tilt = NEST_TILT_IN;
  }
  else if(Oi_sw_lift_manu_nest_out == 1)
  {
    motor_vals.nest_tilt = NEST_TILT_OUT;
  }
  else if(Oi_sw_lift_manu_level_up == 1)
  {
    motor_vals.nest_level = NEST_LEVEL_UP;
  }
  else if(Oi_sw_lift_manu_level_down == 1)
  {
    motor_vals.nest_level = NEST_LEVEL_DOWN;
  }
}

/*******************************************************************************
* FUNCTION NAME: lift_control_auto
* PURPOSE:       controls the lift with a state machine
*******************************************************************************/
void lift_control_auto(void)
{
  LiftStateType next_state = g_lift_state;
  static LiftStateType prev_state = LIFT_STATE_UNKNOWN;
  static UINT8 first_time_in_state = 1;
  static UINT8 loop_count = 0;

  if(g_reinit_auto_lift == 1)
  {
    g_lift_state = LIFT_STATE_UNKNOWN;
    //g_man_lift_state = MANU_LIFT_STATE_???
  }

  if(prev_state != g_lift_state)
  {
    first_time_in_state = 1;
  }



#if DEBUG_LIFT_STATES
  printf("LS: ");
#endif
  switch (g_lift_state)
  {
    case LIFT_STATE_UNKNOWN:
#if DEBUG_LIFT_STATES
      printf("UNKNOWN ");
#endif

        /* Transition between states */
        if ((Oi_sw_lift_gather == 1) && (Oi_sw_lift_gather_prev == 0))
        {
          next_state = LIFT_STATE_GATHER;
        }
        else if ((Oi_sw_lift_home == 1) && (Oi_sw_lift_home_prev == 0))
        {
          next_state = LIFT_STATE_HOME;
        }
        else if (((Oi_sw_lift_hurdle == 1) &&
                  (Oi_sw_lift_hurdle_prev == 0)) ||
                 ((Oi_sw_lift_drive_by == 1) &&
                  (Oi_sw_lift_drive_by_prev == 0)))
        {
          //next_state = LIFT_STATE_READY_HURDLE;
          next_state = LIFT_STATE_GATHER_TO_HURDLE;
        }
        else
        {
          next_state = LIFT_STATE_UNKNOWN;
        }
      break;

    case LIFT_STATE_HOME:
#if DEBUG_LIFT_STATES
        printf("HOME ");
#endif
        motor_vals.lift_1 = LIFT_DOWN;
        motor_vals.lift_2 = LIFT_DOWN;
        motor_vals.ladder = LADDER_DOWN;
        motor_vals.nest_level = NEST_LEVEL_DOWN;
        motor_vals.nest_tilt = NEST_TILT_IN;
        motor_vals.accum_tilt = ACCUM_TILT_IN;


        g_man_lift_state = MANU_LIFT_STATE_DOWN;


        /* Transition between states */
        if ((Oi_sw_lift_gather == 1) && (Oi_sw_lift_gather_prev == 0))
        {
          next_state = LIFT_STATE_GATHER;
        }
        else if (((Oi_sw_lift_hurdle == 1) &&
                  (Oi_sw_lift_hurdle_prev == 0)) ||
                 ((Oi_sw_lift_drive_by == 1) &&
                  (Oi_sw_lift_drive_by_prev == 0)))
        {
          next_state = LIFT_STATE_GATHER_TO_HURDLE;
          //next_state = LIFT_STATE_READY_HURDLE;
        }
        else
        {
          next_state = LIFT_STATE_HOME;
        }
      break;

    case LIFT_STATE_GATHER:
#if DEBUG_LIFT_STATES
        printf("GATHER ");
#endif
        motor_vals.lift_1 = LIFT_DOWN;
        motor_vals.lift_2 = LIFT_DOWN;
        motor_vals.ladder = LADDER_DOWN;
        motor_vals.nest_level = NEST_LEVEL_DOWN;
        motor_vals.nest_tilt = NEST_TILT_OUT;
        motor_vals.accum_tilt = ACCUM_TILT_IN;

        g_man_lift_state = MANU_LIFT_STATE_DOWN;

        /* Transition between states */
        if (((Oi_sw_lift_home == 1) &&
             (Oi_sw_lift_home_prev == 0)) ||
            ((Oi_sw_lift_drive_by == 1) &&
             (Oi_sw_lift_drive_by_prev == 0)))
        {
          next_state = LIFT_STATE_HOME;
        }
        else if ((Oi_sw_lift_hurdle == 1) &&
                 (Oi_sw_lift_hurdle_prev == 0))
        {
          next_state = LIFT_STATE_HOME;
          //next_state = LIFT_STATE_GATHER_TO_HURDLE;
        }
        else
        {
          next_state = LIFT_STATE_GATHER;
        }
      break;

    case LIFT_STATE_READY_HURDLE:
#if DEBUG_LIFT_STATES
        printf("READY ");
#endif
        if(first_time_in_state == 1)
        {
          loop_count = 0;
        }

        //if(loop_count < 90)
        if(loop_count < 25)
        {
          motor_vals.lift_1 = LIFT_DOWN;
          motor_vals.lift_2 = LIFT_DOWN;
          loop_count++;
          g_man_lift_state = MANU_LIFT_STATE_MID1;
        }
        else
        {
          motor_vals.lift_1 = LIFT_UP;
          motor_vals.lift_2 = LIFT_UP;
          g_man_lift_state = MANU_LIFT_STATE_UP;
        }

        motor_vals.ladder = LADDER_UP;
        motor_vals.nest_level = NEST_LEVEL_UP;
        motor_vals.nest_tilt = NEST_TILT_IN;
        motor_vals.accum_tilt = ACCUM_TILT_IN;


        /* Transition between states */
        if (((Oi_sw_lift_home == 1) && (Oi_sw_lift_home_prev == 0)) ||
            ((Oi_sw_lift_gather == 1) && (Oi_sw_lift_gather_prev == 0)))
        {
          next_state = LIFT_STATE_HOME;
        }
        else if ((Oi_sw_lift_drive_by == 1) &&
                 (Oi_sw_lift_drive_by_prev == 0))
        {
          next_state = LIFT_STATE_DRIVE_BY;
        }
        else if ((Oi_sw_lift_hurdle == 1) &&
                 (Oi_sw_lift_hurdle_prev == 0))
        {
          next_state = LIFT_STATE_HURDLE;
        }
        else
        {
          next_state = LIFT_STATE_READY_HURDLE;
        }
      break;

    case LIFT_STATE_HURDLE:
#if DEBUG_LIFT_STATES
        printf("HURDLE ");
#endif
        motor_vals.lift_1 = LIFT_UP;
        motor_vals.lift_2 = LIFT_UP;
        motor_vals.ladder = LADDER_UP;
        motor_vals.nest_level = NEST_LEVEL_UP;
        motor_vals.nest_tilt = NEST_TILT_OUT;
        motor_vals.accum_tilt = ACCUM_TILT_IN;

        g_man_lift_state = MANU_LIFT_STATE_UP;

        /* Transition between states */
        if ((Oi_sw_lift_home == 1) && (Oi_sw_lift_home_prev == 0))
        {
          next_state = LIFT_STATE_HURDLE_LOWER;
        }
        else if ((Oi_sw_lift_drive_by == 1) && (Oi_sw_lift_drive_by_prev == 0))
        {
          next_state = LIFT_STATE_READY_HURDLE;
        }
        else if ((Oi_sw_lift_gather == 1) && (Oi_sw_lift_gather_prev == 0))
        {
          next_state = LIFT_STATE_LOWER_TO_GATHER;
        }
        else
        {
          next_state = LIFT_STATE_HURDLE;
        }
      break;

    case LIFT_STATE_HURDLE_LOWER:
#if DEBUG_LIFT_STATES
        printf("HURDLE_LOWER ");
#endif
        motor_vals.lift_1 = LIFT_DOWN;
        motor_vals.lift_2 = LIFT_DOWN;
        motor_vals.ladder = LADDER_DOWN;
        motor_vals.nest_level = NEST_LEVEL_UP;
        motor_vals.nest_tilt = NEST_TILT_IN;
        motor_vals.accum_tilt = ACCUM_TILT_IN;

        g_man_lift_state = MANU_LIFT_STATE_DOWN;
        //g_man_lift_state = MANU_LIFT_STATE_MID1;

        /* Transition between states */
        if ((Oi_sw_lift_home == 1) && (Oi_sw_lift_home_prev == 0))
        {
          next_state = LIFT_STATE_HOME;
        }
        else if (((Oi_sw_lift_drive_by == 1) && (Oi_sw_lift_drive_by_prev == 0)) ||
                ((Oi_sw_lift_hurdle == 1) && (Oi_sw_lift_hurdle_prev == 0)))
        {
          next_state = LIFT_STATE_READY_HURDLE;
        }
        else if ((Oi_sw_lift_gather == 1) && (Oi_sw_lift_gather_prev == 0))
        {
          next_state = LIFT_STATE_GATHER;
        }
        else
        {
          next_state = LIFT_STATE_HURDLE_LOWER;
        }
      break;

    case LIFT_STATE_DRIVE_BY:
#if DEBUG_LIFT_STATES
        printf("DRIVEBY ");
#endif
        motor_vals.lift_1 = LIFT_UP;
        motor_vals.lift_2 = LIFT_UP;
        motor_vals.ladder = LADDER_UP;
        motor_vals.nest_level = NEST_LEVEL_UP;
        motor_vals.nest_tilt = NEST_TILT_IN;
        motor_vals.accum_tilt = ACCUM_TILT_OUT;

        g_man_lift_state = MANU_LIFT_STATE_UP;

        /* Transition between states */
        if (((Oi_sw_lift_home == 1) && (Oi_sw_lift_home_prev == 0)) ||
            ((Oi_sw_lift_hurdle == 1) && (Oi_sw_lift_hurdle_prev == 0)) ||
            ((Oi_sw_lift_gather == 1) && (Oi_sw_lift_gather_prev == 0)))
        {
          next_state = LIFT_STATE_DRIVE_BY_LOWER_1;
        }
        else
        {
          next_state = LIFT_STATE_DRIVE_BY;
        }
      break;

    case LIFT_STATE_DRIVE_BY_LOWER_1:
#if DEBUG_LIFT_STATES
      printf("DRIVEBY LOW1");
#endif
      motor_vals.lift_1 = LIFT_DOWN;
      motor_vals.lift_2 = LIFT_DOWN;
      motor_vals.ladder = LADDER_UP;
      motor_vals.nest_level = NEST_LEVEL_UP;
      motor_vals.nest_tilt = NEST_TILT_IN;
      motor_vals.accum_tilt = ACCUM_TILT_IN;

      g_man_lift_state = MANU_LIFT_STATE_MID1;

      /* Transition between states */
      if (((Oi_sw_lift_home == 1) && (Oi_sw_lift_home_prev == 0)) ||
          ((Oi_sw_lift_gather == 1) && (Oi_sw_lift_gather_prev == 0)))
      {
        next_state = LIFT_STATE_DRIVE_BY_LOWER_2;
      }
      else if(((Oi_sw_lift_hurdle == 1) && (Oi_sw_lift_hurdle_prev == 0)) || 
              ((Oi_sw_lift_drive_by == 1) && (Oi_sw_lift_drive_by_prev == 0)))
      {
        next_state = LIFT_STATE_READY_HURDLE;
      }
      else
      {
        next_state = LIFT_STATE_DRIVE_BY_LOWER_1;
      }
      break;

    case LIFT_STATE_DRIVE_BY_LOWER_2:
#if DEBUG_LIFT_STATES
      printf("DRIVEBY LOW2");
#endif
      motor_vals.lift_1 = LIFT_DOWN;
      motor_vals.lift_2 = LIFT_DOWN;
      motor_vals.ladder = LADDER_DOWN;
      motor_vals.nest_level = NEST_LEVEL_UP;
      motor_vals.nest_tilt = NEST_TILT_IN;
      motor_vals.accum_tilt = ACCUM_TILT_IN;

      g_man_lift_state = MANU_LIFT_STATE_DOWN;

      /* Transition between states */
      if (((Oi_sw_lift_home == 1) && (Oi_sw_lift_home_prev == 0)) ||
          ((Oi_sw_lift_gather == 1) && (Oi_sw_lift_gather_prev == 0)))
      {
        next_state = LIFT_STATE_HOME;
      }
      else if(((Oi_sw_lift_hurdle == 1) && (Oi_sw_lift_hurdle_prev == 0)) || 
              ((Oi_sw_lift_drive_by == 1) && (Oi_sw_lift_drive_by_prev == 0)))
      {
        next_state = LIFT_STATE_READY_HURDLE;
      }
      else
      {
        next_state = LIFT_STATE_DRIVE_BY_LOWER_2;
      }
      break;

    case LIFT_STATE_LOWER_TO_GATHER:
#if DEBUG_LIFT_STATES
        printf("LOWER_TO_GATHER ");
#endif
        motor_vals.lift_1 = LIFT_DOWN;
        motor_vals.lift_2 = LIFT_DOWN;
        motor_vals.ladder = LADDER_DOWN;
        motor_vals.nest_level = NEST_LEVEL_UP;
        motor_vals.nest_tilt = NEST_TILT_OUT;
        motor_vals.accum_tilt = ACCUM_TILT_IN;

        g_man_lift_state = MANU_LIFT_STATE_DOWN;

        /* Transition between states */
        if ((Oi_sw_lift_home == 1) && (Oi_sw_lift_home_prev == 0))
        {
          next_state = LIFT_STATE_HOME;
        }
        else if ((Oi_sw_lift_hurdle == 1) && (Oi_sw_lift_hurdle_prev == 0))
        {
          next_state = LIFT_STATE_HURDLE;
        }
        else if ((Oi_sw_lift_drive_by == 1) && (Oi_sw_lift_drive_by_prev == 0))
        {
          next_state = LIFT_STATE_READY_HURDLE;
        }
        else if ((Oi_sw_lift_gather == 1) && (Oi_sw_lift_gather_prev == 0))
        {
          next_state = LIFT_STATE_GATHER;
        }
        else
        {
          next_state = LIFT_STATE_LOWER_TO_GATHER;
        }
      break;

    case LIFT_STATE_GATHER_TO_HURDLE:
#if DEBUG_LIFT_STATES
        printf("GATHER_TO_HURDLE ");
#endif
        motor_vals.lift_1 = LIFT_DOWN;
        motor_vals.lift_2 = LIFT_DOWN;
        motor_vals.ladder = LADDER_DOWN;
        motor_vals.nest_level = NEST_LEVEL_UP;
        motor_vals.nest_tilt = NEST_TILT_IN;
        motor_vals.accum_tilt = ACCUM_TILT_IN;

        g_man_lift_state = MANU_LIFT_STATE_DOWN;

        /* Transition between states */
        if ((Oi_sw_lift_home == 1) && (Oi_sw_lift_home_prev == 0))
        {
          next_state = LIFT_STATE_HOME;
        }
        else if ((Oi_sw_lift_hurdle == 1) && (Oi_sw_lift_hurdle_prev == 0))
        {
          next_state = LIFT_STATE_READY_HURDLE;
        }
        else if ((Oi_sw_lift_drive_by == 1) && (Oi_sw_lift_drive_by_prev == 0))
        {
          next_state = LIFT_STATE_READY_HURDLE;
        }
        else if ((Oi_sw_lift_gather == 1) && (Oi_sw_lift_gather_prev == 0))
        {
          next_state = LIFT_STATE_GATHER;
        }
        else
        {
          next_state = LIFT_STATE_GATHER_TO_HURDLE;
        }
      break;

    default:
      next_state = LIFT_STATE_UNKNOWN;
      break;
  }

  first_time_in_state = 0;
  prev_state = g_lift_state;
  g_lift_state = next_state;

}

/*******************************************************************************
* FUNCTION NAME: roller_control
* PURPOSE:       controls the roller 
*******************************************************************************/
void roller_control()
{
  static RollerStateType roller_state = ROLLER_OFF;
  RollerButtonType roller_button;
  /* check for roller state */

#if DEBUG_ROLLER_PRINTS
  printf("rv %3d ", Oi_roller);
#endif

  if(HAT_RANGE_CHECK(Oi_roller, ROLLER_FWD))
  {
    roller_button = ROLLER_BUTTON_FWD;
#if DEBUG_ROLLER_PRINTS
    printf("rb FWD ");
#endif
  }
  else if(HAT_RANGE_CHECK(Oi_roller, ROLLER_REV))
  {
    roller_button = ROLLER_BUTTON_REV;
#if DEBUG_ROLLER_PRINTS
    printf("rb REV ");
#endif
  }
  else if(HAT_RANGE_CHECK(Oi_roller, ROLLER_OFF))
  {
    roller_button = ROLLER_BUTTON_OFF;
#if DEBUG_ROLLER_PRINTS
    printf("rb OFF ");
#endif
  }
  else if(HAT_RANGE_CHECK(Oi_roller, ROLLER_NOTHING))
  {
    roller_button = ROLLER_BUTTON_NOTHING;
#if DEBUG_ROLLER_PRINTS
    printf("rb NOT ");
#endif
  }
  else if(HAT_RANGE_CHECK(Oi_roller, ROLLER_REV_SLOW))
  {
    roller_button = ROLLER_BUTTON_REV_SLOW;
#if DEBUG_ROLLER_PRINTS
    printf("rb REVSLOW ");
#endif
  }
  else
  {
    roller_button = ROLLER_BUTTON_NOTHING;
#if DEBUG_ROLLER_PRINTS
    printf("rb ??? ");
#endif
  }

  if(g_reinit_roller == 1)
  {
    roller_state = ROLLER_STATE_OFF;
    g_reinit_roller = 0;
  }

  switch (roller_state)
  {
    case ROLLER_STATE_OFF:
      motor_vals.roller = ROLLER_SPEED_OFF;

      if (roller_button == ROLLER_BUTTON_FWD)
      {
        roller_state = ROLLER_STATE_FWD_FAST;
      }
      else if (roller_button == ROLLER_BUTTON_REV)
      {
        roller_state = ROLLER_STATE_REV;
      }
      else if (roller_button == ROLLER_BUTTON_REV_SLOW)
      {
        roller_state = ROLLER_STATE_REV_SLOW;
      }
      else
      {
        roller_state = ROLLER_STATE_OFF;
      }

#if DEBUG_ROLLER_PRINTS
      printf("RS OFF ");
#endif
      break;

    case ROLLER_STATE_FWD_SLOW:
      motor_vals.roller = ROLLER_SPEED_FWD_SLOW;

      if (roller_button == ROLLER_BUTTON_FWD)
      {
        roller_state = ROLLER_STATE_FWD_FAST;
      }
      else if (roller_button == ROLLER_BUTTON_REV)
      {
        roller_state = ROLLER_STATE_REV;
      }
      else if (roller_button == ROLLER_BUTTON_OFF)
      {
        roller_state = ROLLER_STATE_OFF;
      }
      else if (roller_button == ROLLER_BUTTON_REV_SLOW)
      {
        roller_state = ROLLER_STATE_REV_SLOW;
      }
      else
      {
        roller_state = ROLLER_STATE_FWD_SLOW;
      }

#if DEBUG_ROLLER_PRINTS
      printf("RS FSL ");
#endif
      break;

    case ROLLER_STATE_FWD_FAST:
      motor_vals.roller = ROLLER_SPEED_FWD_FAST;

      if (roller_button == ROLLER_BUTTON_REV)
      {
        roller_state = ROLLER_STATE_REV;
      }
      else if (roller_button == ROLLER_BUTTON_OFF)
      {
        roller_state = ROLLER_STATE_OFF;
      }
      else if (roller_button == ROLLER_BUTTON_NOTHING)
      {
        roller_state = ROLLER_STATE_FWD_SLOW;
      }
      else if (roller_button == ROLLER_BUTTON_REV_SLOW)
      {
        roller_state = ROLLER_STATE_REV_SLOW;
      }
      else
      {
        roller_state = ROLLER_STATE_FWD_FAST;
      }

#if DEBUG_ROLLER_PRINTS
      printf("RS FST ");
#endif
      break;

    case ROLLER_STATE_REV:
      motor_vals.roller = ROLLER_SPEED_REV;

      if (roller_button == ROLLER_BUTTON_FWD)
      {
        roller_state = ROLLER_STATE_FWD_FAST;
      }
      else if (roller_button == ROLLER_BUTTON_NOTHING)
      {
        roller_state = ROLLER_STATE_OFF;
      }
      else if (roller_button == ROLLER_BUTTON_OFF)
      {
        roller_state = ROLLER_STATE_OFF;
      }
      else if (roller_button == ROLLER_BUTTON_REV_SLOW)
      {
        roller_state = ROLLER_STATE_REV_SLOW;
      }
      else
      {
        roller_state = ROLLER_STATE_REV;
      }

#if DEBUG_ROLLER_PRINTS
      printf("RS REV ");
#endif
      break;

    case ROLLER_STATE_REV_SLOW:
      motor_vals.roller = ROLLER_SPEED_REV_SLOW;

      if (roller_button == ROLLER_BUTTON_FWD)
      {
        roller_state = ROLLER_STATE_FWD_FAST;
      }
      else if (roller_button == ROLLER_BUTTON_REV)
      {
        roller_state = ROLLER_STATE_REV;
      }
      else if (roller_button == ROLLER_BUTTON_NOTHING)
      {
        roller_state = ROLLER_STATE_OFF;
      }
      else if (roller_button == ROLLER_BUTTON_OFF)
      {
        roller_state = ROLLER_STATE_OFF;
      }
      else
      {
        roller_state = ROLLER_STATE_REV_SLOW;
      }

#if DEBUG_ROLLER_PRINTS
      printf("RS REVSLOW ");
#endif
      break;

    default:
      motor_vals.roller = ROLLER_SPEED_OFF;
      roller_state = ROLLER_STATE_OFF;
#if DEBUG_ROLLER_PRINTS
      printf("RS DEF ");
#endif
      break;
  }

#if DEBUG_ROLLER_PRINTS
      printf("MV %4d ", motor_vals.roller);
#endif
}

/*******************************************************************************
* FUNCTION NAME: slapper_control
* PURPOSE:       Puts slapper up when input condition is satisfied
*******************************************************************************/
void slapper_control()
{
  static SlapperType slapper_prev = SLAPPER_DOWN;
  if(Oi_slapper > SLAPPER_THRESH)
  {
    motor_vals.slapper = SLAPPER_UP;
    motor_vals.ladder = LIFT_UP;
  }
  else
  {
    if(slapper_prev == SLAPPER_UP)
    {
      g_reinit_auto_lift = 1;
      motor_vals.ladder = LIFT_UP;
    }
    motor_vals.slapper = SLAPPER_DOWN;
  }
  slapper_prev = motor_vals.slapper;
}

/*******************************************************************************
* FUNCTION NAME: unfold_accumulator
* PURPOSE:       Unfolds the accumulator
*******************************************************************************/
void unfold_accumulator(UINT8 reinit)
{
  static UINT8 unfold_loop = ACCUM_UNFOLD_LOOPS + 1;

  if(unfold_loop < ACCUM_UNFOLD_LOOPS)
  {
    /* tilt out to unfold */
    motor_vals.accum_tilt = ACCUM_TILT_OUT;
    unfold_loop++;
  }
  else
  {
    if(unfold_loop == ACCUM_UNFOLD_LOOPS)
    {
      /* We only want to tilt in the first loop after unfolding
         otherwise we interfere with lift_control */
      motor_vals.accum_tilt = ACCUM_TILT_IN;
      unfold_loop++;
    }

    if(reinit == TRUE)
    {
      /* we got a reinit passed in, so reset the counter */
      unfold_loop = 0;
    }
  }
}


