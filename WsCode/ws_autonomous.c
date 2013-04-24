/*******************************************************************************
* FILE NAME: ws_autonomous.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#if 0
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "user_routines.h"
#include "delays.h"       /*defined locally*/
#include "ws_cc.h"
#include "ws_general.h"
#endif
#include "ifi_frc.h"
#include "ws_io.h"
#include "ws_includes.h"

#include "ws_autonomous.h"

extern void feedback_interrupt(void);

static UINT8 s_prog_num = 0;
static UINT8 s_auto_locked_in = FALSE;

ColorType g_auto_color = 0;
DelayType g_auto_delay = 0;
StartPosType g_start_pos = 0;
BallPosType g_ball_pos = 0;


/*******************************************************************************
* FUNCTION NAME: autonomous_init
* PURPOSE:       Initialize the autonomous system...called only once
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void autonomous_init(void)
{
  return;
}


/*******************************************************************************
* FUNCTION NAME: auto_lock_in
* PURPOSE:       Lock in autonomous program & pass info to CC
* CALLED FROM:
* ARGUMENTS:
* RETURNS:
*******************************************************************************/
void auto_lock_in(void)
{

#if DEBUG_AUTO_LOCKIN
  printf("LOCK_SW: %d ", Oi_sw_auto_lockin);
#endif
  if (Oi_sw_auto_lockin == 1)
  {
      s_auto_locked_in = TRUE;
  }
  else
  {
    /* lock in program selector & robot position selectors */
    auto_chooser();
    s_auto_locked_in = FALSE;
  }

  switch(g_ball_pos)
  {
    case BALL_POS_LEFT:
      Pwm1_green = 1;
      Pwm2_green = 0;
      Relay1_green = 0;
      break;
    case BALL_POS_CENTER:
      Pwm1_green = 0;
      Pwm2_green = 1;
      Relay1_green = 0;
      break;
    case BALL_POS_RIGHT:
      Pwm1_green = 0;
      Pwm2_green = 0;
      Relay1_green = 1;
      break;
    default:
      Pwm1_green = 0;
      Pwm2_green = 0;
      Relay1_green = 0;
      break;
  }


#if DEBUG_AUTO_LOCKIN
  printf("LOCKED: %d PROG: %d ", s_auto_locked_in, s_prog_num);
#endif

  return;
}


/*******************************************************************************
* FUNCTION NAME: auto_chooser
* PURPOSE:       set autonomous program, side & starting position;
*                lock in forced auto
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void auto_chooser(void)
{
  //printf("auto_prog %d, auto_pos %d\r", (int)Oi_auto_prog_select, (int)Oi_auto_pos_select);
  /*
     1 - 220
     2 - 199
     3 - 175
     4 - 150
     5 - 131
     6 - 106
     7 - 82
     8 -  62
     9 - 39

  */


  /* read autonomous program dial */
  /* Set s_prog_num based on where Oi_auto_prog_select is */
  /* Be sure to check that the switch is +- AUTO_PROG_OI_DIFF from the value */
  if((Oi_auto_prog_select >= (AUTO_PROG_OI_SEL0 - AUTO_PROG_OI_DIFF)) &&
     (Oi_auto_prog_select <= (AUTO_PROG_OI_SEL0 + AUTO_PROG_OI_DIFF)))
  {
    s_prog_num = 1;
  }
  else if((Oi_auto_prog_select >= (AUTO_PROG_OI_SEL1 - AUTO_PROG_OI_DIFF)) &&
          (Oi_auto_prog_select <= (AUTO_PROG_OI_SEL1 + AUTO_PROG_OI_DIFF)))
  {
    s_prog_num = 2;
  }
  else if((Oi_auto_prog_select >= (AUTO_PROG_OI_SEL2 - AUTO_PROG_OI_DIFF)) &&
          (Oi_auto_prog_select <= (AUTO_PROG_OI_SEL2 + AUTO_PROG_OI_DIFF)))
  {
    s_prog_num = 3;
  }
  else if((Oi_auto_prog_select >= (AUTO_PROG_OI_SEL3 - AUTO_PROG_OI_DIFF)) &&
          (Oi_auto_prog_select <= (AUTO_PROG_OI_SEL3 + AUTO_PROG_OI_DIFF)))
  {
    s_prog_num = 4;
  }
  else if((Oi_auto_prog_select >= (AUTO_PROG_OI_SEL4 - AUTO_PROG_OI_DIFF)) &&
          (Oi_auto_prog_select <= (AUTO_PROG_OI_SEL4 + AUTO_PROG_OI_DIFF)))
  {
    s_prog_num = 5;
  }
  else if((Oi_auto_prog_select >= (AUTO_PROG_OI_SEL5 - AUTO_PROG_OI_DIFF)) &&
          (Oi_auto_prog_select <= (AUTO_PROG_OI_SEL5 + AUTO_PROG_OI_DIFF)))
  {
    s_prog_num = 6;
  }
  else if((Oi_auto_prog_select >= (AUTO_PROG_OI_SEL6 - AUTO_PROG_OI_DIFF)) &&
          (Oi_auto_prog_select <= (AUTO_PROG_OI_SEL6 + AUTO_PROG_OI_DIFF)))
  {
    s_prog_num = 7;
  }
  else if((Oi_auto_prog_select >= (AUTO_PROG_OI_SEL7 - AUTO_PROG_OI_DIFF)) &&
          (Oi_auto_prog_select <= (AUTO_PROG_OI_SEL7 + AUTO_PROG_OI_DIFF)))
  {
    s_prog_num = 8;
  }
  else
  {
    /* default to dead auto program */
    s_prog_num = 1;
  }

  if((Oi_auto_delay_select >= (AUTO_DELAY_OI_0 - AUTO_DELAY_OI_DIFF)) &&
     (Oi_auto_delay_select <= (AUTO_DELAY_OI_0 + AUTO_DELAY_OI_DIFF)))
  {
    g_auto_delay = DELAY_0;
  }
  else if((Oi_auto_delay_select >= (AUTO_DELAY_OI_1 - AUTO_DELAY_OI_DIFF)) &&
          (Oi_auto_delay_select <= (AUTO_DELAY_OI_1 + AUTO_DELAY_OI_DIFF)))
  {
    g_auto_delay = DELAY_1;
  }
  else
  {
    g_auto_delay = DELAY_2;
  }

  if(Oi_auto_color_select == 0)
  {
    g_auto_color = COLOR_RED;
  }
  else
  {
    g_auto_color = COLOR_BLUE;
  }

  if((Oi_auto_pos_select >= (AUTO_POS_OI_LEFT - AUTO_POS_OI_DIFF)) &&
     (Oi_auto_pos_select <= (AUTO_POS_OI_LEFT + AUTO_POS_OI_DIFF)))
  {
    g_start_pos = START_POS_LEFT;
  }
  else if((Oi_auto_pos_select >= (AUTO_POS_OI_CENTER - AUTO_POS_OI_DIFF)) &&
          (Oi_auto_pos_select <= (AUTO_POS_OI_CENTER + AUTO_POS_OI_DIFF)))
  {
    g_start_pos = START_POS_CENTER;
  }
  else
  {
    g_start_pos = START_POS_RIGHT;
  }

  switch(cc_data.data.encoder.ball_near_pos)
  {
    case CC_BALL_POS_UNKNOWN:
      g_ball_pos = BALL_POS_NONE;
      break;
    case CC_BALL_POS_BLUE_RED_EMPTY:
      if(g_auto_color == COLOR_RED)
      {
        g_ball_pos = BALL_POS_CENTER;
      }
      else
      {
        g_ball_pos = BALL_POS_LEFT;
      }
      break;
    case CC_BALL_POS_BLUE_EMPTY_RED:
      if(g_auto_color == COLOR_RED)
      {
        g_ball_pos = BALL_POS_RIGHT;
      }
      else
      {
        g_ball_pos = BALL_POS_LEFT;
      }
      break;
    case CC_BALL_POS_EMPTY_BLUE_RED:
      if(g_auto_color == COLOR_RED)
      {
        g_ball_pos = BALL_POS_RIGHT;
      }
      else
      {
        g_ball_pos = BALL_POS_CENTER;
      }
      break;
    case CC_BALL_POS_RED_BLUE_EMPTY:
      if(g_auto_color == COLOR_RED)
      {
        g_ball_pos = BALL_POS_LEFT;
      }
      else
      {
        g_ball_pos = BALL_POS_CENTER;
      }
      break;
    case CC_BALL_POS_RED_EMPTY_BLUE:
      if(g_auto_color == COLOR_RED)
      {
        g_ball_pos = BALL_POS_LEFT;
      }
      else
      {
        g_ball_pos = BALL_POS_RIGHT;
      }
      break;
    case CC_BALL_POS_EMPTY_RED_BLUE:
      if(g_auto_color == COLOR_RED)
      {
        g_ball_pos = BALL_POS_CENTER;
      }
      else
      {
        g_ball_pos = BALL_POS_RIGHT;
      }
      break;
    case CC_BALL_POS_LEFT:
      g_ball_pos = BALL_POS_LEFT;
      break;
    case CC_BALL_POS_CENTER:
      g_ball_pos = BALL_POS_CENTER;
      break;
    case CC_BALL_POS_RIGHT:
      g_ball_pos = BALL_POS_RIGHT;
      break;
    default:
      g_ball_pos = BALL_POS_NONE;
      break;
  }

  //printf("CC BALL: %d RC BALL: %d ", cc_data.data.encoder.ball_near_pos, g_ball_pos);

#if DEBUG_AUTO_LOCKIN
  printf("ProgSw %d prog %d color %d delay %d pos %d ",
         Oi_auto_prog_select, s_prog_num, g_auto_color, g_auto_delay,
         g_start_pos);
#endif

  /*
   * send autonomous related data to the dashboard
   *
   * User_Byte1:
   * 0xF0 = Auto program #
   * 0x0C = Starting position
   * 0x02 = Color
   * 0x01 = Auto lock-in
   *
   * User_Byte2:
   * 0xC0 = Delay
   *
   */
  User_Byte1 = s_prog_num << 4;
  User_Byte1 |= ((g_start_pos & 0x03) << 2);
  User_Byte1 |= ((g_auto_color & 0x01) << 1);
  User_Byte1 |= ((s_auto_locked_in & 0x01) << 1);

  User_Byte2 = g_auto_delay << 6;

  return;
}

/*******************************************************************************
* FUNCTION NAME: auto_main
* PURPOSE:       main loop for autonomous
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void auto_main()
{
  static UINT8 prog_status = AUTO_PROGRAM_NOT_DONE;

  auto_output_off();

  /* Wildstang autonomous code */
  if(prog_status != AUTO_PROGRAM_DONE)
  {
    prog_status = auto_run_program(s_auto_locked_in, s_prog_num);
    printf("STATUS %d ", prog_status);
  }

  if(prog_status == AUTO_PROGRAM_DONE)
  {
    printf("PROG DONE ");
    auto_output_off();
  }


  return;
}

/*******************************************************************************
* FUNCTION NAME: display_auto_data
* PURPOSE:       display autonomous data on the OI user display
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void display_auto_data(void)
{
  static UINT8 num_blinks = 0;
  static UINT16 num_cycles = 0;
  static UINT8 blink_state = BLINK_ON;
  //Default LEDs to off
  UINT8 do_led_on = 0;

  if ((Oi_calibrate < OI_CALIBRATE_JOYSTICKS) &&
      (Oi_calibrate > OI_CALIBRATE_ENCODERS))
  {

    if(disabled_mode == ROBOT_DISABLED)
    {
      if(s_auto_locked_in == TRUE)
      {
        do_led_on = 1;
        num_cycles = 0;
        num_blinks = 0;
        blink_state = BLINK_ON;
      }
      else
      {
        //  We are disabled and not locked in
        //  blink the program number (1-based)
        switch(blink_state)
        {
          case BLINK_ON:
            if(num_cycles == AUTO_LED_BLINK_END)
            {
              // We've been in the ON part of a blink
              // long enough
              // Move to off state, reset cycle count
              blink_state = BLINK_OFF;
              num_cycles = 0;
            }
            else
            {
              // Turn the LEDs on
              do_led_on = 1;
            }
            break;
          case BLINK_OFF:
            if(num_cycles == AUTO_LED_BLINK_END)
            {
              // We've been in the OFF part of a blink
              // long enough

              if(++num_blinks < (s_prog_num))
              {
                // We haven't blinked the number of times
                // for the current program
                // Reset the cycle count and start another blink
                num_cycles = 0;
                blink_state = BLINK_ON;
                do_led_on = 1;
              }
              else
              {
                // We've blinked the number of times for
                // the current program
                // Reset the number of blinks and move
                // to the delay state
                num_blinks = 0;
                blink_state = BLINK_DELAY;
              }
            }
            break;
          case BLINK_DELAY:
            if(num_cycles == AUTO_LED_BLINK_DELAY_END)
            {
              // We've delayed long enough
              // Start a new blink sequence
              num_cycles = 0;
              blink_state = BLINK_ON;
              do_led_on = 1;
            }
            break;
          default:
            break;
        }
        // Increment the cycle count
        num_cycles++;
      }
    }
    else if(autonomous_mode == AUTO_ENABLED)
    {
      do_led_on = 1;
    }

    /* turn LEDs on when locked in or in auto; blink LEDs otherwise */
    if (do_led_on)
    {
      Relay2_green = ((INT16)s_prog_num & 8) >> 3;
      Switch1_LED = (s_prog_num & 4) >> 2;
      Switch2_LED = (s_prog_num & 2) >> 1;
      Switch3_LED = (s_prog_num & 1);
    }
    else
    {
      Relay2_green = 0;
      Switch1_LED = 0;
      Switch2_LED = 0;
      Switch3_LED = 0;
    }

  }
}




/*******************************************************************************
* FUNCTION NAME: auto_output_off
* PURPOSE:       Sets the outputs to a safe state.  Used for initialization
*                as well as at the end of autonomous
* ARGUMENTS:
* RETURNS:       none
*******************************************************************************/
void auto_output_off()
{
  //set_motor_vals_off();
}

