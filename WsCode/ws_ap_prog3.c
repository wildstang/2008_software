#include <stdio.h>
#include "ifi_frc.h"
#include "ws_io.h"
#include "ws_includes.h"
#include "ws_autonomous.h"
#include "ws_autoprog.h"
#include "ws_pid.h"
#include "ws_drive.h"
#include "ws_lift.h"

#define WAIT_FOR_IR_LOOPS    160

/* Auto Program 3
 * Slapper
 */
UINT8 ap_prog3(UINT8 num_lines, UINT8 allow_slap, UINT8 random_slap)
{
  static UINT8 prog_state = AP_SLPR_STATE_WAIT_FOR_BALL_POS;

  static UINT16 timeout_thresh = TO_NONE;
  static UINT16 timeout_loop = 0;
  static UINT16 loop_count = 0;
  UINT8 prog_ret = AUTO_PROGRAM_NOT_DONE;
  UINT8 delay_loops = 0;

  /* Declare static variables used in the state machine.
     In general, use the following types:
        speeds - UINT8
        headings - INT16
        crab brads - UINT8
        accelerations - UINT8
   */
  static INT8 approach_brad = 0;
  static UINT16 approach_dist = 0;
  static UINT8 approach_speed = 0;
  static UINT8 approach_accel = 0;
  static UINT8 accel_approach_speed = 127;

  static UINT16 point_attack_dist = 0;
  static UINT8 attack_speed = 0;
  static INT16 attack_brad = 0;
  static UINT16 attack_dist = 0;
  static UINT16 attack_done_dist = 0;

  static INT8 clear_wall_brad = 0;
  static UINT8 clear_wall_speed = 127;
  static UINT16 clear_wall_dist = 0;

  static UINT16 prep_line2_dist = 0;

  static UINT8 line2_x_speed = 127;
  static UINT8 line2_y_speed = 127;
  static INT16 line2_rotation = 0;
  static INT16 drive_short_line2_dist = 0;
  static UINT8 drive_short_line2_speed = 0;

  static UINT8 finish_line2_x_speed = 127;
  static UINT8 finish_line2_y_speed = 127;
  static INT16 finish_line2_rotation = 0;

  static UINT16 line3_dist = 0;
  static UINT16 finish_line3_dist = 0;

  static UINT8 line4_x_speed = 127;
  static UINT8 line4_y_speed = 127;
  static INT16 line4_rotation = 0;

  static INT16 drive_short_line4_dist = 0;
  static UINT8 drive_short_line4_speed = 0;

  static UINT8 finish_line4_x_speed = 127;
  static UINT8 finish_line4_y_speed = 127;
  static INT16 finish_line4_rotation = 0;

  static UINT8  use_landing_gear = 1;

  static INT16 initial_gyro = 0;

  /* Set this to AP_SLPR_STATE_DONE when the
     whole program is to be run */
  UINT8 exit_early_state = AP_SLPR_STATE_DONE;

  /* We always want theta correction on */
  Oi_sw_theta_correct = 1;

  /*
  Oi_sw_lift_disabled = 1;
  */
  /* Useful for hardcoding CC info */
  /*
  num_lines = 4;
  cc_data.type = CC_REQ_ENCODER;
  cc_data.data.encoder.ball_near_pos = CC_BALL_POS_BLUE_EMPTY_RED;
  g_auto_color = COLOR_RED;
  g_start_pos = START_POS_LEFT;
  */

  switch(prog_state)
  {
    case AP_SLPR_STATE_WAIT_FOR_BALL_POS:
      /* Wait in this state until we get a valid ball position
         from the CC.  If the timeout is reached, move on as if
         the ball was in the center position, but don't slap.
       */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_WAIT_FOR_BALL_POS ");

      if(loop_count < WAIT_FOR_IR_LOOPS)
      {
        if(g_ball_pos != BALL_POS_NONE)
        {
          /* The ball position has been determined, move on to initialize */
          printf("CHOSE BALL %d ", g_ball_pos);

          loop_count = 0;
          timeout_loop = 0;

          prog_state = AP_SLPR_STATE_INIT_FROM_BALL_POS;

        }
        else
        {
          if(cc_data.type == CC_REQ_ENCODER)
          {
            /* Only read the data if the CC data is good */

            /* Store off the initial gyro heading. */
            initial_gyro = cc_data.data.encoder.orient;

            /* The CC doesn't know what color we are, so
               determine the ball position from the combination of our color
               and where the balls are.

               However, this was designed for using the camera to locate the
               balls.  Since we didn't go that route, the only cases that are
               hit are the hardcoded left, right, center cases
            */
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
            timeout_loop = 0;
            prog_state = AP_SLPR_STATE_WAIT_FOR_BALL_POS;
          }
          else
          {
            /* CC Data was bad...we can't trust the ball position */
            g_ball_pos = BALL_POS_NONE;
          }
        }
        loop_count++;
      }
      else
      {
        /* No ball position entered....
           Drive down the center but don't slap
        */
        loop_count = 0;
        timeout_loop = 0;
        g_ball_pos = BALL_POS_CENTER;
        allow_slap = 0;
        use_landing_gear = 1;
        prog_state = AP_SLPR_STATE_WAIT_FOR_BALL_POS;
        printf("TIMED OUT %d ", g_ball_pos);
      }
      break;
    case AP_SLPR_STATE_INIT_FROM_BALL_POS:
      /* This state is processed once.  It takes the combination
         of the starting position and the ball position and initializes
         the variables used in the drive engine */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_INIT_FROM_BALL_POS ");

      if(g_start_pos == START_POS_CENTER)
      {
        printf(" - C - ");

        if(allow_slap)
        {
          Oi_slapper = 255;
        }

        if(use_landing_gear == 1)
        {
          /* When using the landing gear we want differential theta correction
             We also disable crab feedback so that the front wheels don't
             oscillate */
          Oi_sw_landing_gear = 1;
          Oi_sw_crab_manu_mode = 1;
          g_use_differential_theta = 1;
        }

        if(g_ball_pos == BALL_POS_CENTER)
        {
          /* START: CENTER
             BALL:  CENTER
          */

          /* Approach point is straight ahead */
          approach_brad = 0;
          approach_dist = 1600;
          approach_speed = 127 + 127;
          approach_accel = 13;

          /* Since the ball is in front of us, the attack point
             is the same as the approach point */
          attack_dist = 0;
          attack_brad = 0;

          /* Drive through the ball */
          attack_done_dist = 1275;

          /* No need to clear the wall in this case */
          clear_wall_brad = 0;
          clear_wall_speed = 127 + 75;
          clear_wall_dist = 0;

          /* No need to prep for the turn */
          prep_line2_dist = 0;

          /* Begin the turn */
          line2_x_speed = 127 + 79;
          line2_y_speed = 127 + 127;
          line2_rotation = -475;

          /* This is only here in case we need to adjust
             the distance we drive between turns */
          drive_short_line2_dist = 1;
          drive_short_line2_speed = 127 + 127;

          /* Finish the turn */
          finish_line2_x_speed = 127 + 79;
          finish_line2_y_speed = 127 + 127;
          finish_line2_rotation = -600;

          /* Drive the backstretch.  The two steps are probably
             not needed anymore and can probably be replaced with one
             long drive.  It was here originally in case we wanted to stop in
             the third quadrant */
          line3_dist = 500;
          finish_line3_dist = 1900;

          /* Begin the turn at the fourth line */
          line4_x_speed = 127 + 79;
          line4_y_speed = 127 + 127;
          line4_rotation = -350;

          /* Drive across the fourth line */
          drive_short_line4_dist = 325;
          drive_short_line4_speed = 127 + 100;

          /* Finish the fourth line */
          finish_line4_x_speed = 127 + 63;
          finish_line4_y_speed = 127 + 63;
          finish_line4_rotation = -200;

        }
        else if(g_ball_pos == BALL_POS_LEFT)
        {
          /* START: CENTER
             BALL:  LEFT
          */

          /* Approach point is straight ahead */
          approach_brad = 0;
          approach_dist = 1400;
          approach_speed = 127 + 127;
          approach_accel = 13;

          /* Point to attack will use the attack brad value below
             and use a really slow speed to drive to the distance needed
             to in order to crab to the left.  There was too much momentum
             to deal with, so we resorted to backing off the speed. */
          point_attack_dist = 400;

          /* The short slow speed drive breaks up the momentum that we had. */
          attack_speed = 127 + 30;
          attack_dist = 1;
          attack_brad = 63;

          /* The wheels are pointed forward at this point and we drive through
             the ball */
          attack_done_dist = 600;

          /* Make a quick jog to the right to get off of the wall before the turn */
          clear_wall_brad = 240;
          clear_wall_speed = 127 + 100;
          clear_wall_dist = 350;

          /* Continue driving forward */
          prep_line2_dist = 200;

          /* Begin the turn */
          line2_x_speed = 127 + 79;
          line2_y_speed = 127 + 127;
          line2_rotation = -475;

          /* This is only here in case we need to adjust
             the distance we drive between turns */
          drive_short_line2_dist = 1;
          drive_short_line2_speed = 127 + 127;

          /* Finish the turn */
          finish_line2_x_speed = 127 + 85;
          finish_line2_y_speed = 127 + 127;
          finish_line2_rotation = -600;

          /* Drive the backstretch.  The two steps are probably
             not needed anymore and can probably be replaced with one
             long drive.  It was here originally in case we wanted to stop in
             the third quadrant */
          line3_dist = 1300;
          finish_line3_dist = 1200;

          /* Begin the turn at the fourth line */
          line4_x_speed = 127 + 79;
          line4_y_speed = 127 + 127;
          line4_rotation = -350;

          /* Drive across the fourth line */
          drive_short_line4_dist = 325;
          drive_short_line4_speed = 127 + 100;

          /* Finish the fourth line */
          finish_line4_x_speed = 127 + 63;
          finish_line4_y_speed = 127 + 63;
          finish_line4_rotation = -200;
        }
        else
        {
          /* START: CENTER
             BALL:  RIGHT
          */

          /* Approach point is straight ahead */
          approach_brad = 0;
          approach_dist = 1300;
          approach_speed = 127 + 127;
          approach_accel = 13;

          /* Point to attack will use the attack brad value below
             and use a really slow speed to drive to the distance needed
             to in order to crab to the left.  There was too much momentum
             to deal with, so we resorted to backing off the speed. */
          point_attack_dist = 360;

          /* The short slow speed drive breaks up the momentum that we had. */
          attack_speed = 127 + 30;
          attack_dist = 1;
          attack_brad = 191;

          /* The wheels are pointed forward at this point and we drive through
             the ball */
          attack_done_dist = 600;

          /* Angle to the left to get off of the wall before the turn */
          clear_wall_brad = 15;
          clear_wall_speed = 127 + 100;
          clear_wall_dist = 425;

          /* Continue driving forward */
          prep_line2_dist = 200;

          /* Begin the turn */
          line2_x_speed = 127 + 79;
          line2_y_speed = 127 + 127;
          line2_rotation = -475;

          /* Drive to the next turn */
          drive_short_line2_dist = 200;
          drive_short_line2_speed = 127 + 127;

          /* Finish the turn */
          finish_line2_x_speed = 127 + 79;
          finish_line2_y_speed = 127 + 127;
          finish_line2_rotation = -600;

          /* Drive the backstretch.  The two steps are probably
             not needed anymore and can probably be replaced with one
             long drive.  It was here originally in case we wanted to stop in
             the third quadrant */
          line3_dist = 1300;
          finish_line3_dist = 1200;

          /* Begin the turn at the fourth line */
          line4_x_speed = 127 + 79;
          line4_y_speed = 127 + 127;
          line4_rotation = -350;

          /* Drive across the fourth line */
          drive_short_line4_dist = 325;
          drive_short_line4_speed = 127 + 100;

          /* Finish the fourth line */
          finish_line4_x_speed = 127 + 63;
          finish_line4_y_speed = 127 + 63;
          finish_line4_rotation = -200;
        }

        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DELAY;
      }
      else if(g_start_pos == START_POS_LEFT)
      {
        /* When we're on the left, drive a lap without
           slapping (not really tested */
        printf(" - L - ");
        approach_brad = 0;
        approach_dist = 2900;
        approach_speed = 127 + 127;
        approach_accel = 0;
        use_landing_gear = 0;

        attack_dist = 0;
        attack_brad = 0;

        attack_done_dist = 0;

        clear_wall_brad = 0;
        clear_wall_speed = 127 + 75;

        clear_wall_dist = 0;

        prep_line2_dist = 0;

        line2_x_speed = 127 + 79;
        line2_y_speed = 127 + 127;
        line2_rotation = -475;

        drive_short_line2_dist = 100;
        drive_short_line2_speed = 127 + 127;

        finish_line2_x_speed = 127 + 79;
        finish_line2_y_speed = 127 + 127;
        finish_line2_rotation = -530;

        line3_dist = 500;
        finish_line3_dist = 1900;

        line4_x_speed = 127 + 79;
        line4_y_speed = 127 + 127;
        line4_rotation = -350;

        drive_short_line4_dist = 325;
        drive_short_line4_speed = 127 + 100;

        finish_line4_x_speed = 127 + 63;
        finish_line4_y_speed = 127 + 63;
        finish_line4_rotation = -200;

        use_landing_gear = 0;
        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DELAY;
      }
      else
      {
        printf("INVALID STARTING POSITION <%d> ", g_start_pos);
        timeout_loop = 0;
        /* The delay state will at least unfold us */
        prog_state = AP_SLPR_STATE_DELAY;
      }
      break;

    case AP_SLPR_STATE_DELAY:
      /* This state has two actions
         - Raise the slapper and lift
         - Add additional delay as needed */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DELAY %d loops <%d> ", delay_loops, loop_count);

      if(allow_slap)
      {
        Oi_slapper = 255;
      }

      if(use_landing_gear == 1)
      {
        Oi_sw_landing_gear = 1;
        Oi_sw_crab_manu_mode = 1;
        g_use_differential_theta = 1;
      }


      /* Choose the delay time */
      if(g_auto_delay == DELAY_1)
      {
        delay_loops = 40;
      }
      else if(g_auto_delay == DELAY_2)
      {
        delay_loops = 80;
      }
      else
      {
        delay_loops = 0;
      }

      if(loop_count >= delay_loops)
      {
        /* Delay is done */
        loop_count = 0;
        timeout_loop = 0;

        prog_state = AP_SLPR_STATE_DRIVE_TO_APPROACH;
      }
      else
      {
        /* Delay as necessary */
        loop_count++;
      }

      if(g_start_pos == START_POS_RIGHT )
      {
        /* We can exit since we don't run
           from the right side */
        prog_state = AP_SLPR_STATE_DONE;
      }
      break;


    case AP_SLPR_STATE_DRIVE_TO_APPROACH:
      /* This state is used to drive to the approach point.
         It has a timeout of 300 loops in case we run into something
      */
      timeout_thresh = 300;
      printf("AP_SLPR_STATE_DRIVE_TO_APPROACH ");

      if(allow_slap == 1)
      {
        Oi_slapper = 255;
      }

      if(use_landing_gear == 1)
      {
        Oi_sw_landing_gear = 1;
        Oi_sw_crab_manu_mode = 1;
        g_use_differential_theta = 1;
      }

      /* Get joystick values for the desired approach angle */
      convert_brads_to_joystick(approach_brad, &Oi_crab_x, &Oi_crab_y);

      /* Accelerate as needed.  This could probably reuse the function from
         the drive code */
      if(approach_accel > 0)
      {
        /* This only handles the forward case since we don't drive backwards */
        if(accel_approach_speed < (approach_speed - approach_accel))
        {
          accel_approach_speed += approach_accel;
        }
        else
        {
          accel_approach_speed = approach_speed;
        }
      }
      else
      {
        accel_approach_speed = approach_speed;
      }

      if(drive_by_ticks(127, accel_approach_speed, approach_dist) == AUTON_DRIVE_DONE)
      {
        /* Drive until we're at the approach point */
        timeout_loop = 0;
        loop_count = 0;

        if(attack_brad != 0)
        {
          /* The attack point isn't directly in front of us, so we need to point */
          prog_state = AP_SLPR_STATE_POINT_TO_ATTACK;
        }
        else
        {
          prog_state =  AP_SLPR_STATE_DRIVE_TO_ATTACK;
        }
      }

      break;


    case AP_SLPR_STATE_POINT_TO_ATTACK:
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_POINT_TO_ATTACK ");
      if(allow_slap == 1)
      {
        Oi_slapper = 255;
      }

      /* Get the joystick values to point to the attack point */
      convert_brads_to_joystick(attack_brad, &Oi_crab_x, &Oi_crab_y);

      if(drive_by_ticks(127, attack_speed , point_attack_dist) == AUTON_DRIVE_DONE)
      {
        /* Drive until we're pointed at the attack point */
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DRIVE_TO_ATTACK;
      }
      break;

    case AP_SLPR_STATE_DRIVE_TO_ATTACK:
      /* This state drives to the attack point */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_TO_ATTACK ");

      if(allow_slap == 1)
      {
        Oi_slapper = 255;
      }

      g_forced_theta = initial_gyro;
      g_use_forced_theta = 1;

      convert_brads_to_joystick(attack_brad, &Oi_crab_x, &Oi_crab_y);
      if(drive_by_ticks(127, attack_speed, attack_dist) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;

        if(attack_brad != 0)
        {
          prog_state = AP_SLPR_STATE_POINT_TO_ATTACK_DONE;
        }
        else
        {
          prog_state = AP_SLPR_STATE_DRIVE_TO_ATTACK_DONE;
        }
      }

      break;

    case AP_SLPR_STATE_POINT_TO_ATTACK_DONE:
      /* This state is used to point the wheels downfield */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_POINT_TO_ATTACK_DONE ");

      if(allow_slap == 1)
      {
        Oi_slapper = 255;
      }

      convert_brads_to_joystick(0, &Oi_crab_x, &Oi_crab_y);
      loop_count++;
      if(attack_brad != 0)
      {
        /* This short delay allows the wheels to point forward */
        if(loop_count > 20)
        {
          timeout_loop = 0;
          prog_state = AP_SLPR_STATE_DRIVE_TO_ATTACK_DONE;
        }
      }
      else
      {
        prog_state = AP_SLPR_STATE_DRIVE_TO_ATTACK_DONE;
      }
      break;

    case AP_SLPR_STATE_DRIVE_TO_ATTACK_DONE:
      /* Drive through the ball */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_TO_ATTACK_DONE ");

      if(allow_slap == 1)
      {
        Oi_slapper = 255;
      }

      /* Force the theta correction to use the initial heading in case we skewed */
      g_forced_theta = initial_gyro;
      g_use_forced_theta = 1;

      if(drive_by_ticks(127, 127 + 127, attack_done_dist) == AUTON_DRIVE_DONE)
      {
        /* We've reached the attack done point */
        loop_count = 0;
        timeout_loop = 0;
        if(clear_wall_brad != 0)
        {
          prog_state = AP_SLPR_STATE_DRIVE_TO_CLEAR_WALL;
        }
        else
        {
          prog_state = AP_SLPR_STATE_DRIVE_TO_CLEAR_WALL;
        }
      }

      break;

    case AP_SLPR_STATE_DRIVE_TO_CLEAR_WALL:
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_TO_CLEAR_WALL ");

      if(allow_slap == 1)
      {
        Oi_slapper = 255;
      }

      /* Force the theta correction to use the initial heading in case we skewed */
      g_forced_theta = initial_gyro;
      g_use_forced_theta = 1;

      /* Crab off the wall */
      convert_brads_to_joystick(clear_wall_brad, &Oi_crab_x, &Oi_crab_y);
      if(drive_by_ticks(127, clear_wall_speed, clear_wall_dist) == AUTON_DRIVE_DONE)
      {
        /* Move on to the next state */
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DRIVE_TO_PREP_FOR_LINE2;
      }

      break;

    case AP_SLPR_STATE_DRIVE_TO_PREP_FOR_LINE2:
      /* This state is used to drive further down field, usually in
         order to straighten out after a crab maneuver */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_TO_PREP_FOR_LINE2 ");

      if((allow_slap == 1) && (random_slap == 1))
      {
        Oi_slapper = 255;
      }

      convert_brads_to_joystick(0, &Oi_crab_x, &Oi_crab_y);
      if(drive_by_ticks(127, 127 + 75, prep_line2_dist) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DRIVE_TO_LINE2;
      }

      if(num_lines == 1)
      {
        /* Call it quits after 1 lines */
        prog_state = AP_SLPR_STATE_DONE;
      }
      break;

    case AP_SLPR_STATE_DRIVE_TO_LINE2:
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_TO_LINE2 ");

      if((allow_slap == 1) && (random_slap == 1))
      {
        Oi_slapper = 255;
      }

      /* Do about a quarter turn in monster mode */
      Oi_sw_monster_mode = 1;
      if(drive_by_gyro(line2_x_speed, line2_y_speed, line2_rotation) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DRIVE_SHORT_LINE2;
      }
      break;

    case AP_SLPR_STATE_DRIVE_SHORT_LINE2:
      /* Drive the short line between the quarter turns */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_SHORT_LINE2 ");

      if((allow_slap == 1) && (random_slap == 1))
      {
        Oi_slapper = 255;
      }

      convert_brads_to_joystick(0, &Oi_crab_x, &Oi_crab_y);

      if(drive_by_ticks(127, drive_short_line2_speed, drive_short_line2_dist) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DRIVE_TO_FINISH_LINE2;
      }
      break;

    case AP_SLPR_STATE_DRIVE_TO_FINISH_LINE2:
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_TO_FINISH_LINE2 ");

      if((allow_slap == 1) && (random_slap == 1))
      {
        Oi_slapper = 255;
      }

      /* Do about a quarter turn in monster mode */
      Oi_sw_monster_mode = 1;
      if(drive_by_gyro(finish_line2_x_speed, finish_line2_y_speed,
                       finish_line2_rotation) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DRIVE_TO_LINE3;
      }

      break;

    case AP_SLPR_STATE_DRIVE_TO_LINE3:
      /* Start the drive down the backstretch */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_TO_LINE3 ");

      /* Use a theta heading that is 180 deg. from the initial.  This will
         help to stay straight */
      g_forced_theta = initial_gyro - GYRO_180_DIFF;
      g_use_forced_theta = 1;

      if((allow_slap == 1) && (random_slap == 1))
      {
        Oi_slapper = 255;
      }

      convert_brads_to_joystick(0, &Oi_crab_x, &Oi_crab_y);
      if(loop_count < 10)
      {
        /* This small delay allows the wheels to straighten coming out of the turn */
        loop_count++;
      }
      else
      {
        if(drive_by_ticks(127, 127 + 127, line3_dist) == AUTON_DRIVE_DONE)
        {
          loop_count = 0;
          timeout_loop = 0;
          prog_state = AP_SLPR_STATE_DRIVE_TO_FINISH_LINE3;
        }
      }

      if(num_lines == 2)
      {
        /* Call it quits after 2 lines */
        prog_state = AP_SLPR_STATE_DONE;
      }

      break;

    case AP_SLPR_STATE_DRIVE_TO_FINISH_LINE3:
      /* Keep driving to clear the third line */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_TO_FINISH_LINE3 ");

      /* Use a theta heading that is 180 deg. from the initial.  This will
         help to stay straight */
      g_forced_theta = initial_gyro - GYRO_180_DIFF;
      g_use_forced_theta = 1;

      if((allow_slap == 1) && (random_slap == 1))
      {
        Oi_slapper = 255;
      }

      convert_brads_to_joystick(0, &Oi_crab_x, &Oi_crab_y);
      if(drive_by_ticks(127, 127 + 75, finish_line3_dist) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;
        g_use_forced_theta = 0;
        prog_state = AP_SLPR_STATE_DRIVE_TO_LINE4;
      }

      break;

    case AP_SLPR_STATE_DRIVE_TO_LINE4:
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_TO_LINE4 ");

      /* Begin the monster turn around the fourth line */
      Oi_sw_monster_mode = 1;
      if(drive_by_gyro(line4_x_speed, line4_y_speed, line4_rotation) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DRIVE_SHORT_LINE4;
      }

      if(num_lines == 3)
      {
        /* Call it quits after 3 lines */
        prog_state = AP_SLPR_STATE_DONE;
      }
      break;

    case AP_SLPR_STATE_DRIVE_SHORT_LINE4:
      /* Drive the shord distance between the two quarter turns */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_SHORT_LINE4 ");

      if((allow_slap == 1) && (random_slap == 1))
      {
        Oi_slapper = 255;
      }

      convert_brads_to_joystick(0, &Oi_crab_x, &Oi_crab_y);

      if(drive_by_ticks(127, drive_short_line4_speed, drive_short_line4_dist) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DRIVE_TO_FINISH_LINE4;
      }
      break;

    case AP_SLPR_STATE_DRIVE_TO_FINISH_LINE4:
      /* Finish the last turn */
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DRIVE_TO_FINISH_LINE4 ");

      Oi_sw_monster_mode = 1;
      if(drive_by_gyro(finish_line4_x_speed, finish_line4_y_speed,
                       finish_line4_rotation) == AUTON_DRIVE_DONE)
      {
        loop_count = 0;
        timeout_loop = 0;
        prog_state = AP_SLPR_STATE_DONE;
      }

      break;

    case AP_SLPR_STATE_DONE:
      timeout_thresh = TO_NONE;
      printf("AP_SLPR_STATE_DONE ");
      prog_state = AP_SLPR_STATE_DONE;
      prog_ret = AUTO_PROGRAM_DONE;
      break;

    default:
      timeout_thresh = TO_NONE;
      printf("INVALID...TERM ");
      prog_state = AP_SLPR_STATE_DONE;
      prog_ret = AUTO_PROGRAM_DONE;
    break;
  }

  /* Each state has the ability to set the timeout threshold.  This block will
     monitor how long a current state has been running and will bail out if the
     state has reached its timeout */
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
      prog_state = AP_SLPR_STATE_DONE;
    }
  }


  /* Setting the early exit state allows us to quickly change how much of the
     state machine to run */
  if(exit_early_state == prog_state)
  {
    prog_state = AP_SLPR_STATE_DONE;
  }

  /* Display the ball we're going for at the OI */
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
      break;
  }
  return prog_ret;
}

