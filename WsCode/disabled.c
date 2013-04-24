/*******************************************************************************
*
* TITLE:    disabled.c
*
* VERSION:  0.1 (Beta)
*
* DATE:   31-Dec-2007
*
* AUTHOR:   R. Kevin Watson
*           kevinw@jpl.nasa.gov
*
* COMMENTS: This file best viewed with tabs set to four.
*
*           You are free to use this source code for any non-commercial
*           use. Please do not make copies of this source code, modified
*           or un-modified, publicly available on the internet or elsewhere
*           without permission. Thanks.
*
*           Copyright ©2007-2008 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
* Change log:
*
* DATE         REV  DESCRIPTION
* -----------  ---  ----------------------------------------------------------
* 31-Dec-2007  0.1  RKW Original
*
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "ifi_frc.h"
#include "pwm.h"
#include "timers.h"
#include "interrupts.h"
#include "serial_ports.h"
#include "ifi_code.h"
#include "disabled.h"
#include "adc.h"
#include "eeprom.h"
#include "ws_includes.h"
#include "ws_io.h"
#include "ws_cc.h"
#include "ws_calibrate.h"
#include "ws_pid.h"
#include "ws_drive.h"
#include "ws_lift.h"
#include "ws_general.h"
#include "ws_autonomous.h"

extern PotCheckVarsType back_pot_check_vars;
extern PotCheckVarsType front_pot_check_vars;

/*******************************************************************************
*
* FUNCTION:   Disabled_Init()
*
* PURPOSE:    This is where you put code that needs to execute
*         just once at the start of disabled mode.
*
* CALLED FROM:  main() in ifi_frc.c
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   While in this mode, all operator interface data is valid
*         and all robot controller outputs (PWMs and relays) are
*         disabled.
*
*******************************************************************************/
void Disabled_Init(void)
{

}

/*******************************************************************************
*
* FUNCTION:   Disabled()
*
* PURPOSE:    This is where you put code that you want to execute while
*         your robot is disabled. While in autonomous mode, this
*         function is called every 26.2ms when after data is received
*         from the master processor.
*
* CALLED FROM:  main() in ifi_frc.c
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   While in this mode, all operator interface data is valid
*         and all robot controller outputs (PWMs and relays) are
*         disabled.
*
*******************************************************************************/
void Disabled(void)
{
  static int count = 1024;
  static int saved_eeprom = FALSE;
  static INT16 left_accum = 0;
  static INT16 right_accum = 0;
#if NEWLINE_IN_DISABLED
  printf("\r\n");
#endif

  /* Clear the CC loss of communication LEDs */
  Pwm1_green = 0;
  Pwm2_green = 0;
  Relay1_green = 0;
  Relay2_green = 0;

  /* get new data from the CC */
  cc_get_new_data();

  if (cc_data.type == CC_REQ_EEPROM)
  {
    /* just retrieved eeprom from CC, copy it to local copy & parse it out */
    memcpy((void *)rc_eeprom, (void *)cc_data.data.eeprom, NUM_EEPROM_BYTES);
    parse_calibration();
  }

  io_print_oi_inputs();
  io_print_rc_inputs();

  if (Oi_calibrate <= OI_CALIBRATE_ENCODERS)
  {
    /* pot calibration */
    calibrate_pots();
    display_calibration();
  }
  else if (Oi_calibrate >= OI_CALIBRATE_JOYSTICKS)
  {
    display_calibration();
  }
  else
  {
    if (rc_eeprom_dirty == TRUE)
    {
      /* left calibration mode & the local copy was updated, save eeprom to the
         CC, call the write handler, and parse calibration */
      parse_calibration();

      if (saved_eeprom == FALSE)
      {
        write_eeprom_calibration();
        cc_send_eeprom_calibration(NUM_EEPROM_BYTES, rc_eeprom);
        saved_eeprom = TRUE;
      }

      EEPROM_Write_Handler();

      if (EEPROM_Queue_Free_Space() == EEPROM_QUEUE_SIZE)
      {
        /* finished writing eeprom, unset dirty flag */
        rc_eeprom_dirty = FALSE;
      }
    }
    else
    {
      saved_eeprom = FALSE;
    }


    /* calibrate switch is in the middle, debug CC & pot data */
    if ((Oi_sw_encoder_debug == 1) && (cc_data.type == CC_REQ_ENCODER))
    {
      left_accum += cc_data.data.encoder.left;
      right_accum += cc_data.data.encoder.right;

      printf("L: %04d R: %04d Orient %05d B1 %d B2 %d AccumL: %04d AccumR: %04d\r\n",
             cc_data.data.encoder.left, cc_data.data.encoder.right,
             cc_data.data.encoder.orient, cc_data.data.encoder.ball_near_pos,
             cc_data.data.encoder.ball_far_pos, left_accum, right_accum);
    }
    else
    {
      left_accum = 0;
      right_accum = 0;
    }

    if (Oi_sw_crab_pot_debug == 1)
    {
#if USE_DIGITAL_POT
      printf("Fc %4d Fr %4d Fs %d Fa %d Bc %4d Br %4d Bs %d Ba %d\r\n",
             calibration_vals.front_crab.pot_mid, pot_vals.front_crab.raw_val,
             pot_vals.front_crab.spiral_count, pot_vals.front_crab.abs_val,
             calibration_vals.back_crab.pot_mid, pot_vals.back_crab.raw_val,
             pot_vals.back_crab.spiral_count, pot_vals.back_crab.abs_val);
#else
      printf("Fa %d Ba %d\r\n", pot_vals.front_crab.abs_val,
             pot_vals.back_crab.abs_val);
#endif
    }

    /* print out eeprom contents, this causes a code error right now once it
       prints ~500 values */
    if ((Oi_eeprom_debug == 1) && (Oi_eeprom_debug_prev == 0) &&
        (EEPROM_Queue_Free_Space() == EEPROM_QUEUE_SIZE))
    {
      count = 0;
    }

    while (count < 1024)
    {
      if ((count % 8) == 0)
      {
        printf("\r\n");
      }
      printf("ee[%4d]:%3d ", count, EEPROM_Read(count));
      count++;
    }
  }

  auto_lock_in();
  display_auto_data();
  /* Reset the single solenoids so that we don't move when we enable */
  lift_reset();

  /* Set the CC loss of communication LEDs if necessary */
  if (cc_data.type == CC_REQ_UNINIT)
  {
    Pwm1_green = 1;
    Pwm2_green = 1;
    Relay1_green = 1;
    Relay2_green = 1;
  }


  /* reset dead pot check variables */
  front_pot_check_vars.crab_speed_prev = 0;
  front_pot_check_vars.pot_prev = pot_vals.front_crab.abs_val;
  front_pot_check_vars.bad_loop_counter = 0;
  back_pot_check_vars.crab_speed_prev = 0;
  back_pot_check_vars.pot_prev = pot_vals.back_crab.abs_val;
  back_pot_check_vars.bad_loop_counter = 0;

  /* save previous button states */
  oi_swA_byte_prev = rxdata.oi_swA_byte;
  oi_swB_byte_prev = rxdata.oi_swB_byte;
}

/*******************************************************************************
*
* FUNCTION:   Disabled_Spin()
*
* PURPOSE:    While in disabled mode, this function is called
*         continuously between calls to Disabled().
*
* CALLED FROM:  main() in ifi_frc.c
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   While in this mode, all operator interface data is valid
*         and all robot controller outputs (PWMs and relays) are
*         disabled.
*
*******************************************************************************/
void Disabled_Spin(void)
{
  if (process_adc())
  {
    //crab_pos_control();
    //assign_outputs_fast();
#if NEWLINE_IN_DISABLED_SPIN
    printf("\r\n");
#endif
  }
}

