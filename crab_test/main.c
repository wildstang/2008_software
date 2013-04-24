/*******************************************************************************
* FILE NAME: main.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the main program loop.
*
* USAGE:
*  You should not need to modify this file.
*  Note the different loop speed for the two routines:
*     Process_Data_From_Master_uP
*     Process_Data_From_Local_IO
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ws_includes.h"
#include "ws_pid.h"
#include "user_routines.h"
#include "ws_io.h"
#include "ws_drive_input.h"
#include "ws_arm.h"
#include "ws_calibrate.h"
#include "ws_cc.h"
#include "ws_autonomous.h"

tx_data_record txdata;          /* DO NOT CHANGE! */
rx_data_record rxdata;          /* DO NOT CHANGE! */
packed_struct statusflag;       /* DO NOT CHANGE! */
RcButtonsType oi_swA_byte_prev;
RcButtonsType oi_swB_byte_prev;

MotorValsType       motor_vals;
CalibrationValsType calibration_vals;
EncoderValsType     g_encoder_vals;
UINT8 g_cc_encoder_ret_val;
UINT8 g_crab_arc_ccw;
/*******************************************************************************
* FUNCTION NAME: main
* PURPOSE:       Main program function.
* CALLED FROM:   ifi_startup.c
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT DELETE THIS FUNCTION 
*******************************************************************************/
void main (void)
{
#ifdef UNCHANGEABLE_DEFINITION_AREA
  IFI_Initialization ();        /* DO NOT CHANGE! */
#endif

  User_Initialization();        /* You edit this in user_routines.c */

  statusflag.NEW_SPI_DATA = 0;  /* DO NOT CHANGE! */ 

  retrieve_calibration();

  /* initialize subsystems */
  crab_init();
  //arm_init();

  /* initialize motor_vals so motors are off & pneumatics are in default
     position */
  set_motor_vals_off();

  while (1)   /* This loop will repeat indefinitely. */
  {
#ifdef _SIMULATOR
    statusflag.NEW_SPI_DATA = 1;
#endif

    /*
     * 26.2ms loop area
     * I'm slow!  I only execute every 26.2ms because that's how fast the
     * Master uP gives me data.
     */
    if (statusflag.NEW_SPI_DATA)
    {
      /* get the encoder vals & orientation from CC */
      //g_cc_encoder_ret_val = cc_get_encoder_vals(&g_encoder_vals);

      Process_Data_From_Master_uP();

#if 0
      if (autonomous_mode == AUTO_DISABLED)
      {
        auto_lock_in();
      }

      /* set auto-lock-in LEDs */
      display_auto_data();

      if (autonomous_mode == AUTO_ENABLED)
      {
        auto_main();
      }
#endif

      /* save state of OI buttons for next loop */
      oi_swA_byte_prev = rxdata.oi_swA_byte;
      oi_swB_byte_prev = rxdata.oi_swB_byte;
    }
    Process_Data_From_Local_IO();     /* I'm fast!  I execute every loop.*/
  }
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
