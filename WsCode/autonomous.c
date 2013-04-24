/*******************************************************************************
*
* TITLE:    autonomous.c
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
#include "autonomous.h"
#include "ws_includes.h"
#include "ws_io.h"
#include "ws_pid.h"
#include "ws_cc.h"
#include "ws_drive.h"
#include "ws_lift.h"
#include "ws_general.h"
#include "ws_autonomous.h"
#include "ws_calibrate.h"

/*******************************************************************************
*
* FUNCTION:   Autonomous_Init()
*
* PURPOSE:    This is where you put code that you want to execute
*         just once at the start of autonomous mode.
*
* CALLED FROM:  main() in ifi_frc.c
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   While in this mode, all operator interface data is invalid
*         (i.e., all analog inputs are set to 127 and digital inputs
*         are set to zero) and all robot controller outputs (PWMs and
*         relays) are enabled.
*
*******************************************************************************/
void Autonomous_Init(void)
{

}

/*******************************************************************************
*
* FUNCTION:   Autonomous()
*
* PURPOSE:    This is where you put code that you want to execute while
*         your robot is in autonomous mode. While in autonomous mode,
*         this function is called every 26.2ms after new data is
*         received from the master processor.
*
* CALLED FROM:  main() in ifi_frc.c
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   While in this mode, all operator interface data is invalid
*         (i.e., all analog inputs are set to 127 and digital inputs
*         are set to zero) and all robot controller outputs (PWMs and
*         relays) are enabled.
*
*******************************************************************************/
void Autonomous(void)
{
#if NEWLINE_IN_AUTON
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

  g_use_forced_theta = 0;

  auto_main();

  drive_control(FALSE);

  lift_control();
  roller_control();
  slapper_control();
  pump_control();

  display_auto_data();
  assign_outputs_slow();

  /* Set the CC loss of communication LEDs if necessary */
  if (cc_data.type == CC_REQ_UNINIT)
  {
    Pwm1_green = 1;
    Pwm2_green = 1;
    Relay1_green = 1;
    Relay2_green = 1;
  }

  /* save previous button states */
  oi_swA_byte_prev = rxdata.oi_swA_byte;
  oi_swB_byte_prev = rxdata.oi_swB_byte;
}

/*******************************************************************************
*
* FUNCTION:   Autonomous_Spin()
*
* PURPOSE:    While in autonomous mode, this function is called
*         continuously between calls to Autonomous().
*
* CALLED FROM:  main() in ifi_frc.c
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   While in this mode, all operator interface data is invalid
*         (i.e., all analog inputs are set to 127 and digital inputs
*         are set to zero) and all robot controller outputs (PWMs and
*         relays) are enabled.
*
*******************************************************************************/
void Autonomous_Spin(void)
{
#if NEWLINE_IN_AUTON_SPIN
    printf("\r\n");
#endif
  if (process_adc())
  {
#if AUTON_ADC_RESULT_PRINTS
#if USE_DIGITAL_POT
    printf("fr %4d fs %d br %4d bs %d ", pot_vals.front_crab.raw_val,
        pot_vals.front_crab.spiral_count, pot_vals.back_crab.raw_val,
        pot_vals.back_crab.spiral_count);
#else
    printf("fa %4d ba %4d ", pot_vals.front_crab.abs_val,
           pot_vals.back_crab.abs_val);
#endif
#endif
    motor_vals.front_crab = 0;
    motor_vals.back_crab = 0;

    crab_pos_control();
    assign_outputs_fast();
  }
}
