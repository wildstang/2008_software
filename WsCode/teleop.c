/*******************************************************************************
*
* TITLE:    teleop.c
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
#include "adc.h"
#include "gyro.h"
#include "teleop.h"
#include "ws_includes.h"
#include "ws_io.h"
#include "ws_pid.h"
#include "ws_cc.h"
#include "ws_drive.h"
#include "ws_lift.h"
#include "ws_calibrate.h"
#include "ws_general.h"


/*******************************************************************************
*
* FUNCTION:   Initialization()
*
* PURPOSE:    This function is called once when the robot controller
*         is cold or warm booted. You should initialize your code
*         here.
*
* CALLED FROM:  main() in ifi_frc.c
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:
*
*******************************************************************************/
void Initialization(void)
{
  // Setup the digital I/O pins. Use "INPUT" to setup the pin
  // as an input and "OUTPUT" to setup the pin as an output.
  digital_io_01 = INPUT;
  digital_io_02 = OUTPUT;
  digital_io_03 = INPUT;
  digital_io_04 = INPUT;
  digital_io_05 = INPUT;
  digital_io_06 = INPUT;
  digital_io_07 = INPUT;
  digital_io_08 = INPUT;
  digital_io_09 = INPUT;
  digital_io_10 = INPUT;
  digital_io_11 = INPUT;
  digital_io_12 = INPUT;
  digital_io_13 = INPUT;
  digital_io_14 = INPUT;
  digital_io_15 = INPUT;
  digital_io_16 = INPUT;
  digital_io_17 = INPUT;
  digital_io_18 = INPUT;


  // Initialize the digital outputs. If the pin is configured
  // as an input above, it doesn't matter what state you
  // initialize it to here.
  rc_dig_out01 = 0;
  rc_dig_out02 = 0;
  rc_dig_out03 = 0;
  rc_dig_out04 = 0;
  rc_dig_out05 = 0;
  rc_dig_out06 = 0;
  rc_dig_out07 = 0;
  rc_dig_out08 = 0;
  rc_dig_out09 = 0;
  rc_dig_out10 = 0;
  rc_dig_out11 = 0;
  rc_dig_out12 = 0;
  rc_dig_out13 = 0;
  rc_dig_out14 = 0;
  rc_dig_out15 = 0;
  rc_dig_out16 = 0;
  rc_dig_out17 = 0;
  rc_dig_out18 = 0;

  // initialize timers and external interrupts here
  // (see timers.c/.h and interrupts.c/.h)

#if USE_DIGITAL_POT
  pot_vals.front_crab.is_init = 0;
  pot_vals.back_crab.is_init = 0;
#endif

  cc_data.type = CC_REQ_UNINIT;
  crab_init();

  Initialize_ADC();

  printf("IFI User Processor Initialized ...\r\n");
}

/*******************************************************************************
*
* FUNCTION:   Teleop_Init()
*
* PURPOSE:    This is where you put code that needs to execute
*         just once at the start of teleoperation mode.
*
* CALLED FROM:  main() in ifi_frc.c
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   While in this mode, all operator interface data is valid
*         and all robot controller outputs are enabled.
*
*******************************************************************************/
void Teleop_Init(void)
{

}

/*******************************************************************************
*
* FUNCTION:   Teleop()
*
* PURPOSE:    This is where you put code that you want to execute while
*         your robot is in teleoperation mode. While in teleoperation
*         mode, this function is called every 26.2ms after new data
*         is received from the master processor.
*
* CALLED FROM:  main() in ifi_frc.c
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   While in this mode, all operator interface data is valid
*         and all robot controller outputs are enabled.
*
*******************************************************************************/
void Teleop(void)
{
#if NEWLINE_IN_TELEOP
  printf("\r\n");
#endif

  /* Clear the CC loss of communication LEDs */
  Pwm1_green = 0;
  Pwm2_green = 0;
  Relay1_green = 0;
  Relay2_green = 0;

  g_use_forced_theta = 0;

  /* get new data from the CC */
  cc_get_new_data();

  if (cc_data.type == CC_REQ_EEPROM)
  {
    /* just retrieved eeprom from CC, copy it to local copy & parse it out */
    memcpy((void *)rc_eeprom, (void *)cc_data.data.eeprom, NUM_EEPROM_BYTES);
    parse_calibration();
  }

  if ((Oi_calibrate <= OI_CALIBRATE_ENCODERS) ||
      (Oi_calibrate >= OI_CALIBRATE_JOYSTICKS))
  {
    /* show pot and joystick calibration */
    display_calibration();
  }

  drive_control(TRUE);
  lift_control();
  pump_control();

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
* FUNCTION:   Teleop_Spin()
*
* PURPOSE:    While in teleoperation mode, this function is called
*         continuously between calls to Teleop().
*
* CALLED FROM:  main() in ifi_frc.c
*
* PARAMETERS:   None
*
* RETURNS:    Nothing
*
* COMMENTS:   While in this mode, all operator interface data is valid
*         and all robot controller outputs are enabled.
*
*******************************************************************************/
void Teleop_Spin(void)
{
  if (process_adc())
  {
    crab_pos_control();
    assign_outputs_fast();
#if NEWLINE_IN_TELEOP_SPIN
    printf("\r\n");
#endif
  }
}

