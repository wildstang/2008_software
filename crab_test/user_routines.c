/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/

#include <stdio.h>
#include <timers.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "eeprom.h"
#include "pwm.h"
#include "ws_pid.h"
#include "ws_io.h"
#include "ws_includes.h"
#include "ws_drive_input.h"
#include "ws_general.h"
#include "ws_calibrate.h"
#include "ws_feedback.h"
#include "ws_cc.h"

/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

  /* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_17 = digital_io_18 = INPUT;
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

  /* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_02 = OUTPUT;    /* Example - Not used in Default Code. */

  /* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

  /* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

  /* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
   *   Choose from these parameters for PWM 13-16 respectively:
   *     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...)
   *     USER_CCP - User can use PWM pin as digital I/O or CCP pin.
   */
  Setup_PWM_Output_Type(USER_CCP,USER_CCP,USER_CCP,USER_CCP);

  /* Example: The following would generate a 40KHz PWM with a 50% duty cycle
     on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */

  Init_Serial_Port_One();
  Init_Serial_Port_Two();
  stdout_serial_port = SERIAL_PORT_ONE;

  Initialize_PWM();

  /* initialize the timer that periodically calls the feedback routine(s),
     the timer will fire every 10ms */
#if 0
  OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_8 &
             T1_OSC1EN_OFF & T1_SYNC_EXT_OFF);
  WriteTimer1(53035);
#else
  OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_8 &
             T1_OSC1EN_OFF & T1_SYNC_EXT_OFF);
  WriteTimer1(0);
#endif

  Putdata(&txdata);            /* DO NOT CHANGE! */

//  ***  IFI Code Starts Here***
//
//  Serial_Driver_Initialize();
//
//  printf("IFI 2006 User Processor Initialized ...\r");  /* Optional - Print initialization message. */

  User_Proc_Is_Ready(); /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
  /* This is a simulated button */
  Oi_sw_crab_arc_ccw = 0;

  Getdata(&rxdata);

  io_print_oi_inputs();
  io_print_rc_inputs();

#if 0
  if (g_cc_encoder_ret_val == CC_SUCCESS)
  {
    printf("LF: %04d RF: %04d LB: %04d RB: %04d Orient %05d Sonar %05u\r",
        g_encoder_vals.left_front, g_encoder_vals.right_front,
        g_encoder_vals.left_back, g_encoder_vals.right_back,
        g_encoder_vals.orient, g_encoder_vals.sonar);
  }
#endif

  if (autonomous_mode == AUTO_DISABLED)
  {
    if ((Oi_calibrate <= OI_CALIBRATE_ENCODERS) && (user_display_mode == 0))
    {
      /* calibrate encoders & not user display mode: turn all motors off */
      set_motor_vals_off();

      /* flash the lights on the OI */
      if ((rxdata.packet_num % 8) < 4)
      {
        display_oi_data(255, DISPLAY_DATA_CALIBRATE);
        Switch1_LED = 1;
        Switch2_LED = 1;
        Switch3_LED = 1;
      }
      else
      {
        display_oi_data(0, DISPLAY_DATA_CALIBRATE);
        Switch1_LED = 0;
        Switch2_LED = 0;
        Switch3_LED = 0;
      }

      if (Oi_sw_encoder_debug == 1)
      {
        if (g_cc_encoder_ret_val == CC_SUCCESS)
        {
          printf("LF: %04d RF: %04d LB: %04d RB: %04d Orient %05d Sonar %05u\r",
                 g_encoder_vals.left_front, g_encoder_vals.right_front,
                 g_encoder_vals.left_back, g_encoder_vals.right_back,
                 g_encoder_vals.orient, g_encoder_vals.sonar);
        }
      }
    }
    else
    {
      if (Oi_calibrate <= OI_CALIBRATE_ENCODERS)
      {
        /* pot calibration */
        EEPROM_Write_Handler();

        /* only set calibrtion if disabled */
        if (disabled_mode == ROBOT_DISABLED)
        {
          calibrate_pots();
        }
        display_calibration();

        if (EEPROM_Queue_Free_Space() == EEPROM_QUEUE_SIZE)
        {
          /* queue is empty so it's safe to read the calibration values */
          retrieve_calibration();
        }
      }
      else if (Oi_calibrate >= OI_CALIBRATE_JOYSTICKS)
      {
        display_calibration();
      }

      /* scale joysticks */
      Oi_drive_x = joystick_scaling(Oi_drive_x, DRIVE_STICK_X_MIN,
                                    DRIVE_STICK_X_MIDDLE, DRIVE_STICK_X_MAX);
      Oi_drive_y = joystick_scaling(Oi_drive_y, DRIVE_STICK_Y_MIN,
                                    DRIVE_STICK_Y_MIDDLE, DRIVE_STICK_Y_MAX);
      Oi_crab_x = joystick_scaling(Oi_crab_x, CRAB_STICK_X_MIN,
                                   CRAB_STICK_X_MIDDLE, CRAB_STICK_X_MAX);

      /* call functions that control robot by setting motor_vals */
      drive_preproc();
      crab_stick_input();
      drive_stick_input(TRUE);
      feedback_interrupt();
      deploy_ramp_n_tower();

#if DEBUGGING_THETA
      printf("\r");
#endif


      pump_control();
    }

    //assign_outputs_fast();
    //Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

    assign_outputs_slow();
    Putdata(&txdata);
  }
}

