/*******************************************************************************
* FILE NAME: ifi_code.c
*
* DESCRIPTION:
*  This file contains some useful functions that you can call in your program.
*
*******************************************************************************/

#include <adc.h>
#include <delays.h>
#include "ifi_frc.h"
#include "ifi_code.h"

/*******************************************************************************
* FUNCTION NAME: Get_Analog_Value
* PURPOSE:       Reads the analog voltage on an A/D port and returns the
*                10-bit value read stored in an unsigned int.
* CALLED FROM:
* ARGUMENTS:     
*      Argument         Type        IO   Description
*     -----------   -------------   --   -----------
*     ADC_channel       alias       I    alias found in ifi_aliases.h
* RETURNS:       unsigned int
*******************************************************************************/
unsigned int Get_Analog_Value (unsigned char ADC_channel)
{
  unsigned int result;

#if defined(__18F8722)
  OpenADC( ADC_FOSC_RC & ADC_RIGHT_JUST & ADC_0_TAD,
		 ADC_channel & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, 15);
#else
  OpenADC( ADC_FOSC_RC & ADC_RIGHT_JUST & ADC_16ANA,
		 ADC_channel & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS );
#endif
  Delay10TCYx(10);
  ConvertADC();
  while(BusyADC());
  ReadADC();
  CloseADC();
  result = (int) ADRESH << 8 | ADRESL;
  return result;
}


/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{
 /*---------- Analog Inputs (Joysticks) to PWM Outputs-----------------------
  *--------------------------------------------------------------------------
  *   This maps the joystick axes to specific PWM outputs.
  */
  pwm01 = p1_y;
  pwm02 = p2_y;   
  pwm03 = p3_y;   
  pwm04 = p4_y;   
  pwm05 = p1_x;  
  pwm06 = p2_x;   
  pwm07 = p3_x;   
  pwm08 = p4_x;   
  pwm09 = p1_wheel;
  pwm10 = p2_wheel;   
  pwm11 = p3_wheel;   
  pwm12 = p4_wheel;   
  
 /*---------- 1 Joystick Drive ----------------------------------------------
  *--------------------------------------------------------------------------
  *  This code mixes the Y and X axis on Port 1 to allow one joystick drive. 
  *  Joystick forward  = Robot forward
  *  Joystick backward = Robot backward
  *  Joystick right    = Robot rotates right
  *  Joystick left     = Robot rotates left
  *  Connect the right drive motors to PWM13 and/or PWM14 on the RC.
  *  Connect the left  drive motors to PWM15 and/or PWM16 on the RC.
  */  
  p1_x = 255 - p1_y;
  p1_y = 255 - pwm05;

  pwm13 = pwm14 = Limit_Mix(2000 + p1_y + p1_x - 127);
  pwm15 = pwm16 = Limit_Mix(2000 + p1_y - p1_x + 127);
  
 /*---------- Buttons to Relays----------------------------------------------
  *--------------------------------------------------------------------------
  *  This default code maps the joystick buttons to specific relay outputs.  
  *  Relays 1 and 2 use limit switches to stop the movement in one direction.
  *  The & used below is the C symbol for AND                                
  */
  relay1_fwd = p1_sw_trig & rc_dig_in01;  /* FWD only if switch1 is not closed. */
  relay1_rev = p1_sw_top  & rc_dig_in02;  /* REV only if switch2 is not closed. */
  relay2_fwd = p2_sw_trig & rc_dig_in03;  /* FWD only if switch3 is not closed. */
  relay2_rev = p2_sw_top  & rc_dig_in04;  /* REV only if switch4 is not closed. */
  relay3_fwd = p3_sw_trig;
  relay3_rev = p3_sw_top;
  relay4_fwd = p4_sw_trig;
  relay4_rev = p4_sw_top;
  relay5_fwd = p1_sw_aux1;
  relay5_rev = p1_sw_aux2;
  relay6_fwd = p3_sw_aux1;
  relay6_rev = p3_sw_aux2;
  relay7_fwd = p4_sw_aux1;
  relay7_rev = p4_sw_aux2;
  relay8_fwd = !rc_dig_in18;  /* Power pump only if pressure switch is off. */
  relay8_rev = 0;
  
  /*---------- PWM outputs Limited by Limit Switches  ------------------------*/
  
  Limit_Switch_Max(rc_dig_in05, &pwm03);
  Limit_Switch_Min(rc_dig_in06, &pwm03);
  Limit_Switch_Max(rc_dig_in07, &pwm04);
  Limit_Switch_Min(rc_dig_in08, &pwm04);
  Limit_Switch_Max(rc_dig_in09, &pwm09);
  Limit_Switch_Min(rc_dig_in10, &pwm09);
  Limit_Switch_Max(rc_dig_in11, &pwm10);
  Limit_Switch_Min(rc_dig_in12, &pwm10);
  Limit_Switch_Max(rc_dig_in13, &pwm11);
  Limit_Switch_Min(rc_dig_in14, &pwm11);
  Limit_Switch_Max(rc_dig_in15, &pwm12);
  Limit_Switch_Min(rc_dig_in16, &pwm12);
}

/*******************************************************************************
*
*	FUNCTION:		Update_OI_LEDs()
*
*	PURPOSE:		Updates the state of the various user
*					defined	LEDs on the operator interface.	
* 
*	CALLED FROM:
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
* 
*	COMMENTS:
*
*******************************************************************************/
void Update_OI_LEDs(void)
{
	if(user_display_mode == 0)
	{
		// update the "PWM 1" LED
    	if(p1_y >= 0 && p1_y <= 56)
    	{						// joystick is in full reverse position
      		Pwm1_green = 0;		// turn PWM1 green LED off
      		Pwm1_red = 1;		// turn PWM1 red LED on
		}
		else if(p1_y >= 125 && p1_y <= 129)
		{						// joystick is in neutral position
			Pwm1_green = 1;		// turn PWM1 green LED on
			Pwm1_red = 1;		// turn PWM1 red LED on
		}
		else if(p1_y >= 216 && p1_y <= 255)
		{						// joystick is in full forward position
			Pwm1_green = 1;		// turn PWM1 green LED on
			Pwm1_red = 0;		// turn PWM1 red LED off
		}
		else
		{						// in either forward or reverse position
			Pwm1_green = 0;		// turn PWM1 green LED off
			Pwm1_red = 0;		// turn PWM1 red LED off
		}

		// update the "PWM 2" LED
		if(p2_y >= 0 && p2_y <= 56)
		{						// joystick is in full reverse position
			Pwm2_green = 0;		// turn pwm2 green LED off
			Pwm2_red = 1;		// turn pwm2 red LED on
		}
		else if(p2_y >= 125 && p2_y <= 129)
		{						// joystick is in neutral position
			Pwm2_green = 1;		// turn PWM2 green LED on
			Pwm2_red = 1;		// turn PWM2 red LED on
		}
		else if(p2_y >= 216 && p2_y <= 255)
		{						// joystick is in full forward position
			Pwm2_green = 1;		// turn PWM2 green LED on
			Pwm2_red = 0;		// turn PWM2 red LED off
		}
		else
		{						// in either forward or reverse position
			Pwm2_green = 0;		// turn PWM2 green LED off
			Pwm2_red = 0;		// turn PWM2 red LED off
		}

		// update the "Relay 1" and "Relay 2" LEDs
		Relay1_green = relay1_fwd;	// LED is on when Relay 1 is FWD
		Relay1_red = relay1_rev;	// LED is on when Relay 1 is REV
		Relay2_green = relay2_fwd;	// LED is on when Relay 2 is FWD
		Relay2_red = relay2_rev;	// LED is on when Relay 2 is REV

		// update the "Switch 1", "Switch 2" and "Switch 3" LEDs
		Switch1_LED = !(int)rc_dig_in01;
		Switch2_LED = !(int)rc_dig_in02;
		Switch3_LED = !(int)rc_dig_in03;
	}
  	else  /* User Mode is On - displays data in OI 4-digit display*/
	{
		User_Mode_byte = backup_voltage * 10;
	}
}   


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Max
* PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Min
* PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value < 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}