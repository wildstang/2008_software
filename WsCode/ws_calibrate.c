/*******************************************************************************
* FILE NAME: ws_calibrate.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_frc.h"
#include "eeprom.h"
#include "ws_includes.h"
#include "ws_io.h"
#include "ws_calibrate.h"

/* used to flash "000" after calibrating a pot.  Usually set to 0 to show that
   no calibration was just done.  Set it to 1 to start the blinking. */
static UINT8 sg_cal_ctr = 0;

/*******************************************************************************
* FUNCTION NAME: calibrate_pots
* PURPOSE:       store calibration values for crab in local copy
*
* To calibrate a pot position:
*   - set the pot to the desired position
*   - enable pot calibration mode
*
*******************************************************************************/
void calibrate_pots(void)
{
  static UINT8 eeprom_init_check_flag = FALSE;
  UINT8 bitmask;

  /* only check once if EEPROM has been initialized */
  if (eeprom_init_check_flag == FALSE)
  {
    /* only initialize EEPROM if the 'known' bits are not correct */
    if ((rc_eeprom[ADDR_KNOWN_BYTE1] != CALIBRATE_KNOWN_BYTE_1) ||
        (rc_eeprom[ADDR_KNOWN_BYTE2] != CALIBRATE_KNOWN_BYTE_2))
    {
#ifdef CALIBRATION_DEBUG
      printf("initializing EEPROM  ");
#endif
      rc_eeprom_dirty = TRUE;

      /* set 'known' bytes */
      rc_eeprom[ADDR_KNOWN_BYTE1] = CALIBRATE_KNOWN_BYTE_1;
      rc_eeprom[ADDR_KNOWN_BYTE2] = CALIBRATE_KNOWN_BYTE_2;

      /* initialize bitmasks to 0 */
      rc_eeprom[ADDR_DATA_BITMASK_CRAB] = 0;
    }

    eeprom_init_check_flag = TRUE;
  }

  /* Crab calibration */
  if ((Oi_sw_cal_crab == 1) && (Oi_sw_cal_crab_prev == 0))
  {
#ifdef CALIBRATION_DEBUG
    printf("SET crab ");
#endif
    rc_eeprom_dirty = TRUE;

#if USE_DIGITAL_POT
    if (Oi_crab_y > (127 + CALIBRATE_ZONE))
    {
      /* crab stick is moved forward, calibrate front crab middle position on
         the digital and blue pots */
      SAVE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
          CAL_MASK_F_CRAB_MID, "f_pot %4d ", pot_vals.front_crab.raw_val,
          ADDR_F_CRAB_MID_LO, ADDR_F_CRAB_MID_HI);
    }
    else if (Oi_crab_y < (127 - CALIBRATE_ZONE))
    {
      /* crab stick is moved backward, calibrate back crab middle position on
         the digital and blue pots */
      SAVE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
          CAL_MASK_B_CRAB_MID, "b_pot %4d ", pot_vals.back_crab.raw_val,
          ADDR_B_CRAB_MID_LO, ADDR_B_CRAB_MID_HI);
    }
    /* else stick is in the middle, no calibration */

#else

    if (Oi_drive_y > (127 + CALIBRATE_ZONE))
    {
      /* drive stick is moved forward, calibrate front crab  */
      if (Oi_crab_x < (127 - CALIBRATE_ZONE))
      {
        /* crab stick is moved right, calibrate right position */
        SAVE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
            CAL_MASK_F_CRAB_RIGHT, "f_crab_right %4d ",
            pot_vals.front_crab.abs_val, ADDR_F_CRAB_RIGHT_LO,
            ADDR_F_CRAB_RIGHT_HI);
      }
      else if (Oi_crab_x > (127 + CALIBRATE_ZONE))
      {
        /* crab stick is moved left, calibrate left position */
        SAVE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
            CAL_MASK_F_CRAB_LEFT, "f_crab_left %4d ",
            pot_vals.front_crab.abs_val, ADDR_F_CRAB_LEFT_LO,
            ADDR_F_CRAB_LEFT_HI);
      }
      else
      {
        /* crab stick is neutral, calibrate mid position */
        SAVE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
            CAL_MASK_F_CRAB_MID, "f_crab_mid %4d ",
            pot_vals.front_crab.abs_val, ADDR_F_CRAB_MID_LO,
            ADDR_F_CRAB_MID_HI);
      }
    }
    else if (Oi_drive_y < (127 - CALIBRATE_ZONE))
    {
      /* drive stick is moved backward, calibrate back crab */
      if (Oi_crab_x < (127 - CALIBRATE_ZONE))
      {
        /* crab stick is moved right, calibrate right position */
        SAVE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
            CAL_MASK_B_CRAB_RIGHT, "b_crab_right %4d ",
            pot_vals.back_crab.abs_val, ADDR_B_CRAB_RIGHT_LO,
            ADDR_B_CRAB_RIGHT_HI);
      }
      else if (Oi_crab_x > (127 + CALIBRATE_ZONE))
      {
        /* crab stick is moved left, calibrate left position */
        SAVE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
            CAL_MASK_B_CRAB_LEFT, "b_crab_left %4d ",
            pot_vals.back_crab.abs_val, ADDR_B_CRAB_LEFT_LO,
            ADDR_B_CRAB_LEFT_HI);
      }
      else
      {
        /* crab stick is neutral, calibrate mid position */
        SAVE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
            CAL_MASK_B_CRAB_MID, "b_crab_mid %4d ",
            pot_vals.back_crab.abs_val, ADDR_B_CRAB_MID_LO,
            ADDR_B_CRAB_MID_HI);
      }
    }
    /* else stick is in the middle, no calibration */
#endif

#ifdef CALIBRATION_DEBUG
    printf("\r\n");
#endif
  }

  return;
}


/*******************************************************************************
* FUNCTION NAME: write_eeprom_calibration
* PURPOSE:       write the calibration data stored in the local copy to eeprom
*******************************************************************************/
void write_eeprom_calibration(void)
{
  int i;

  for (i = 0; i < NUM_EEPROM_BYTES; i++)
  {
    EEPROM_Write(i, rc_eeprom[i]);
  }
}


/*******************************************************************************
* FUNCTION NAME: read_eeprom_calibration
* PURPOSE:       read the calibration data stored in eeprom into a local copy
*******************************************************************************/
void read_eeprom_calibration(void)
{
  int i;

  for (i = 0; i < NUM_EEPROM_BYTES; i++)
  {
    rc_eeprom[i] = EEPROM_Read(i);
  }
}


/*******************************************************************************
* FUNCTION NAME: parse_calibration
* PURPOSE:       parse the local copy of eeprom to get the calibration values
*******************************************************************************/
void parse_calibration()
{

  /* initialize calibration to good, the code later will set it to bad if we're
     missing some calibrated values */
  calibration_vals.state = CALIBRATION_GOOD;

  if ((rc_eeprom[ADDR_KNOWN_BYTE1] == CALIBRATE_KNOWN_BYTE_1) &&
      (rc_eeprom[ADDR_KNOWN_BYTE2] == CALIBRATE_KNOWN_BYTE_2))
  {
    /* RC has been used for calibration, we can trust it */
#ifdef CALIBRATION_DEBUG
    printf("EEg");
    printf(" bmc %02x ", rc_eeprom[ADDR_DATA_BITMASK_CRAB]);
#endif

#if USE_DIGITAL_POT
    PARSE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
        CAL_MASK_F_CRAB_MID, "f_pot %4d ", calibration_vals.front_crab.pot_mid,
        ADDR_F_CRAB_MID_LO, ADDR_F_CRAB_MID_HI, calibration_vals.state);
    PARSE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
        CAL_MASK_B_CRAB_MID, "b_pot %4d:", calibration_vals.back_crab.pot_mid,
        ADDR_B_CRAB_MID_LO, ADDR_B_CRAB_MID_HI, calibration_vals.state);

#else

    PARSE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
        CAL_MASK_F_CRAB_MID, "fm %4d ", calibration_vals.front_crab.pot_mid,
        ADDR_F_CRAB_MID_LO, ADDR_F_CRAB_MID_HI, calibration_vals.state);
    PARSE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
        CAL_MASK_F_CRAB_RIGHT, "fr %4d ",
        calibration_vals.front_crab.pot_right, ADDR_F_CRAB_RIGHT_LO,
        ADDR_F_CRAB_RIGHT_HI, calibration_vals.state);
    PARSE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
        CAL_MASK_F_CRAB_LEFT, "fl %4d ",
        calibration_vals.front_crab.pot_left, ADDR_F_CRAB_LEFT_LO,
        ADDR_F_CRAB_LEFT_HI, calibration_vals.state);

    PARSE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
        CAL_MASK_B_CRAB_MID, "bm %4d ", calibration_vals.back_crab.pot_mid,
        ADDR_B_CRAB_MID_LO, ADDR_B_CRAB_MID_HI, calibration_vals.state);
    PARSE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
        CAL_MASK_B_CRAB_RIGHT, "br %4d ",
        calibration_vals.back_crab.pot_right, ADDR_B_CRAB_RIGHT_LO,
        ADDR_B_CRAB_RIGHT_HI, calibration_vals.state);
    PARSE_CALIBRATION_16BIT(rc_eeprom, ADDR_DATA_BITMASK_CRAB,
        CAL_MASK_B_CRAB_LEFT, "bl %4d ",
        calibration_vals.back_crab.pot_left, ADDR_B_CRAB_LEFT_LO,
        ADDR_B_CRAB_LEFT_HI, calibration_vals.state);
#endif
  }
  else
  {
    /* RC has never been given calibration values, set calibration state */
#ifdef CALIBRATION_DEBUG
    printf("EEPROM unknown ");
#endif

    calibration_vals.state = CALIBRATION_BAD_RC_DATA;
  }

#ifdef CALIBRATION_DEBUG
    printf("\r\n");
#endif
}



/*******************************************************************************
* FUNCTION NAME: display_calibration
* PURPOSE:       display joystick vals & pot values for crab & arms on OI
* ARGUMENTS:     none
* RETURNS:       none
*
* To display pot values on the OI:
*   - enable calibration mode
*   - move the joystick axis that corresponds to the pot (crab stick X,
*     big arm Y, big arm X, small arm X) full forward or left (depending on
*     the axis) and allow it to return to the middle
*   - now the OI displays the pot value
*
* To display joystick values on the OI:
*   - enable calibration mode
*   - enable manual crab mode
*   - move the joystick axis that you want to display full forward or left
*     (depending on the axis) and allow it to return to the middle
*   - now the OI displays the X/Y axis of the stick
*
*******************************************************************************/
void display_calibration(void)
{
  static DisplayType display_mode = DISPLAY_NONE;
  UINT16 display_data = 0;

  /* Check if the calibrate switch is in the joystick position. */
  if (Oi_calibrate >= OI_CALIBRATE_JOYSTICKS)
  {
    /* print joystick values when switch is to joystick side */
    /* Check if the joystick has been moved to the calibrate zone */
    if (Oi_drive_x >= (127 + CALIBRATE_ZONE))
    {
      display_mode = DISPLAY_DRIVE_X;
    }
    else if (Oi_drive_y >= (127 + CALIBRATE_ZONE))
    {
      display_mode = DISPLAY_DRIVE_Y;
    }
    else if (Oi_crab_x >= (127 + CALIBRATE_ZONE))
    {
      display_mode = DISPLAY_CRAB_X;
    }
    else if (Oi_crab_y >= (127 + CALIBRATE_ZONE))
    {
      display_mode = DISPLAY_CRAB_Y;
    }
  }
  else
  {
    /* print pot/encoder values when switch is to calibrate encoder side */
    if (Oi_crab_y > (127 + CALIBRATE_ZONE))
    {
      display_mode = DISPLAY_POT_FRONT_CRAB;
    }
    else if (Oi_crab_y < (127 - CALIBRATE_ZONE))
    {
      display_mode = DISPLAY_POT_BACK_CRAB;
    }
    else if (Oi_sw_landing_gear == 1)
    {
      display_mode = DISPLAY_ORIENT;
    }
    else if ((Oi_sw_turbo == 1) && (Oi_crab_x > (127 + CALIBRATE_ZONE)))
    {
      display_mode = DISPLAY_ENCODER_L;
    }
    else if ((Oi_sw_turbo == 1) && (Oi_crab_x < (127 - CALIBRATE_ZONE)))
    {
      display_mode = DISPLAY_ENCODER_R;
    }
  }

  /* flash "000" if a pot was just calibrated */
  if ((sg_cal_ctr == 0) ||
      ((sg_cal_ctr >= 5) && (sg_cal_ctr <= 10)) ||
      ((sg_cal_ctr >= 15) && (sg_cal_ctr <= 20)))
  {
    switch (display_mode)
    {
      case DISPLAY_DRIVE_X:
        display_data = Oi_drive_x;
        break;

      case DISPLAY_DRIVE_Y:
        display_data = Oi_drive_y;
        break;

      case DISPLAY_CRAB_X:
        display_data = Oi_crab_x;
        break;

      case DISPLAY_CRAB_Y:
        display_data = Oi_crab_y;
        break;

      case DISPLAY_POT_FRONT_CRAB:
#if USE_DIGITAL_POT
        display_data = pot_vals.front_crab.raw_val;
#else
        display_data = pot_vals.front_crab.abs_val;
#endif
        break;

      case DISPLAY_POT_BACK_CRAB:
#if USE_DIGITAL_POT
        display_data = pot_vals.front_crab.raw_val;
#else
        display_data = pot_vals.front_crab.abs_val;
#endif
        break;

      case DISPLAY_ORIENT:
        display_data = cc_data.data.encoder.orient >> 8;
        break;

      case DISPLAY_ENCODER_L:
        display_data = cc_data.data.encoder.left;
        break;

      case DISPLAY_ENCODER_R:
        display_data = cc_data.data.encoder.right;
        break;

      default:
        break;
    }

    if (sg_cal_ctr != 0)
    {
      sg_cal_ctr++;
    }
  }
  else
  {
    display_data = 0;
    sg_cal_ctr++;

    if (sg_cal_ctr > 25)
    {
      sg_cal_ctr = 0;
    }
  }

  display_oi_data((UINT8)display_data, DISPLAY_DATA_CALIBRATE);

  /*
   * send calibration related data to the dashboard
   *
   * User_Byte3:
   * 0xF0 = Calibration number
   * 0x0F = Calibration value (lower nibble of upper byte)
   *
   * User_Byte4:
   * 0xFF = Calibration value (lower byte)
   *
   */
  User_Byte3 = (display_mode << 4) | (UINT8)((display_data >> 8) & 0x0F);
  User_Byte4 = (UINT8)(display_data & 0xFF);

  return;
}

