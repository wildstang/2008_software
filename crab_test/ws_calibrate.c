/*******************************************************************************
* FILE NAME: ws_calibrate.c
*
* DESCRIPTION:
*
* USAGE:
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "eeprom.h"
#include "ws_includes.h"
#include "delays.h"       /*defined locally*/

#include "ws_calibrate.h"
#include "ws_pid.h"
#include "ws_drive_input.h"
#include "ws_arm.h"
#include "ws_io.h"

#define MACRO_TEST

extern UINT16 g_cur_elbow;

/* used to flash "000" after calibrating a pot.  Usually set to 0 to show that
   no calibration was just done.  Set it to 1 to start the blinking. */
static UINT8 sg_cal_ctr = 0;

/*******************************************************************************
* FUNCTION NAME: calibrate_pots
* PURPOSE:       store calibration values for crab & arms in EEPROM
* ARGUMENTS:     none
* RETURNS:       none
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
  UINT16 pot_val;


  /* only check once if EEPROM has been initialized */
  if (eeprom_init_check_flag == FALSE)
  {
    /* only initialize EEPROM if the 'known' bits are not correct */
    if ((EEPROM_Read(ADDR_KNOWN_BYTE1) != CALIBRATE_KNOWN_BYTE_1) &&
        (EEPROM_Read(ADDR_KNOWN_BYTE2) != CALIBRATE_KNOWN_BYTE_2))
    {
#ifdef CALIBRATION_DEBUG
      printf("initializing EEPROM  ");
#endif
      /* initialize bitmasks to 0 */
      EEPROM_Write(ADDR_DATA_BITMASK_CRAB, 0);
      EEPROM_Write(ADDR_DATA_BITMASK_ARM, 0);
      EEPROM_Write(ADDR_DATA_BITMASK_3, 0);
      EEPROM_Write(ADDR_DATA_BITMASK_4, 0);

      /* set 'known' bytes */
      EEPROM_Write(ADDR_KNOWN_BYTE1, CALIBRATE_KNOWN_BYTE_1);
      EEPROM_Write(ADDR_KNOWN_BYTE2, CALIBRATE_KNOWN_BYTE_2);
    }

    eeprom_init_check_flag = TRUE;
  }

  /* Crab calibration */
  if ((Oi_sw_cal_crab == 1) && (Oi_sw_cal_crab_prev == 0))
  {
#ifdef CALIBRATION_DEBUG
    printf("SET crab ");
#endif

    if (Oi_crab_y > (127 + CALIBRATE_ZONE))
    {
      /* crab stick is moved forward, calibrate the front crab */
      pot_val = GET_ANALOG_VALUE_SHIFT(Analog_in_front_crab);

      if ((Oi_crab_x < (127 + CALIBRATE_ZONE)) &&
          (Oi_crab_x > (127 - CALIBRATE_ZONE)))
      {
        /* stick is in middle, calibrate wheels straight forward */
        WRITE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_F_CRAB_MID,
                               " fm %3d", pot_val, ADDR_F_CRAB_MID);
      }
      else if (Oi_crab_x > (127 + CALIBRATE_ZONE))
      {
        /* stick is left, calibrate wheels to the left */
        WRITE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_F_CRAB_LEFT,
                               " fl %3d", pot_val, ADDR_F_CRAB_LEFT);
      }
      else
      {
        /* stick is right, calibrate wheels to the right */
        WRITE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_F_CRAB_RIGHT,
                               " fr %3d", pot_val, ADDR_F_CRAB_RIGHT);
      }
    }
    else if (Oi_crab_y < (127 - CALIBRATE_ZONE))
    {
      /* crab stick is moved backward, calibrate the back crab */
      pot_val = GET_ANALOG_VALUE_SHIFT(Analog_in_back_crab);

      if ((Oi_crab_x < (127 + CALIBRATE_ZONE)) &&
          (Oi_crab_x > (127 - CALIBRATE_ZONE)))
      {
        /* stick is in middle, calibrate wheels straight forward */
        WRITE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_B_CRAB_MID,
                               " bm %3d", pot_val, ADDR_B_CRAB_MID);
      }
      else if (Oi_crab_x > (127 + CALIBRATE_ZONE))
      {
        /* stick is left, calibrate wheels to the left */
        WRITE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_B_CRAB_LEFT,
                               " bl %3d", pot_val, ADDR_B_CRAB_LEFT);
      }
      else
      {
        /* stick is right, calibrate wheels to the right */
        WRITE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_B_CRAB_RIGHT,
                               " br %3d", pot_val, ADDR_B_CRAB_RIGHT);
      }
    }
    /* else stick is in the middle, no calibration */

#ifdef CALIBRATION_DEBUG
    printf("\r");
#endif
  }

  return;
}



/*******************************************************************************
* FUNCTION NAME: retrieve_calibration
* PURPOSE:       retrieve calibration values for crab & arms from EEPROM
* ARGUMENTS:     none
* RETURNS:       none
*******************************************************************************/
void retrieve_calibration()
{
  UINT8 verify1, verify2;
  UINT8 bitmask;

  verify1 = EEPROM_Read(ADDR_KNOWN_BYTE1);
  verify2 = EEPROM_Read(ADDR_KNOWN_BYTE2);

  if ((verify1 == CALIBRATE_KNOWN_BYTE_1) &&
      (verify2 == CALIBRATE_KNOWN_BYTE_2))
  {
    /* RC has been used for calibration, we can trust it */
#ifdef CALIBRATION_DEBUG
    printf("EEg");
    bitmask = EEPROM_Read(ADDR_DATA_BITMASK_CRAB);
    printf(" bmc %02x", bitmask);
#endif

    RETRIEVE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_F_CRAB_LEFT,
                              " fc %d:", calibration_vals.front_crab.left,
                              ADDR_F_CRAB_LEFT, DEFAULT_CRAB_90_LEFT);
    RETRIEVE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_F_CRAB_MID,
                              "%d:", calibration_vals.front_crab.mid,
                              ADDR_F_CRAB_MID, DEFAULT_CRAB_MIDDLE);
    RETRIEVE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_F_CRAB_RIGHT,
                              "%d", calibration_vals.front_crab.right,
                              ADDR_F_CRAB_RIGHT, DEFAULT_CRAB_90_RIGHT);
    RETRIEVE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_B_CRAB_LEFT,
                              " bc %d:", calibration_vals.back_crab.left,
                              ADDR_B_CRAB_LEFT, DEFAULT_CRAB_90_LEFT);
    RETRIEVE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_B_CRAB_MID,
                              "%d:", calibration_vals.back_crab.mid,
                              ADDR_B_CRAB_MID, DEFAULT_CRAB_MIDDLE);
    RETRIEVE_CALIBRATION_8BIT(ADDR_DATA_BITMASK_CRAB, CAL_MASK_B_CRAB_RIGHT,
                              "%d", calibration_vals.back_crab.right,
                              ADDR_B_CRAB_RIGHT, DEFAULT_CRAB_90_RIGHT);

#ifdef CALIBRATION_DEBUG
    bitmask = EEPROM_Read(ADDR_DATA_BITMASK_ARM);
    printf(" bma %02x ", bitmask);
#endif

    RETRIEVE_CALIBRATION_16BIT(ADDR_DATA_BITMASK_ARM, CAL_MASK_SHOULDER_F,
                              " sf %d", calibration_vals.shoulder_front,
                              ADDR_SHOULDER_F_LO, ADDR_SHOULDER_F_HI,
                              DEFAULT_SHOULDER_FRONT);
    RETRIEVE_CALIBRATION_16BIT(ADDR_DATA_BITMASK_ARM, CAL_MASK_SHOULDER_B,
                              " sb %d", calibration_vals.shoulder_back,
                              ADDR_SHOULDER_B_LO, ADDR_SHOULDER_B_HI,
                              DEFAULT_SHOULDER_BACK);
    RETRIEVE_CALIBRATION_16BIT(ADDR_DATA_BITMASK_ARM, CAL_MASK_ELBOW_TOP,
                              " et %d", calibration_vals.elbow_top,
                              ADDR_ELBOW_TOP_LO, ADDR_ELBOW_TOP_HI,
                              DEFAULT_ELBOW_TOP);
    RETRIEVE_CALIBRATION_16BIT(ADDR_DATA_BITMASK_ARM, CAL_MASK_ELBOW_BOT,
                              " eb %d", calibration_vals.elbow_bottom,
                              ADDR_ELBOW_BOT_LO, ADDR_ELBOW_BOT_HI,
                              DEFAULT_ELBOW_BOTTOM);
    RETRIEVE_CALIBRATION_16BIT(ADDR_DATA_BITMASK_ARM, CAL_MASK_ROTATE_L,
                              " rl %d", calibration_vals.rotate_left,
                              ADDR_ROTATE_L_LO, ADDR_ROTATE_L_HI,
                              DEFAULT_ROTATE_LEFT);
    RETRIEVE_CALIBRATION_16BIT(ADDR_DATA_BITMASK_ARM, CAL_MASK_ROTATE_R,
                              " rr %d", calibration_vals.rotate_right,
                              ADDR_ROTATE_R_LO, ADDR_ROTATE_R_HI,
                              DEFAULT_ROTATE_RIGHT);

    RETRIEVE_CALIBRATION_16BIT(ADDR_DATA_BITMASK_ARM, CAL_MASK_ARM_HORIZ,
                              " hs %d", calibration_vals.shoulder_horizontal,
                              ADDR_SHOULDER_HORIZ_LO, ADDR_SHOULDER_HORIZ_HI,
                              DEFAULT_SHOULDER_HORIZ);
    RETRIEVE_CALIBRATION_16BIT(ADDR_DATA_BITMASK_ARM, CAL_MASK_ARM_HORIZ,
                              " he %d", calibration_vals.elbow_horizontal,
                              ADDR_ELBOW_HORIZ_LO, ADDR_ELBOW_HORIZ_HI,
                              DEFAULT_ELBOW_HORIZ);
    RETRIEVE_CALIBRATION_16BIT(ADDR_DATA_BITMASK_ARM, CAL_MASK_ARM_HORIZ,
                              " hr %d", calibration_vals.rotate_horizontal,
                              ADDR_ROTATE_HORIZ_LO, ADDR_ROTATE_HORIZ_HI,
                              DEFAULT_ROTATE_HORIZ);
`


    calibration_vals.back_crab.mid = 215;


  }
  else
  {
    /* RC has never been given calibration values, use defaults */
#ifdef CALIBRATION_DEBUG
    printf("EEPROM unknown ");
#endif
    calibration_vals.front_crab.left = DEFAULT_CRAB_90_LEFT;
    calibration_vals.front_crab.mid = DEFAULT_CRAB_MIDDLE;
    calibration_vals.front_crab.right = DEFAULT_CRAB_90_RIGHT;
    calibration_vals.back_crab.left = DEFAULT_CRAB_90_LEFT;
    calibration_vals.back_crab.mid = DEFAULT_CRAB_MIDDLE;
    calibration_vals.back_crab.right = DEFAULT_CRAB_90_RIGHT;
    calibration_vals.shoulder_front = DEFAULT_SHOULDER_FRONT;
    calibration_vals.shoulder_back = DEFAULT_SHOULDER_BACK;
    calibration_vals.elbow_top = DEFAULT_ELBOW_TOP;
    calibration_vals.elbow_bottom = DEFAULT_ELBOW_BOTTOM;
    calibration_vals.rotate_left = DEFAULT_ROTATE_LEFT;
    calibration_vals.rotate_right = DEFAULT_ROTATE_RIGHT;
  }

#ifdef CALIBRATION_DEBUG
    printf("\r");
#endif

  return;
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


      case DISPLAY_POT_FRONT_CRAB:
        display_data = GET_ANALOG_VALUE_SHIFT(Analog_in_front_crab);
        break;

      case DISPLAY_POT_BACK_CRAB:
        display_data = GET_ANALOG_VALUE_SHIFT(Analog_in_back_crab);
        break;

      case DISPLAY_POT_SHOULDER:
        display_data = GET_ANALOG_VALUE_SHIFT(Analog_in_shoulder);
        break;

      case DISPLAY_POT_ELBOW:
        display_data = g_cur_elbow >> 2;
        break;

      case DISPLAY_POT_ROTATE:
        display_data = GET_ANALOG_VALUE_SHIFT(Analog_in_rotate);
        break;

      case DISPLAY_ORIENT:
        display_data = g_encoder_vals.orient >> 8;
        break;

      case DISPLAY_ENCODER_L:
        display_data = g_encoder_vals.left_back;
        break;

      case DISPLAY_ENCODER_R:
        display_data = g_encoder_vals.right_back;
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

  return;
}


