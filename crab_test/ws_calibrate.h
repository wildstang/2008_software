/*******************************************************************************
* FILE NAME: ws_calibrate.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_calibrate.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_calibrate_h_
#define __ws_calibrate_h_

/******************************* TYPEDEFS *************************************/
typedef enum
{
  DISPLAY_NONE,
  DISPLAY_DRIVE_X,
  DISPLAY_DRIVE_Y,
  DISPLAY_CRAB_X,
  DISPLAY_SHOULDER,
  DISPLAY_ELBOW,

  DISPLAY_POT_FRONT_CRAB,
  DISPLAY_POT_BACK_CRAB,
  DISPLAY_POT_SHOULDER,
  DISPLAY_POT_ELBOW,
  DISPLAY_POT_ROTATE,

  DISPLAY_ORIENT,
  DISPLAY_ENCODER_L,
  DISPLAY_ENCODER_R
} DisplayType;

/******************************** MACROS **************************************/

//#define CALIBRATION_DEBUG

#ifdef CALIBRATION_DEBUG
#define PRINT_CAL(string, cal_val)  printf(string, cal_val)
#else
#define PRINT_CAL(string, cal_val)
#endif


#define WRITE_CALIBRATION_8BIT(bm_addr, bm, string, pot_val, addr) \
do { \
  PRINT_CAL(string, pot_val); \
  EEPROM_Write(addr, (UINT8)pot_val); \
  EEPROM_Write(bm_addr, EEPROM_Read(bm_addr) | bm); \
  sg_cal_ctr = 1; \
} while (0);

#define WRITE_CALIBRATION_16BIT(bm_addr, bm, string, pot_val, addr_lo, addr_hi) \
do { \
  PRINT_CAL(string, pot_val); \
  EEPROM_Write(addr_lo, (UINT8)pot_val); \
  EEPROM_Write(addr_hi, (UINT8)(pot_val >> 8)); \
  EEPROM_Write(bm_addr, EEPROM_Read(bm_addr) | bm); \
  sg_cal_ctr = 1; \
} while (0);



#define RETRIEVE_CALIBRATION_8BIT(bm_addr, bitmask, string, cal_val, addr, def) \
do { \
  if ((EEPROM_Read(bm_addr) & (bitmask)) != 0) \
  { \
    cal_val = EEPROM_Read(addr); \
    PRINT_CAL(string, cal_val); \
  } \
  else \
  { \
    cal_val = (def); \
  } \
} while (0);


#define RETRIEVE_CALIBRATION_16BIT(bm_addr, bitmask, string, cal_val, addr_lo, addr_hi, def) \
do { \
  if ((EEPROM_Read(bm_addr) & (bitmask)) != 0) \
  { \
    cal_val = EEPROM_Read(addr_lo); \
    cal_val |= ((UINT16)EEPROM_Read(addr_hi) << 8); \
    PRINT_CAL(string, cal_val); \
  } \
  else \
  { \
    cal_val = (def); \
  } \
} while (0);


/***************************** DEFINITIONS ************************************/
#define CALIBRATE_ZONE      70

#define CALIBRATE_KNOWN_BYTE_1  0x07
#define CALIBRATE_KNOWN_BYTE_2  0x01

#define CAL_MASK_F_CRAB_LEFT    0x01
#define CAL_MASK_F_CRAB_MID     0x02
#define CAL_MASK_F_CRAB_RIGHT   0x04
#define CAL_MASK_B_CRAB_LEFT    0x08
#define CAL_MASK_B_CRAB_MID     0x10
#define CAL_MASK_B_CRAB_RIGHT   0x20

#define CAL_MASK_SHOULDER_F     0x01
#define CAL_MASK_SHOULDER_B     0x02
#define CAL_MASK_ELBOW_TOP      0x04
#define CAL_MASK_ELBOW_BOT      0x08
#define CAL_MASK_ROTATE_L       0x10
#define CAL_MASK_ROTATE_R       0x20
#define CAL_MASK_ARM_HORIZ      0x40

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void calibrate_pots(void);
extern void retrieve_calibration(void);
extern void display_calibration(void);

#endif /* __ws_calibrate_h_ */

