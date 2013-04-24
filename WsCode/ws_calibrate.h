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
  DISPLAY_NONE = 0,
  DISPLAY_DRIVE_X,
  DISPLAY_DRIVE_Y,
  DISPLAY_CRAB_X,
  DISPLAY_CRAB_Y,

  DISPLAY_POT_FRONT_CRAB,
  DISPLAY_POT_BACK_CRAB,

  DISPLAY_ORIENT,
  DISPLAY_ENCODER_L,
  DISPLAY_ENCODER_R
} DisplayType;

/******************************** MACROS **************************************/

#define CALIBRATION_DEBUG

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



#define RETRIEVE_CALIBRATION_8BIT(bm_addr, bitmask, string, cal_val, addr, state) \
do { \
  if ((EEPROM_Read(bm_addr) & (bitmask)) != 0) \
  { \
    cal_val = EEPROM_Read(addr); \
    PRINT_CAL(string, cal_val); \
  } \
  else \
  { \
    state |= CALIBRATION_BAD_RC_DATA; \
  } \
} while (0);


#define RETRIEVE_CALIBRATION_16BIT(bm_addr, bitmask, string, cal_val, addr_lo, addr_hi, state) \
do { \
  if ((EEPROM_Read(bm_addr) & (bitmask)) != 0) \
  { \
    cal_val = EEPROM_Read(addr_lo); \
    cal_val |= ((UINT16)EEPROM_Read(addr_hi) << 8); \
    PRINT_CAL(string, cal_val); \
  } \
  else \
  { \
    state |= CALIBRATION_BAD_RC_DATA; \
  } \
} while (0);



#define SAVE_CALIBRATION_16BIT(eeprom, bm_addr, bm, string, pot_val, addr_lo, addr_hi) \
do { \
  PRINT_CAL(string, pot_val); \
  eeprom[addr_lo] = (UINT8)pot_val; \
  eeprom[addr_hi] = (UINT8)(pot_val >> 8); \
  eeprom[bm_addr] = (eeprom[bm_addr] | bm); \
  sg_cal_ctr = 1; \
} while (0);

#define PARSE_CALIBRATION_16BIT(eeprom, bm_addr, bitmask, string, cal_val, addr_lo, addr_hi, state) \
do { \
  if ((eeprom[bm_addr] & (bitmask)) != 0) \
  { \
    cal_val = eeprom[addr_lo]; \
    cal_val |= ((UINT16)eeprom[addr_hi] << 8); \
    PRINT_CAL(string, cal_val); \
  } \
  else \
  { \
    state |= CALIBRATION_BAD_RC_DATA; \
  } \
} while (0);


/***************************** DEFINITIONS ************************************/
#define CALIBRATE_ZONE      70

#define CALIBRATE_KNOWN_BYTE_1  0x08
#define CALIBRATE_KNOWN_BYTE_2  0x10

/* Crab byte masks */
#define CAL_MASK_F_CRAB_MID       0x01
#define CAL_MASK_F_CRAB_LEFT      0x02
#define CAL_MASK_F_CRAB_RIGHT     0x04
#define CAL_MASK_B_CRAB_MID       0x08
#define CAL_MASK_B_CRAB_LEFT      0x10
#define CAL_MASK_B_CRAB_RIGHT     0x20

#define DEFAULT_CRAB_MIDDLE     127

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void calibrate_pots(void);
extern void write_eeprom_calibration(void);
extern void read_eeprom_calibration(void);
extern void parse_calibration(void);
extern void display_calibration(void);

#endif /* __ws_calibrate_h_ */

