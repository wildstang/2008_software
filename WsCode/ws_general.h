/*******************************************************************************
* FILE NAME: ws_general.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_general.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_general_h_
#define __ws_general_h_

/******************************* TYPEDEFS *************************************/

/******************************** MACROS **************************************/

#define MAX_POT_DELTA 30

#define ROLLOVER(raw_val, raw_val_prev, last_diff, diff) \
  diff = raw_val - raw_val_prev; \
  \
  /* printf("rv %4d rvp %4d lbd %4d d %5d ", raw_val, raw_val_prev, last_diff, diff); */ \
  \
  if (diff > MAX_POT_DELTA) { \
    diff -= CRAB_POT_RES; \
    if (diff < -MAX_POT_DELTA) { \
      raw_val = raw_val_prev + last_diff; \
      if (raw_val < 0) \
        raw_val += CRAB_POT_RES; \
      diff = last_diff; \
      /* printf("!!! rv %d nd %d ", raw_val, diff); */ \
    } \
  } \
  else if (diff < -MAX_POT_DELTA) { \
    diff += CRAB_POT_RES; \
    if (diff > MAX_POT_DELTA) { \
      raw_val = raw_val_prev + last_diff; \
      if (raw_val > CRAB_POT_RES) \
        raw_val -= CRAB_POT_RES; \
      diff = last_diff; \
      /* printf("!!! rv %d nd %d ", raw_val, diff); */ \
    } \
  } \
  last_diff = diff; \
  raw_val_prev = raw_val;

/***************************** DEFINITIONS ************************************/
#define PUMP_LOOP_COUNT 160

#define ADC_WAIT 5

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void pump_control(void);
extern UINT8 process_adc(void);
extern INT8 calc_pot_rollover(UINT16 val, UINT16 val_prev);
extern void process_crab_pot_data(PotDataType *pot_data, INT16 raw_val, UINT16 calib_mid);

#endif /* __ws_general_h_ */
