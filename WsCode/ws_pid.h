/*******************************************************************************
* FILE NAME: ws_pid.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_pid.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_pid_h_
#define __ws_pid_h_

/***************************** DEFINITIONS ************************************/
#define LAST_ERROR_SIZE    1
#define NO_INT_RESET_THRESH -1
#define NO_ERROR_THRESH -1

/******************************* TYPEDEFS *************************************/
typedef struct
{
  INT32  prop_gain;
  INT32  int_gain;
  INT32  deriv_gain;
  INT32  scale_factor;
  INT32  integral;
  INT32  max_integral;
  INT16  last_error[LAST_ERROR_SIZE];
  INT16  min_val;
  INT16  max_val;
  INT16  int_reset_thresh;
  INT8  error_thresh;
} PidValsType;

/******************************** MACROS **************************************/

/****************************** STRUCTURES ************************************/

extern UINT8 show_pid;

/************************* FUNCTION PROTOTYPES ********************************/
extern INT16 ws_pid(PidValsType *, INT32, INT32);
extern void pid_last_error_init(PidValsType *);
extern void clear_pid_vals_history(PidValsType *);

#endif /* __ws_pid_h_ */

