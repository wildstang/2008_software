/*******************************************************************************
* FILE NAME: ws_feedback.h
*
* DESCRIPTION:
*  This is the include file which corresponds to ws_drive_input.c
*
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __ws_feedback_h_
#define __ws_feedback_h_

/******************************* TYPEDEFS *************************************/

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/
typedef enum
{
  FEEDBACK_DISABLED = 0,
  FEEDBACK_ENABLED
} FeedbackState;

/****************************** STRUCTURES ************************************/
typedef struct
{
  FeedbackState state;
  UINT8 front_pot_tgt;
  UINT8 back_pot_tgt;
} CrabTgts;

/************************* FUNCTION PROTOTYPES ********************************/
extern void feedback_interrupt(void);
extern INT8 crab_feedback(UINT8, CrabType *, int, PidValsType *);

#endif /* __ws_feedback_h_ */

