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

/***************************** DEFINITIONS ************************************/
#define PUMP_LOOP_COUNT 160

#define BTN_PRESS_COUNT 40

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void pump_control(void);
extern INT16 acceleration_adjust(INT16, INT16, UINT8);
extern void deploy_ramp_n_tower(void);

#endif /* __ws_general_h_ */
