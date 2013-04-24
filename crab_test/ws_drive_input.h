/*******************************************************************************
* FILE NAME: ws_drive_input.h
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

#ifndef __ws_drive_input_h_
#define __ws_drive_input_h_

/******************************* TYPEDEFS *************************************/

/******************************** MACROS **************************************/

/***************************** DEFINITIONS ************************************/
#define DEBUGGING_THETA 0

/*** Scaling constants ***/
#define DRIVE_SCALE_NUMERATOR     1  /* these two constants make up the */
#define DRIVE_SCALE_DENOMINATOR   2  /* scaling factor for anti-turbo */

/* The next two constants are used to scale driving with the
   landing gear down */
#define DRIVE_LG_SCALE_NUMERATOR    3
#define DRIVE_LG_SCALE_DENOMINATOR  4

#define DRIVE_DEADZONE            2
#define DRIVE_ACCEL_RATE          35

#define CRAB_DEADZONE             2

#define F_CRAB_90_RIGHT           69
#define F_CRAB_CENTER             126
#define F_CRAB_90_LEFT            184

#define B_CRAB_90_RIGHT           69
#define B_CRAB_CENTER             127
#define B_CRAB_90_LEFT            190

#define DEFAULT_CRAB_90_RIGHT   (127 - 40)
#define DEFAULT_CRAB_MIDDLE     127
#define DEFAULT_CRAB_90_LEFT    (127 + 40)

#define CRAB_TURN_DISABLE_VAL   20

#define CAR_DRIVE_DEG_RANGE     10
#define CAR_DRIVE_Y_VAL         90
#define CAR_DRIVE_X_VAL         50
#define CRAB_MANU_FB_RANGE      34

#define BRAKE_MODE_LIMIT        10
#define CAR_STEER_LIMIT         10

/* distance to travel past 90 degrees */
#define CRAB_PAST_90_DEG        80
#define CRAB_LEFT_LIMIT         (-90 - (INT16)(CRAB_PAST_90_DEG))
#define CRAB_RIGHT_LIMIT        (90 + (INT16)(CRAB_PAST_90_DEG))

#define CRAB_X_LIVEZONE         50
#define CRAB_Y_LIVEZONE         90

/****************************** STRUCTURES ************************************/

/************************* FUNCTION PROTOTYPES ********************************/
extern void drive_preproc(void);
extern void crab_init(void);
extern void crab_stick_input(void);
extern void theta_correct(INT8 *, INT8 *);
extern UINT8 get_crab_pot_tgt(INT16, CrabType *);
extern void  drive_stick_input(UINT8);

#endif /* __ws_drive_input_h_ */

